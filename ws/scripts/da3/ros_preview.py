"""Publish uncalibrated DA3 preview in a separate relative coordinate frame."""
import argparse
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

p = argparse.ArgumentParser()
p.add_argument('--ipc', required=True)
p.add_argument('--records', required=True)
a = p.parse_args()
ipc = Path(a.ipc)
rclpy.init(args=[])
node = rclpy.create_node('go2_da3_preview')
# Coordinate convention only; deliberately disconnected from the robot/map TF.
tf_broadcaster = StaticTransformBroadcaster(node)
optical_tf = TransformStamped()
optical_tf.header.stamp = node.get_clock().now().to_msg()
optical_tf.header.frame_id = 'go2_da3_relative'
optical_tf.child_frame_id = 'go2_da3_relative_optical'
optical_tf.transform.rotation.x = -.5
optical_tf.transform.rotation.y = .5
optical_tf.transform.rotation.z = -.5
optical_tf.transform.rotation.w = .5
tf_broadcaster.sendTransform(optical_tf)
qos = qos_profile_sensor_data
cloud_pub = node.create_publisher(PointCloud2, '/go2_da3/relative_points', qos)
depth_pub = node.create_publisher(Image, '/go2_da3/relative_depth', qos)
preview_pub = node.create_publisher(CompressedImage, '/go2_da3/preview/compressed', qos)
status_pub = node.create_publisher(String, '/go2_da3/status', 1)
state = {'last_input':0., 'last_output':None, 'last_published_monotonic_ns':None, 'frames':0, 'error':''}
record = {'directory': None, 'until': 0., 'bytes': 0, 'frames': 0}

def receive(msg):
    now = time.monotonic()
    if now-state['last_input'] < 1: return
    state['last_input'] = now
    stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    if (ipc/'record.request').exists():
        (ipc/'record.request').unlink()
        directory = Path(a.records)/str(time.time_ns())
        directory.mkdir(parents=True, exist_ok=False)
        record.update(directory=directory, until=now+120, bytes=0, frames=0)
    if record['directory'] and now < record['until'] and record['bytes']+len(msg.data) < 256*1024*1024:
        (record['directory']/(str(stamp)+'.jpg')).write_bytes(bytes(msg.data))
        with (record['directory']/'frames.jsonl').open('a') as f:
            f.write(json.dumps({'stamp_ns':stamp,'exposure_ns':None,'timestamp_basis':'Jetson JPEG receipt'})+'\n')
        record['bytes'] += len(msg.data)
        record['frames'] += 1
    with (ipc/'input.tmp').open('wb') as f:
        np.savez(f,jpeg=np.asarray(msg.data,dtype=np.uint8),stamp_ns=np.int64(stamp),receipt_monotonic_ns=np.int64(time.monotonic_ns()))
    os.replace(str(ipc/'input.tmp'),str(ipc/'input.npz'))

sub = node.create_subscription(CompressedImage,'/go2_stock_camera/image/compressed',receive,qos)

def publish():
    try:
        with np.load(ipc/'output.npz',allow_pickle=False) as packet:
            stamp=int(packet['stamp_ns'])
            receipt=int(packet['receipt_monotonic_ns'])
            if stamp == state['last_output'] or not 0 <= (time.monotonic_ns()-receipt)/1e9 < 2.5: return
            depth=packet['predicted_depth'].squeeze().astype(np.float32)
            confidence=packet['confidence'].squeeze()
            rgb=packet['rgb']
            k=packet['intrinsics'].reshape(3,3)
            latency=float(packet['inference_seconds'])
        if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0: raise ValueError('Invalid inferred intrinsics')
        header=Image().header
        header.stamp.sec,header.stamp.nanosec=divmod(stamp,10**9)
        header.frame_id='go2_da3_relative_optical'
        image=Image(header=header,height=depth.shape[0],width=depth.shape[1],encoding='32FC1',is_bigendian=False,step=depth.shape[1]*4,data=depth.tobytes())
        depth_pub.publish(image)
        v,u=np.mgrid[0:depth.shape[0]:3,0:depth.shape[1]:3]
        z=depth[::3,::3]
        colors=rgb[::3,::3].astype(np.uint32)
        mask=np.isfinite(z)&(z>0)&(confidence[::3,::3]>=np.percentile(confidence,10))
        points=np.empty((int(mask.sum()),4),dtype='<f4')
        points[:,0]=((u-k[0,2])*z/k[0,0])[mask]
        points[:,1]=((v-k[1,2])*z/k[1,1])[mask]
        points[:,2]=z[mask]
        packed=((colors[:,:,0]<<16)|(colors[:,:,1]<<8)|colors[:,:,2])[mask]
        points[:,3]=packed.view('<f4')
        cloud=PointCloud2(header=header,height=1,width=len(points),is_bigendian=False,point_step=16,row_step=len(points)*16,is_dense=True,data=points.tobytes())
        cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z','rgb'])]
        cloud_pub.publish(cloud)
        lo,hi=np.percentile(depth,[2,98])
        heat=cv2.applyColorMap(np.clip((depth-lo)/max(hi-lo,1e-6)*255,0,255).astype(np.uint8),cv2.COLORMAP_VIRIDIS)
        panel=np.concatenate([cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),heat],axis=1)
        cv2.putText(panel,'RGB | DA3 relative depth (not metres)',(5,15),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
        ok,jpeg=cv2.imencode('.jpg',panel)
        if ok: preview_pub.publish(CompressedImage(header=header,format='bgr8; jpeg compressed bgr8',data=jpeg.tobytes()))
        state.update(last_output=stamp,last_published_monotonic_ns=time.monotonic_ns(),frames=state['frames']+1,error='',inference_seconds=latency)
    except FileNotFoundError:
        pass
    except Exception as error:
        state['error']=str(error)

node.create_timer(.05,publish)
def status():
    last=state['last_published_monotonic_ns']
    age=(time.monotonic_ns()-last)/1e9 if last else None
    status_pub.publish(String(data=json.dumps(dict(state,age_seconds=age,live=age is not None and age<2.5,units='relative; not metres',calibrated=False,fusion_enabled=False,recorded_frames=record['frames'],record_directory=str(record['directory']),recording=record['directory'] is not None and time.monotonic()<record['until'] and record['bytes']<256*1024*1024))))
node.create_timer(1,status)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
