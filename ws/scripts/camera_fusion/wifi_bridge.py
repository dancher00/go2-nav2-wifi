"""Camera domain bridge; all image/point processing remains on Jetson."""
import argparse,json,os,time
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,Image,PointCloud2,PointField
from std_msgs.msg import String,Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker
p=argparse.ArgumentParser();p.add_argument('--ipc',required=True);a=p.parse_args();ipc=Path(a.ipc)
rclpy.init(args=[]);node=rclpy.create_node('camera_fusion_wifi')
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
publishers={name:node.create_publisher(PointCloud2,topic,qos) for name,topic in [('map','/camera_fusion/colored_map'),('da3_map','/camera_fusion/da3_supported')]}
status_pub=node.create_publisher(String,'/camera_fusion/status',1)
overlay_pub=node.create_publisher(CompressedImage,'/camera_fusion/overlay/compressed',qos_profile_sensor_data)
raw_overlay_pub=node.create_publisher(Image,'/camera_fusion/projection_overlay',qos_profile_sensor_data)
text_pub=node.create_publisher(Marker,'/camera_fusion/status_text',1)
last_input=0.;mtimes={}
calibration=json.loads((ipc/'calibration.json').read_text());frame=calibration['map_frame']
broadcaster=StaticTransformBroadcaster(node);transform=TransformStamped()
transform.header.frame_id=frame;transform.child_frame_id='camera_fusion_reference';transform.transform.rotation.w=1.
broadcaster.sendTransform(transform)

def receive(msg):
    global last_input
    now=time.monotonic()
    if now-last_input<1: return
    last_input=now
    temp=ipc/'camera.tmp'
    with temp.open('wb') as f:
        np.savez(f,jpeg=np.asarray(msg.data,np.uint8),stamp_ns=np.int64(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec),receipt_ns=np.int64(time.monotonic_ns()))
    os.replace(temp,ipc/'camera.npz')
node.create_subscription(CompressedImage,'/go2_stock_camera/image/compressed',receive,qos_profile_sensor_data)

def publish():
    for name,pub in publishers.items():
        path=ipc/(name+'.npz')
        if not path.exists() or mtimes.get(name)==path.stat().st_mtime_ns: continue
        mtimes[name]=path.stat().st_mtime_ns
        with np.load(path,allow_pickle=False) as data:
            xyz=data['xyz'];rgb=data['rgb'].astype(np.uint32);stamp=int(data['stamp_ns'])
        points=np.empty((len(xyz),4),dtype='<f4');points[:,:3]=xyz
        points[:,3]=((rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]).view('<f4')
        header=Header(frame_id=frame);header.stamp.sec,header.stamp.nanosec=divmod(stamp,10**9)
        cloud=PointCloud2(header=header,height=1,width=len(points),is_bigendian=False,point_step=16,row_step=len(points)*16,is_dense=True,data=points.tobytes())
        cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z','rgb'])]
        pub.publish(cloud)
    path=ipc/'overlay.npz'
    if path.exists() and mtimes.get('overlay')!=path.stat().st_mtime_ns:
        mtimes['overlay']=path.stat().st_mtime_ns
        with np.load(path,allow_pickle=False) as data:
            image=CompressedImage(format='bgr8; jpeg compressed bgr8',data=data['jpeg'].tobytes())
            image.header.stamp.sec,image.header.stamp.nanosec=divmod(int(data['stamp_ns']),10**9)
            overlay_pub.publish(image)
            if raw_overlay_pub.get_subscription_count():
                pixels=cv2.imdecode(data['jpeg'],cv2.IMREAD_COLOR)
                if pixels is not None:
                    raw_overlay_pub.publish(Image(header=image.header,height=pixels.shape[0],width=pixels.shape[1],encoding='bgr8',is_bigendian=False,step=pixels.shape[1]*3,data=pixels.tobytes()))
node.create_timer(.2,publish)

def status():
    try: state=json.loads((ipc/'status.json').read_text())
    except FileNotFoundError: state={'last_error':'Waiting for native Point-LIO consumer'}
    last=state.get('last_success_monotonic_ns')
    state['color_age_seconds']=(time.monotonic_ns()-last)/1e9 if last else None
    state['live']=last is not None and state['color_age_seconds']<3
    state['camera_bridge_age_seconds']=time.monotonic()-last_input if last_input else None
    status_pub.publish(String(data=json.dumps(state)))
    label=Marker(header=Header(frame_id=frame),ns='camera_fusion_status',id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
    label.pose.orientation.w=1.;label.pose.position.z=1.5;label.scale.z=.12
    label.color.r=1.;label.color.g=.7;label.color.a=1.
    label.text='Point-LIO RGB map' if state.get('calibration_verified') else 'Nominal camera calibration: stationary color preview'
    if not state['live']: label.text+='\nColor updates paused: '+state.get('last_error','stale input')
    text_pub.publish(label)
node.create_timer(1,status)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.destroy_node();rclpy.try_shutdown()
