"""Read-only RViz publisher for one relative DA3 reconstruction."""
import argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster
p=argparse.ArgumentParser()
p.add_argument('scene')
a=p.parse_args()
scene=Path(a.scene)
with (scene/'relative_scene.ply').open('rb') as f:
    count=None
    for _ in range(30):
        line=f.readline()
        if line.startswith(b'element vertex '): count=int(line.split()[-1])
        if line==b'end_header\n': break
    else: raise ValueError('Unsupported PLY header')
    if count is None: raise ValueError('Missing vertex count')
    vertices=np.frombuffer(f.read(),dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,))]))
    if len(vertices)!=count: raise ValueError('Truncated PLY')
points=np.empty((count,4),dtype='<f4')
points[:,:3]=vertices['xyz']
colors=vertices['rgb'].astype(np.uint32)
points[:,3]=((colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]).view('<f4')
with np.load(scene/'prediction.npz',allow_pickle=False) as packet:
    centers=[-w[:,:3].T@w[:,3] for w in packet['extrinsics'][0]]
rclpy.init(args=[])
node=rclpy.create_node('go2_da3_scene')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
pub=node.create_publisher(PointCloud2,'/go2_da3/reconstruction',qos)
pathpub=node.create_publisher(Marker,'/go2_da3/reconstruction_path',qos)
transform=TransformStamped()
transform.header.frame_id='go2_da3_scene'
transform.child_frame_id='go2_da3_scene_optical'
transform.transform.rotation.x=-.5
transform.transform.rotation.y=.5
transform.transform.rotation.z=-.5
transform.transform.rotation.w=.5
broadcaster=StaticTransformBroadcaster(node)
broadcaster.sendTransform(transform)
header=Header(frame_id='go2_da3_scene_optical')
cloud=PointCloud2(header=header,height=1,width=count,is_bigendian=False,point_step=16,row_step=count*16,is_dense=True,data=points.tobytes())
cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z','rgb'])]
path=Marker(header=header,ns='DA3 estimated relative trajectory',id=0,type=Marker.LINE_STRIP,action=Marker.ADD)
path.pose.orientation.w=1.
path.scale.x=.01
path.color.r=1.
path.color.g=.3
path.color.a=1.
path.points=[Point(x=float(c[0]),y=float(c[1]),z=float(c[2])) for c in centers]
pub.publish(cloud)
pathpub.publish(path)
print('Published relative scene, vertices=%d; no connection to robot map TF'%count,flush=True)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
