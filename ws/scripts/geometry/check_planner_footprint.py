#!/usr/bin/env python3
"""Isolated Nav2 test: 65 cm corridor, polygon straight succeeds, sideways fails.

Run in ROS with a private domain and ROS_LOCALHOST_ONLY=1, never a live domain.
"""
import os,math,time,tempfile,subprocess,json,signal
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory

assert os.environ.get('ROS_LOCALHOST_ONLY')=='1' and os.environ.get('ROS_DOMAIN_ID')=='89'
pkg=get_package_share_directory('go2_nav2');smac=get_package_share_directory('nav2_smac_planner')
geo=yaml.safe_load(open(pkg+'/config/go2_footprint.yaml'))
params={'planner_server':{'ros__parameters':{'planner_plugins':['GridBased'],'GridBased':{
 'plugin':'nav2_smac_planner/SmacPlannerLattice','lattice_filepath':smac+'/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json',
 'allow_unknown':False,'tolerance':.05,'max_planning_time':2.,'smooth_path':False}}},
 'global_costmap':{'global_costmap':{'ros__parameters':dict(geo,global_frame='map',robot_base_frame='base_link',
 resolution=.05,plugins=['static_layer','inflation_layer'],static_layer={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True},
 inflation_layer={'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.55,'cost_scaling_factor':3.})}}}
f=tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',delete=False);yaml.safe_dump(params,f);f.close()
log=open('/tmp/go2-footprint-check.log','w');children=[]
rclpy.init();n=rclpy.create_node('footprint_check');pub=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
tf=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link';t.transform.translation.x=-1.;t.transform.rotation.w=1.;tf.sendTransform(t)
m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.05;m.info.width=120;m.info.height=80;m.info.origin.position.x=-3.;m.info.origin.position.y=-2.;m.info.origin.orientation.w=1.
m.data=[0 if 34<=y<=46 else 100 for y in range(80) for x in range(120)];pub.publish(m)
client=ActionClient(n,ComputePathToPose,'compute_path_to_pose')
def wait(f):
 rclpy.spin_until_future_complete(n,f,timeout_sec=10)
 assert f.done(),'Action timed out'
 return f.result()
try:
 children.append(subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',f.name],stdout=log,stderr=log,start_new_session=True))
 children.append(subprocess.Popen(['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-p','autostart:=true','-p','node_names:=[planner_server]'],stdout=log,stderr=log,start_new_session=True))
 end=time.monotonic()+15
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if client.server_is_ready():break
 assert client.server_is_ready(),'Planner not ready'
 results=[]
 for yaw in [0.,math.pi/2]:
  g=ComputePathToPose.Goal();g.use_start=True;g.planner_id='GridBased'
  g.start.header.frame_id=g.goal.header.frame_id='map';g.start.pose.position.x=-1.;g.goal.pose.position.x=1.
  g.start.pose.orientation.z=math.sin(yaw/2);g.start.pose.orientation.w=math.cos(yaw/2);g.goal.pose.orientation.w=1.
  handle=wait(client.send_goal_async(g));assert handle.accepted
  result=wait(handle.get_result_async());results.append({'start_yaw':yaw,'status':result.status,'path_poses':len(result.result.path.poses)})
 assert results[0]['status']==4 and results[0]['path_poses']>0,results
 assert results[1]['status']!=4 and results[1]['path_poses']==0,results
 print(json.dumps({'corridor_width_m':.65,'footprint':geo,'checks':results},indent=2))
finally:
 for child in children:os.killpg(child.pid,signal.SIGINT)
 for child in children:
  try:child.wait(timeout=6)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 n.destroy_node();rclpy.shutdown();os.unlink(f.name);log.close()
