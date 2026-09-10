"""Own an isolated instance of the verified Point-LIO baseline; no motion nodes."""
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
root=Path('/work')
result=Path('/data')
(result/'PCD').mkdir(exist_ok=True)
config=yaml.safe_load((root/'pointlio_go2.yaml').read_text())
parameters=config['/**']['ros__parameters']
parameters['common']['lid_topic']='/camera_fusion/lio/cloud_sync'
parameters['common']['imu_topic']='/camera_fusion/lio/imu_sync'
parameters['mapping']['output_frame']='camera_fusion_map'
(result/'pointlio.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
clock=['python3','/work/go2_nav2/cloud_stamp_sync.py','--ros-args','-r','__node:=camera_fusion_sensor_clock',
       '-p','cloud_in:=/utlidar/cloud','-p','cloud_out:=/camera_fusion/lio/cloud_sync',
       '-p','odom_in:=/utlidar/robot_odom','-p','odom_out:=/camera_fusion/lio/clock_reference',
       '-p','imu_in:=/utlidar/imu','-p','imu_out:=/camera_fusion/lio/imu_sync',
       '-p','trace_path:=/data/sensor-clock.jsonl']
lio=['/opt/go2-lidar3d/point_lio_unilidar/lib/point_lio_unilidar/pointlio_mapping',
     '--ros-args','-r','__node:=camera_fusion_lio','--params-file','/data/pointlio.yaml']
for source,target in [('cloud_registered','registered'),('Laser_map','initial_map'),
                      ('aft_mapped_to_init','odom'),('path','path'),
                      ('cloud_registered_body','body'),('cloud_effected','effective')]:
    lio+=['-r','/'+source+':=/camera_fusion/lio/'+target]
stop=False
def shutdown(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,shutdown)
signal.signal(signal.SIGTERM,shutdown)
children=[]
try:
    for command in [clock,lio]:
        children.append(subprocess.Popen(command,env=dict(os.environ,PYTHONPATH='/work:'+os.environ.get('PYTHONPATH',''),GO2_LIDAR3D_RESULT_DIR='/data')))
    while not stop and all(p.poll() is None for p in children): time.sleep(.2)
finally:
    for child in children:
        if child.poll() is None: child.send_signal(signal.SIGINT)
    for child in children:
        try: child.wait(timeout=40)
        except subprocess.TimeoutExpired: child.kill();child.wait()
if not stop and any(p.returncode for p in children): raise SystemExit(1)
