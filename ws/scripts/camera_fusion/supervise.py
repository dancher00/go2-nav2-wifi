"""Own only this add-on's two ROS processes; exit if either child fails."""
import os,signal,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parent
stop=False

def shutdown(*_):
    global stop
    stop=True
signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
common=['--ipc','/ipc']
native=['python3','-u',str(root/'native_node.py'),*common,'--session','/data/session','--calibration','/ipc/calibration.json']
if os.environ.get('FUSION_PREVIEW')=='1': native.append('--allow-unverified-color')
if os.environ.get('FUSION_RESUME')=='1': native.append('--resume')
if os.environ.get('FUSION_DA3')=='1': native.append('--da3')
children=[]
try:
    for command,domain,config in [(native,'0','native.xml'),(['python3','-u',str(root/'wifi_bridge.py'),*common],os.environ['FUSION_DOMAIN'],'wifi.xml'),(['python3','-u',str(root/'camera_preview.py')],os.environ['FUSION_DOMAIN'],'wifi.xml')]:
        env=dict(os.environ,ROS_DOMAIN_ID=domain,CYCLONEDDS_URI='file:///ipc/'+config)
        children.append(subprocess.Popen(command,env=env))
    while not stop and all(p.poll() is None for p in children): time.sleep(.2)
finally:
    for child in children:
        if child.poll() is None: child.send_signal(signal.SIGINT)
    for child in children:
        try: child.wait(timeout=15)
        except subprocess.TimeoutExpired: child.kill();child.wait()
if not stop and any(p.returncode for p in children): raise SystemExit(1)
