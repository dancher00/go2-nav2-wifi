"""Own the Jetson GPU worker for the lifetime of the fusion container."""
import argparse,signal,subprocess,time
p=argparse.ArgumentParser();p.add_argument('--python',required=True);p.add_argument('--worker',required=True);p.add_argument('--engine',required=True);p.add_argument('--ipc',required=True);a=p.parse_args()
stop=False

def shutdown(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
child=subprocess.Popen([a.python,'-u',a.worker,'--ipc',a.ipc,'--backend','trt','--model',a.engine,'--duration','86400'])
try:
    while not stop and child.poll() is None:
        result=subprocess.run(['docker','inspect','--format','{{.State.Running}}','go2-camera-fusion'],capture_output=True,text=True)
        if result.returncode or result.stdout.strip()!='true': break
        time.sleep(1)
finally:
    if child.poll() is None: child.terminate()
    try: child.wait(timeout=10)
    except subprocess.TimeoutExpired: child.kill();child.wait()
    if child.returncode not in (0,-15,-2):
        subprocess.run(['docker','stop','go2-camera-fusion'],check=False)
