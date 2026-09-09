"""Jetson-only inference worker. A single-slot IPC prevents frame backlog."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
import cv2
import numpy as np
from benchmark import TrtEngine, preprocess_bgr

p = argparse.ArgumentParser()
p.add_argument('--ipc', required=True)
p.add_argument('--model', required=True)
p.add_argument('--backend', choices=['ort', 'trt'], default='ort')
p.add_argument('--duration', type=float, default=900)
a = p.parse_args()
ipc = Path(a.ipc)
stop = False

def shutdown(*_):
    global stop
    stop = True
signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)
if a.backend == 'ort':
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    model = ort.InferenceSession(a.model, sess_options=options, providers=['CPUExecutionProvider'])
    names = [out.name for out in model.get_outputs()]
    run = lambda x: dict(zip(names, model.run(None, {'pixel_values': x})))
else:
    model = TrtEngine(a.model)
    run = model.run
last = None
started = time.monotonic()
while not stop and time.monotonic()-started < a.duration:
    try:
        with np.load(ipc/'input.npz', allow_pickle=False) as packet:
            stamp = int(packet['stamp_ns'])
            receipt = int(packet['receipt_monotonic_ns'])
            jpeg = packet['jpeg'].copy()
    except FileNotFoundError:
        time.sleep(.05)
        continue
    if stamp == last or not 0 <= (time.monotonic_ns()-receipt)/1e9 < 1.5:
        time.sleep(.05)
        continue
    last = stamp
    started_frame = time.monotonic()
    bgr = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
    if bgr is None: continue
    x, rgb = preprocess_bgr(bgr)
    pred = run(x)
    if not all(np.isfinite(v).all() for v in pred.values()):
        raise RuntimeError('Nonfinite DA3 output')
    elapsed = time.monotonic()-started_frame
    temp = ipc/'output.tmp'
    with temp.open('wb') as f:
        np.savez(f, **pred, rgb=rgb, stamp_ns=np.int64(stamp), receipt_monotonic_ns=np.int64(receipt),
                 inference_seconds=np.float64(elapsed))
    os.replace(str(temp), str(ipc/'output.npz'))
    print(json.dumps({'stamp_ns':stamp,'seconds':elapsed,'backend':a.backend}),flush=True)
if a.backend == 'trt': model.close()
