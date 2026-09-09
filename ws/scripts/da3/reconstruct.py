"""Bounded multi-view DA3 reconstruction of recorded frames on Jetson.

This estimates poses and intrinsics jointly; it is not calibrated LiDAR fusion.
"""
import argparse
import json
from pathlib import Path
import resource
import time
import cv2
import numpy as np
import onnxruntime as ort
from benchmark import preprocess

p=argparse.ArgumentParser()
p.add_argument('--model',required=True)
p.add_argument('--images',required=True)
p.add_argument('--output',required=True)
p.add_argument('--views',type=int,default=8)
p.add_argument('--motion-keyframes',action='store_true')
a=p.parse_args()
if not 2<=a.views<=24: p.error('Use 2 to 24 views for this bounded Jetson experiment')
paths=sorted(Path(a.images).glob('*.jpg'))
if len(paths)<2: p.error('At least two frames are required')
input_count=len(paths)
if a.motion_keyframes:
    selected=[]
    previous=None
    for path in paths:
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if image is None: continue
        small=cv2.resize(image,(80,45)).astype(np.float32)
        if previous is None or float(np.mean(np.abs(small-previous)))>=3:
            selected.append(path)
            previous=small
    if paths[-1] not in selected: selected.append(paths[-1])
    paths=selected
paths=[paths[i] for i in np.linspace(0,len(paths)-1,min(a.views,len(paths))).astype(int)]
out=Path(a.output)
out.mkdir(parents=True,exist_ok=False)
inputs=[preprocess(path,182,308) for path in paths]
x=np.concatenate([item[0] for item in inputs],axis=1)
options=ort.SessionOptions()
options.intra_op_num_threads=2
options.inter_op_num_threads=1
started=time.perf_counter()
session=ort.InferenceSession(a.model,sess_options=options,providers=['CPUExecutionProvider'])
loaded=time.perf_counter()
pred=dict(zip([o.name for o in session.get_outputs()],session.run(None,{'pixel_values':x})))
finished=time.perf_counter()
if not all(np.isfinite(v).all() for v in pred.values()): raise ValueError('Nonfinite reconstruction')
np.savez_compressed(out/'prediction.npz',**pred,rgb=np.stack([item[1] for item in inputs]))
clouds=[]
for i,(_,rgb) in enumerate(inputs):
    depth=pred['predicted_depth'][0,i]
    confidence=pred['confidence'][0,i]
    k=pred['intrinsics'][0,i]
    w2c=pred['extrinsics'][0,i]
    if min(k[0,0],k[1,1])<=0: raise ValueError('Invalid inferred focal length')
    v,u=np.mgrid[0:182:2,0:308:2]
    z=depth[::2,::2]
    valid=(z>0)&(confidence[::2,::2]>=np.percentile(confidence,40))
    cam=np.stack([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z],axis=-1)[valid]
    # OpenCV world-to-camera: Xc = R Xw + t.
    world=(cam-w2c[:,3])@w2c[:,:3]
    colors=rgb[::2,::2][valid]
    clouds.append((world.astype('<f4'),colors))
count=sum(len(points) for points,_ in clouds)
with (out/'relative_scene.ply').open('wb') as f:
    f.write(('ply\nformat binary_little_endian 1.0\ncomment DA3 predicted geometry; relative units; uncalibrated\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%count).encode())
    dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,))])
    for points,colors in clouds:
        vertices=np.empty(len(points),dtype=dtype)
        vertices['xyz']=points
        vertices['rgb']=colors
        f.write(vertices.tobytes())
centers=np.stack([-w[:,:3].T@w[:,3] for w in pred['extrinsics'][0]])
report={'sources':[str(p.resolve()) for p in paths],'input_frames':input_count,'motion_keyframes':a.motion_keyframes,'views':len(paths),'points':count,
        'backend':'onnxruntime CPU on Jetson','load_seconds':loaded-started,'inference_seconds':finished-loaded,
        'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
        'units':'relative; not metres','calibrated':False,'fusion_enabled':False,
        'camera_center_extent':np.ptp(centers,axis=0).tolist(),
        'limitation':'No distortion correction, LiDAR alignment, persistent SLAM or map accuracy validation'}
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
