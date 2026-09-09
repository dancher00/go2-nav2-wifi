"""Relative closure/planarity diagnostics; no ground-truth accuracy claim."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
p=argparse.ArgumentParser()
p.add_argument('scene')
a=p.parse_args()
scene=Path(a.scene)
f=np.load(scene/'prediction.npz');w=f['extrinsics'][0]
c=np.stack([-e[:,:3].T@e[:,3] for e in w])
delta=np.linalg.norm(c[-1]-c[0]);diam=max(np.linalg.norm(c-v,axis=1).max() for v in c)
rot=w[-1,:,:3]@w[0,:,:3].T
depth=f['predicted_depth'][0,0];k=f['intrinsics'][0,0]
v,u=np.mgrid[20:100,75:235];z=depth[20:100,75:235]
wall=np.stack([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z],axis=-1).reshape(-1,3)
_,_,vt=np.linalg.svd(wall-wall.mean(axis=0),full_matrices=False)
residual=(wall-wall.mean(axis=0))@vt[-1]
report={'endpoint_distance_relative':float(delta),'trajectory_diameter_relative':float(diam),
        'endpoint_fraction_of_diameter':float(delta/max(diam,1e-6)),
        'endpoint_rotation_degrees':float(np.degrees(np.arccos(np.clip((np.trace(rot)-1)/2,-1,1)))),
        'first_wall_plane_rms_over_median_depth':float(np.sqrt(np.mean(residual**2))/np.median(z)),
        'limitation':'Endpoint is not surveyed ground truth; wall ROI assumes this particular walk starts facing the white wall.'}
(scene/'diagnostics.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
with (scene/'relative_scene.ply').open('rb') as stream:
    while stream.readline()!=b'end_header\n':
        if stream.tell()>4096: raise ValueError('Invalid PLY header')
    vertices=np.frombuffer(stream.read(),dtype=np.dtype([('xyz','<f4',(3,)),('rgb','u1',(3,))]))
xyz=vertices['xyz'];colors=vertices['rgb'][:,::-1]
view=np.array([[.707,0,.707],[.354,-.866,-.354],[.612,.5,-.612]])
panels=[]
for title,rotation in [('Top view, relative units',np.array([[1,0,0],[0,0,1],[0,-1,0]])),('Oblique view, relative units',view)]:
    q=xyz@rotation.T
    low,high=np.percentile(q[:,:2],[1,99],axis=0)
    scale=min(550/max(high[0]-low[0],1e-6),450/max(high[1]-low[1],1e-6))
    uv=np.rint((q[:,:2]-(low+high)/2)*scale+[300,260]).astype(int)
    keep=(uv[:,0]>=0)&(uv[:,0]<600)&(uv[:,1]>=35)&(uv[:,1]<500)
    order=np.argsort(q[:,2]);order=order[keep[order]]
    im=np.full((500,600,3),35,np.uint8)
    im[uv[order,1],uv[order,0]]=colors[order]
    path=c@rotation.T
    path=np.rint((path[:,:2]-(low+high)/2)*scale+[300,260]).astype(int)
    cv2.polylines(im,[path],False,(0,100,255),1)
    cv2.putText(im,title,(10,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    panels.append(im)
cv2.imwrite(str(scene/'scene-review.jpg'),np.concatenate(panels,axis=1))
