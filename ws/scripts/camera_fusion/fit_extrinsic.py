"""Experimental local NID extrinsic fit; produces candidates, never deploys them.
NID objective follows Koide et al., ICRA 2023 (arXiv:2302.05094).
This NumPy/SciPy implementation is not the authors' full calibration toolbox.
Intrinsics and exposure timing are NOT estimated by this program.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import minimize
from core import load_calibration, rectify, project_visible


def normalized_information_distance(a,b,bins=16):
    if len(a)<100: return 1.
    x=np.clip(a,0,1)*(bins-1);y=np.clip(b,0,1)*(bins-1)
    ix=x.astype(int);iy=y.astype(int);fx=x-ix;fy=y-iy
    hist=np.zeros(bins*bins)
    for dx,wx in [(0,1-fx),(1,fx)]:
        for dy,wy in [(0,1-fy),(1,fy)]:
            ids=np.minimum(ix+dx,bins-1)*bins+np.minimum(iy+dy,bins-1)
            hist+=np.bincount(ids,weights=wx*wy,minlength=bins*bins)
    hist=hist.reshape(bins,bins);hist/=hist.sum()
    def entropy(v):
        v=v[v>0];return -np.sum(v*np.log(v))
    joint=entropy(hist)
    if joint<1e-8:return 1.
    return float(2-(entropy(hist.sum(0))+entropy(hist.sum(1)))/joint)


def perturb(initial,parameters):
    delta=np.eye(4)
    delta[:3,:3]=cv2.Rodrigues(np.asarray(parameters[3:])*np.deg2rad(2))[0]
    delta[:3,3]=np.asarray(parameters[:3])*.05
    return initial@delta


def sample_image(xyz,t,k,gray):
    camera=(xyz-t[:3,3])@t[:3,:3]
    projected=camera@k.T
    uv=projected[:,:2]/np.maximum(camera[:,2,None],1e-6)
    mask=(camera[:,2]>.2)&(uv[:,0]>=1)&(uv[:,1]>=1)&(uv[:,0]<gray.shape[1]-2)&(uv[:,1]<gray.shape[0]-2)
    uv=uv[mask];u=uv[:,0].astype(int);v=uv[:,1].astype(int)
    du=uv[:,0]-u;dv=uv[:,1]-v
    samples=(1-du)*(1-dv)*gray[v,u]+du*(1-dv)*gray[v,u+1]+(1-du)*dv*gray[v+1,u]+du*dv*gray[v+1,u+1]
    return samples,mask


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('session',type=Path)
    args=p.parse_args();folder=args.session
    capture=json.loads((folder/'capture.json').read_text())
    flow=capture.get('image_displacement_px_at_640x360_p50_p90')
    if flow is None or flow[0]>.5 or flow[1]>2:
        raise SystemExit('Image movement too large for stationary calibration')
    cfg,k,d,t=load_calibration(folder/'calibration.json')
    paths=sorted(x for x in folder.glob('*.npz') if x.stem.isdigit())
    points=[];intensities=[]
    for path in paths:
        with np.load(path,allow_pickle=False) as data:
            points.append(data['xyz']);intensities.append(data['intensity'])
    xyz=np.concatenate(points);intensity=np.concatenate(intensities)
    _,unique=np.unique(np.floor(xyz/.025).astype(np.int64),axis=0,return_index=True)
    xyz=xyz[unique];intensity=intensity[unique]
    image=cv2.imread(str(paths[len(paths)//2].with_suffix('.jpg')))
    rectified,kk,valid=rectify(image,k,d,960,540)
    ids,uv,z=project_visible(xyz,t,kk,960,540,valid)
    xyz=xyz[ids];intensity=intensity[ids]
    low,high=np.percentile(intensity,[2,98])
    if high-low<10: raise SystemExit('Insufficient LiDAR reflectance variation')
    intensity=np.clip((intensity-low)/(high-low),0,1)
    gray=cv2.cvtColor(rectified,cv2.COLOR_BGR2GRAY).astype(float)/255
    train=((uv[:,0]//32+uv[:,1]//32)%2)==0
    if min(train.sum(),(~train).sum())<500:raise SystemExit('Insufficient spatial validation support')
    def loss(parameters,selection):
        if np.max(np.abs(parameters))>3:
            return 2+float(np.maximum(np.abs(parameters)-3,0).sum())
        samples,mask=sample_image(xyz[selection],perturb(t,parameters),kk,gray)
        if mask.mean()<.9:return 2+1-mask.mean()
        return normalized_information_distance(samples,intensity[selection][mask])+.05*(1-mask.mean())
    initial_train=loss(np.zeros(6),train);initial_test=loss(np.zeros(6),~train)
    results=[]
    for rotation in [0,-.75,.75]:
        start=np.zeros(6);start[4]=rotation
        result=minimize(lambda x:loss(x,train),start,method='Powell',
                        options={'maxiter':35,'maxfev':2500,'xtol':.003,'ftol':1e-5})
        candidate=perturb(t,result.x)
        report={'parameters':result.x.tolist(),'train_nid':float(result.fun),
                'holdout_nid':loss(result.x,~train),'evaluations':int(result.nfev),
                'optimizer_success':bool(result.success),'sensor_from_camera':candidate.tolist()}
        results.append(report);print(json.dumps(report),flush=True)
    best=min(results,key=lambda r:r['train_nid'])
    boundary=max(abs(v) for v in best['parameters'])>2.8
    report={'method':'local soft-histogram NID, fixed nominal intrinsics',
            'source':'https://arxiv.org/abs/2302.05094','initial_train_nid':initial_train,
            'initial_holdout_nid':initial_test,'training_points':int(train.sum()),
            'heldout_points':int((~train).sum()),'candidates':results,'best':best,
            'hit_search_boundary':boundary,
            'holdout_improved':bool(best['holdout_nid']<initial_test),
            'candidate_rejected':bool(boundary or best['holdout_nid']>=initial_test),
            'independent_view_verified':False,
            'intrinsics_verified':False,'timing_verified':False,
            'calibration_verified':False,
            'warning':'Lower objective is not proof of correct calibration; another viewpoint and geometric validation are required.'}
    (folder/'extrinsic-fit.json').write_text(json.dumps(report,indent=2)+'\n')
    after=np.array(best['sensor_from_camera'])
    panels=[]
    for transform,label in [(t,'BEFORE: nominal extrinsic'),(after,'CANDIDATE ONLY: not verified')]:
        _,pixels,_=project_visible(xyz,transform,kk,960,540,valid)
        panel=rectified.copy()
        # Original intensity attached to each projected point, no fitted colors.
        indices,_,_=project_visible(xyz,transform,kk,960,540,valid)
        for pixel,level in zip(pixels,intensity[indices]):
            color=(int(255*level),int(255*(1-level)),255)
            cv2.circle(panel,tuple(pixel),1,color,-1)
        cv2.putText(panel,label,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
        panels.append(panel)
    cv2.imwrite(str(folder/'extrinsic-candidate.jpg'),np.vstack(panels))
    print(json.dumps({k:report[k] for k in ['initial_train_nid','initial_holdout_nid','hit_search_boundary']}),flush=True)


if __name__=='__main__':main()
