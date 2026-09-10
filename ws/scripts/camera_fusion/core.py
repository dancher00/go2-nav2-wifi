"""Geometry for read-only Point-LIO camera coloring and gated DA3 support."""
from collections import deque
import json
from pathlib import Path
import numpy as np


def pose_matrix(position, quaternion):
    q=np.asarray(quaternion,dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q)<1e-8: raise ValueError('Invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    t=np.eye(4)
    t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    t[:3,3]=position
    if not np.isfinite(t).all(): raise ValueError('Nonfinite pose')
    return t


class PoseHistory:
    def __init__(self): self.samples=deque(maxlen=200)

    def add(self, stamp, p, q):
        if self.samples and stamp<self.samples[-1][0]: raise ValueError('Odometry clock reset: start a new fusion session')
        if self.samples and stamp==self.samples[-1][0]: return
        q=np.asarray(q,dtype=float);q=q/np.linalg.norm(q)
        pose_matrix(p,q)
        self.samples.append((stamp,np.asarray(p,dtype=float),q))

    def at(self, stamp, max_gap=.2):
        for first,second in zip(self.samples,list(self.samples)[1:]):
            ta,pa,qa=first;tb,pb,qb=second
            if not ta<=stamp<=tb: continue
            dt=(tb-ta)/1e9
            if dt<=0 or dt>max_gap: raise ValueError('Odometry interpolation gap')
            fraction=(stamp-ta)/(tb-ta)
            dot=float(np.dot(qa,qb))
            if dot<0: qb=-qb;dot=-dot
            angle=np.arccos(np.clip(dot,-1,1))
            if dot>.9995: q=qa+fraction*(qb-qa)
            else: q=(np.sin((1-fraction)*angle)*qa+np.sin(fraction*angle)*qb)/np.sin(angle)
            return pose_matrix(pa+fraction*(pb-pa),q),float(np.linalg.norm(pb-pa)/dt),float(2*angle/dt)
        raise ValueError('No bracketing odometry for image time')

    def motion(self, stamp, window=.6):
        """Bound motion by pose diameter over a window, avoiding scan derivative noise."""
        samples=[v for v in self.samples if stamp-int(window*1e9)<=v[0]<=stamp]
        if len(samples)<4 or (samples[-1][0]-samples[0][0])/1e9<window*.8:
            raise ValueError('Waiting for motion observation window')
        duration=(samples[-1][0]-samples[0][0])/1e9
        points=np.array([v[1] for v in samples])
        quaternions=np.array([v[2] for v in samples])
        diameter=np.linalg.norm(points[:,None]-points[None,:],axis=2).max()
        angles=2*np.arccos(np.clip(np.abs(quaternions@quaternions.T),0,1))
        return float(diameter/duration),float(angles.max()/duration)


def load_calibration(path):
    cfg=json.loads(Path(path).read_text())
    k=np.asarray(cfg['camera_matrix'],float).reshape(3,3)
    d=np.asarray(cfg['distortion_coefficients'],float)
    if not np.isfinite(k).all() or not np.isfinite(d).all(): raise ValueError('Nonfinite calibration')
    if not np.allclose(k[2],[0,0,1]) or k[1,0]!=0 or min(k[0,0],k[1,1])<=0: raise ValueError('Invalid pinhole camera matrix')
    if cfg['distortion_model']!='plumb_bob' or len(d)!=5: raise ValueError('Only five-coefficient plumb_bob is supported')
    if min(cfg['width'],cfg['height'])<=0: raise ValueError('Invalid calibration resolution')
    sensor_base=pose_matrix(cfg['sensor_from_base'][:3],cfg['sensor_from_base'][3:])
    base_camera=pose_matrix(cfg['base_from_camera_optical'][:3],cfg['base_from_camera_optical'][3:])
    return cfg,k,d,sensor_base@base_camera


def rectify(image,k,d,width=640,height=360):
    import cv2
    # Rectified image retains the calibrated pixel projection at a smaller size.
    new_k=k.copy();new_k[0]*=width/image.shape[1];new_k[1]*=height/image.shape[0]
    mx,my=cv2.initUndistortRectifyMap(k,d,None,new_k,(width,height),cv2.CV_32FC1)
    result=cv2.remap(image,mx,my,cv2.INTER_LINEAR)
    valid=(mx>=0)&(mx<image.shape[1]-1)&(my>=0)&(my<image.shape[0]-1)
    return result,new_k,valid


def project_visible(world,t_world_camera,k,width,height,valid_pixels=None):
    """Nearest surface per pixel. Return original indices, pixels and camera Z."""
    world=np.asarray(world)
    camera=(world-t_world_camera[:3,3])@t_world_camera[:3,:3]
    valid=np.isfinite(camera).all(axis=1)&(camera[:,2]>.2)&(camera[:,2]<30)
    indices=np.flatnonzero(valid)
    camera=camera[valid]
    if not len(indices): return indices,np.empty((0,2),int),np.empty(0)
    projected=camera@k.T
    uv=np.rint(projected[:,:2]/projected[:,2,None]).astype(int)
    inside=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    indices,uv,z=indices[inside],uv[inside],camera[inside,2]
    if valid_pixels is not None and len(indices):
        good=valid_pixels[uv[:,1],uv[:,0]]
        indices,uv,z=indices[good],uv[good],z[good]
    order=np.argsort(z,kind='stable')
    _,first=np.unique(uv[order,1]*width+uv[order,0],return_index=True)
    keep=order[first]
    return indices[keep],uv[keep],z[keep]


class ColorMap:
    """Bounded voxel representatives; measured XYZ is never moved or averaged."""
    def __init__(self, voxel=.05, limit=200000):
        self.voxel=voxel;self.limit=limit;self.cells={};self.saturated=False

    def add(self, xyz, color_indices, rgb, stamp):
        xyz=np.asarray(xyz)
        keys=np.floor(xyz/self.voxel).astype(np.int64)
        colored={int(i):c for i,c in zip(color_indices,rgb)}
        for i,(key,point) in enumerate(zip(keys,xyz)):
            key=tuple(key)
            old=self.cells.get(key)
            if old is None:
                if len(self.cells)>=self.limit: self.saturated=True;continue
                self.cells[key]=[point.copy(),np.array([100,100,100],np.uint8),False,stamp]
                old=self.cells[key]
            if i in colored: old[1]=colored[i].copy();old[2]=True;old[3]=stamp

    def arrays(self):
        items=list(self.cells.values())
        if not items: return np.empty((0,3),np.float32),np.empty((0,3),np.uint8),np.empty(0,bool)
        return np.array([v[0] for v in items],np.float32),np.array([v[1] for v in items],np.uint8),np.array([v[2] for v in items],bool)


def fit_depth_scale(relative,uv,z):
    """Fit scale on spatial tiles, validate on held-out tiles; no affine shift."""
    values=relative[uv[:,1],uv[:,0]]
    valid=np.isfinite(values)&(values>0)&np.isfinite(z)&(z>.2)
    uv,z,values=uv[valid],z[valid],values[valid]
    if len(z)<60: raise ValueError('DA3 needs at least 60 projected LiDAR samples')
    tiles=(uv[:,0]//8+uv[:,1]//8)%2
    train=tiles==0;test=~train
    if min(train.sum(),test.sum())<20: raise ValueError('Insufficient spatial holdout')
    h,w=relative.shape
    if np.ptp(uv[:,0])<.25*w or np.ptp(uv[:,1])<.25*h: raise ValueError('Insufficient LiDAR image coverage')
    scale=float(np.median(z[train]/values[train]))
    error=np.abs(scale*values[test]-z[test])
    tolerance=np.maximum(.1,.05*z[test])
    if np.mean(error<=tolerance)<.8: raise ValueError('DA3 disagrees with held-out LiDAR')
    return scale,{'scale':scale,'holdout_samples':int(test.sum()),'holdout_median_abs_m':float(np.median(error)),
                  'holdout_inlier_fraction':float(np.mean(error<=tolerance))}


def supported_depth_points(relative,confidence,uv,z,k,t_world_camera,rgb,max_pixel_distance=4):
    """Only densify near LiDAR support; leave unknown space empty."""
    import cv2
    scale,report=fit_depth_scale(relative,uv,z)
    h,w=relative.shape
    support=np.ones((h,w),np.uint8)
    support[uv[:,1],uv[:,0]]=0
    distance,labels=cv2.distanceTransformWithLabels(support,cv2.DIST_L2,5,labelType=cv2.DIST_LABEL_PIXEL)
    ref=np.zeros(int(labels.max())+1,np.float32)
    ref[labels[uv[:,1],uv[:,0]]]=z
    predicted=relative*scale
    valid=(distance<=max_pixel_distance)&(labels>0)&np.isfinite(predicted)&(predicted>.2)
    valid&=np.abs(predicted-ref[labels])<=np.maximum(.1,.05*ref[labels])
    valid&=np.isfinite(confidence)&(confidence>=np.percentile(confidence,40))
    v,u=np.nonzero(valid)
    depth=predicted[v,u]
    cam=np.column_stack([(u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth])
    world=cam@t_world_camera[:3,:3].T+t_world_camera[:3,3]
    return world.astype(np.float32),rgb[v,u],report


def write_ply(path,xyz,rgb):
    data=np.empty(len(xyz),dtype=[('xyz','<f4',(3,)),('rgb','u1',(3,))])
    data['xyz']=xyz;data['rgb']=rgb
    with Path(path).open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'%len(xyz)).encode())
        f.write(data.tobytes())
