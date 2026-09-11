"""Render comparable bird's-eye projections of exported PLYs on the Jetson."""
import numpy as np,cv2,json
root='/data/review';names=['native','no_gravity','original'];clouds=[]
for name in names:
 with open(root+'/'+name+'_cloud.ply') as f:
  skip=0;n=0
  for line in f:
   skip+=1
   if line.startswith('element vertex'):n=int(line.split()[-1])
   if line.strip()=='end_header':break
 points=np.loadtxt(root+'/'+name+'_cloud.ply',skiprows=skip,max_rows=n)
 clouds.append(points)
allp=np.concatenate(clouds);lo=np.percentile(allp[:,:2],1,axis=0)-.3;hi=np.percentile(allp[:,:2],99,axis=0)+.3
scale=min(540/(hi-lo)); images=[]
for name,points in zip(names,clouds):
 im=np.full((680,600,3),245,np.uint8)
 p=points[(points[:,2]>.3)&(points[:,2]<1.8)]
 uv=((p[:,:2]-lo)*scale).astype(int);uv[:,1]=610-uv[:,1];uv[:,0]+=25
 for xy in uv:
  if 30<=xy[1]<650 and 0<=xy[0]<600:cv2.circle(im,tuple(xy),1,(65,65,65),-1)
 poses=np.loadtxt(root+'/'+name+'_poses.txt');xy=((poses[:,1:3]-lo)*scale).astype(int);xy[:,1]=610-xy[:,1];xy[:,0]+=25
 cv2.polylines(im,[xy],False,(20,150,220),2)
 cv2.putText(im,name,(20,28),cv2.FONT_HERSHEY_SIMPLEX,.8,(10,10,10),2)
 cv2.putText(im,'top view, height 0.3-1.8m',(20,663),cv2.FONT_HERSHEY_SIMPLEX,.55,(10,10,10),1)
 images.append(im)
cv2.imwrite(root+'/comparison.png',np.hstack(images))
raw=np.loadtxt(root+'/native_poses.txt');stats={}
for name in names[1:]:
 p=np.loadtxt(root+'/'+name+'_poses.txt');stats[name]={'max_position_change_m':float(np.max(np.linalg.norm(p[:,1:4]-raw[:,1:4],axis=1)))}
print(json.dumps(stats));json.dump(stats,open(root+'/comparison.json','w'),indent=2)
