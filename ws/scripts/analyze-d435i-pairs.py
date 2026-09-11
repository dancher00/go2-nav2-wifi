"""Offline RGB-D/odometry consistency probe. Run on Jetson after rtabmap-export."""
import argparse
import cv2,numpy as np,json,os
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',nargs='?',default='/data/review');root=parser.parse_args().directory
poses=np.loadtxt(root+'/raw_camera_poses.txt')
opt=np.loadtxt(root+'/optimized_poses.txt')
raw=np.loadtxt(root+'/raw_poses.txt')
def matrix(row):
 x,y,z,w=row[4:8];T=np.eye(4)
 T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
 T[:3,3]=row[1:4];return T
orb=cv2.ORB_create(2500); bf=cv2.BFMatcher(cv2.NORM_HAMMING)
cache={}; results=[]
for row in poses:
 i=int(row[-1]);im=cv2.imread(f'{root}/raw_rgb/{i}.jpg',0)
 if im is None:im=cv2.imread(f'{root}/raw_rgb/{i}.png',0)
 dep=cv2.imread(f'{root}/raw_depth/{i}.png',-1)
 f=cv2.FileStorage(f'{root}/raw_calib/{i}.yaml',cv2.FILE_STORAGE_READ);K=np.array([f.getNode('camera_matrix').getNode('data').at(k).real() for k in range(9)]).reshape(3,3);f.release()
 kp,des=orb.detectAndCompute(im,None);cache[i]=(kp,des,dep,K,float(cv2.Laplacian(im,cv2.CV_64F).var()))
for ra,rb in zip(poses[:-1],poses[1:]):
 i,j=int(ra[-1]),int(rb[-1]);ka,da,depth,K,blur=cache[i];kb,db,_,_,blurb=cache[j]
 if da is None or db is None:continue
 matches=[p[0] for p in bf.knnMatch(da,db,k=2) if len(p)==2 and p[0].distance<.72*p[1].distance]
 obj=[];img=[]
 for m in matches:
  u,v=ka[m.queryIdx].pt;z=float(depth[round(v),round(u)])*.001
  if not .3<z<6:continue
  obj.append([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]);img.append(kb[m.trainIdx].pt)
 if len(obj)<20:continue
 ok,r,t,ins=cv2.solvePnPRansac(np.float32(obj),np.float32(img),K,None,iterationsCount=300,reprojectionError=2.0,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
 if not ok or ins is None:continue
 V=np.eye(4);V[:3,:3]=cv2.Rodrigues(r)[0];V[:3,3]=t.ravel()
 positive_fraction=float(np.mean((np.float64(obj)@V[:3,:3].T+V[:3,3])[:,2]>0))
 N=np.linalg.inv(matrix(rb))@matrix(ra)
 error=np.linalg.inv(V)@N
 result={'a':i,'b':j,'positive_depth_fraction':positive_fraction,'matches':len(obj),'inliers':len(ins),'dt':rb[0]-ra[0], 'rotation_error_deg':float(np.linalg.norm(cv2.Rodrigues(error[:3,:3])[0])*180/np.pi),'translation_error_m':float(np.linalg.norm(error[:3,3])),'visual_translation_m':float(np.linalg.norm(t)), 'native_translation_m':float(np.linalg.norm(N[:3,3])),'blur':min(blur,blurb),'visual':V.tolist(),'native':N.tolist()}
 results.append(result)
good=[x for x in results if x['inliers']>=40 and x['inliers']/x['matches']>.4 and x['dt']<2 and x['positive_depth_fraction']>.95]
summary={'pairs':len(results),'strong_pairs':len(good),'rotation_error_deg_percentiles':(np.percentile([x['rotation_error_deg'] for x in good],[10,50,90]).tolist() if good else []),'translation_error_m_percentiles':(np.percentile([x['translation_error_m'] for x in good],[10,50,90]).tolist() if good else []),'optimization_position_change_max_m':float(np.max(np.linalg.norm(opt[:,1:4]-raw[:,1:4],axis=1)))}
json.dump({'summary':summary,'pairs':results},open(root+'/pair-check.json','w'),indent=2);print(json.dumps(summary));print('worst',[(x['a'],x['b'],round(x['rotation_error_deg'],1),round(x['translation_error_m'],2)) for x in sorted(good,key=lambda x:-x['rotation_error_deg'])[:8]])
