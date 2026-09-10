"""Fit plumb-bob camera intrinsics and validate on held-out checkerboard views."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def errors(object_points, corners, k, d):
    # Pose is fitted to half of the board corners, evaluated on the other half.
    keep=np.arange(len(object_points))%2==0
    result=[]
    for detected in corners:
        ok,r,t=cv2.solvePnP(object_points[keep],detected[keep],k,d)
        if not ok:result.append(float('inf'));continue
        projected,_=cv2.projectPoints(object_points[~keep],r,t,k,d)
        result.append(float(np.sqrt(np.mean(np.sum((projected-detected[~keep])**2,axis=2)))))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--reference',type=Path,required=True)
    args=parser.parse_args();cv2.setNumThreads(1)
    folder=args.session;meta=json.loads((folder/'capture.json').read_text())
    files=sorted(folder.glob('*.npz'))
    if len(files)<20:raise SystemExit('Need at least 20 different board views')
    columns,rows=meta['pattern_inner_corners'];size=tuple(meta['image_size'])
    obj=np.zeros((columns*rows,3),np.float32)
    obj[:,:2]=np.mgrid[0:columns,0:rows].T.reshape(-1,2)*meta['printed_square_m']
    cfg=json.loads(args.reference.read_text());k=np.array(cfg['camera_matrix'],float);d=np.array(cfg['distortion_coefficients'],float)
    # Re-detect on original pixels; reject images where the more robust detector
    # cannot resolve the whole board. Selection is independent of fit residuals.
    accepted=[];rejected=[]
    for i,path in enumerate(files):
        gray=cv2.imread(str(path.with_suffix('.jpg')),cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.shape[::-1]!=size:
            rejected.append(path.name);continue
        found,detected=cv2.findChessboardCornersSB(
            gray,(columns,rows),flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            rejected.append(path.name);continue
        accepted.append((i,detected.astype(np.float32)))
    corners=[c for _,c in accepted]
    train=[c for i,c in accepted if i%5!=0]
    test=[c for i,c in accepted if i%5==0]
    if len(train)<12 or len(test)<4:
        raise SystemExit('Insufficient sharp views: need at least 12 training and 4 held-out views')
    flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_FIX_K3
    rms,fitted_k,fitted_d,rs,ts,std_intr,std_ext,per_view=cv2.calibrateCameraExtended(
        [obj]*len(train),train,size,k.copy(),d.copy(),flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,200,1e-10))
    baseline=errors(obj,test,k,d);heldout=errors(obj,test,fitted_k,fitted_d)
    all_corners=np.concatenate(corners).reshape(-1,2)
    coverage=np.ptp(all_corners,axis=0)/np.array(size)
    valid=bool(np.median(heldout)<.8 and np.median(heldout)<.8*np.median(baseline)
               and min(coverage)>.5 and np.max(std_intr[:2].ravel()/np.diag(fitted_k)[:2])<.1
               and .3*size[0]<fitted_k[0,0]<3*size[0]
               and .3*size[1]<fitted_k[1,1]<4*size[1]
               and .25*size[0]<fitted_k[0,2]<.75*size[0]
               and .25*size[1]<fitted_k[1,2]<.75*size[1])
    report={'accepted_files':[files[i].name for i,_ in accepted],'rejected_files':rejected,
            'corner_detector':'findChessboardCornersSB full resolution',
            'training_views':len(train),'heldout_views':len(test),'training_rms_px':float(rms),
            'baseline_heldout_rms_px':baseline,'candidate_heldout_rms_px':heldout,
            'image_coverage_xy':coverage.tolist(),'intrinsic_std':std_intr.ravel().tolist(),
            'camera_matrix':fitted_k.tolist(),'distortion_coefficients':fitted_d.ravel().tolist(),
            'intrinsics_verified':valid,'extrinsics_verified':False,'timing_verified':False,
            'calibration_verified':False,'model':'plumb_bob; k3 fixed at zero',
            'note':'Printed square metric size is assumed 30 mm; verify before metric extrinsic use.'}
    (folder/'intrinsics-fit.json').write_text(json.dumps(report,indent=2)+'\n')
    if valid:
        cfg.update(camera_matrix=fitted_k.tolist(),distortion_coefficients=fitted_d.ravel().tolist(),
                   intrinsics_verified=True,extrinsics_verified=False,calibration_verified=False,timing_verified=False,
                   profile='measured-intrinsics-nominal-extrinsics',
                   source=f'Checkerboard {columns}x{rows} internal corners; {len(train)} training / {len(test)} held-out views. Extrinsics still nominal.')
        (folder/'camera-intrinsics.json').write_text(json.dumps(cfg,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
