"""Collect diverse 8x5 checkerboard views from the camera IPC on Jetson.
Does not alter any calibration. Printed board: 9x6 squares, 30 mm per square.
"""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np


def detect(gray):
    # Detection at half resolution; corner refinement at original resolution.
    small=cv2.resize(gray,None,fx=.5,fy=.5,interpolation=cv2.INTER_AREA)
    ok,corners=cv2.findChessboardCorners(small,(8,5),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK)
    if not ok:return None
    corners*=2
    return cv2.cornerSubPix(gray,corners,(7,7),(-1,-1),
                           (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ipc',type=Path,default=Path('/ipc'))
    p.add_argument('--output',type=Path)
    p.add_argument('--views',type=int,default=25)
    p.add_argument('--timeout',type=float,default=300)
    p.add_argument('--self-test',action='store_true')
    args=p.parse_args();cv2.setNumThreads(1)
    if args.self_test:
        image=np.full((800,1100),255,np.uint8)
        for row in range(6):
            for col in range(9):
                if (row+col)%2==0:image[100+row*90:100+(row+1)*90,100+col*90:100+(col+1)*90]=0
        corners=detect(image)
        assert corners is not None and corners.shape==(40,1,2)
        assert detect(np.full((800,1100),127,np.uint8)) is None
        print('Checkerboard detection and blank-image rejection passed');return
    if args.output is None:p.error('--output required')
    args.output.mkdir(parents=True,exist_ok=False)
    accepted=[];last=None;started=time.monotonic();size=None
    try:
        while len(accepted)<args.views and time.monotonic()-started<args.timeout:
            try:
                with np.load(args.ipc/'camera.npz',allow_pickle=False) as data:
                    stamp=int(data['stamp_ns']);jpeg=data['jpeg'].copy();receipt=int(data['receipt_ns'])
            except FileNotFoundError:time.sleep(.1);continue
            if stamp==last or (time.monotonic_ns()-receipt)/1e9>1.5:
                time.sleep(.05);continue
            last=stamp
            gray=cv2.imdecode(jpeg,cv2.IMREAD_GRAYSCALE)
            if gray is None:continue
            size=[gray.shape[1],gray.shape[0]]
            corners=detect(gray)
            if corners is None:continue
            # Avoid near-duplicates; account for reversed chessboard indexing.
            if accepted and min(min(np.sqrt(np.mean((corners-c)**2)),np.sqrt(np.mean((corners-c[::-1])**2))) for c in accepted)<20:
                continue
            accepted.append(corners)
            jpeg.tofile(str(args.output/(str(stamp)+'.jpg')))
            np.savez(args.output/(str(stamp)+'.npz'),corners=corners,stamp_ns=np.int64(stamp))
            print(f'Accepted {len(accepted)}/{args.views} checkerboard views',flush=True)
    finally:
        report={'views':len(accepted),'image_size':size,'pattern_inner_corners':[8,5],
                'printed_square_m':.03,'complete':len(accepted)>=args.views,
                'calibration_verified':False,'note':'Square metric size must be measured for screen display or scaled printing.'}
        (args.output/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    if len(accepted)<args.views:raise SystemExit('Incomplete capture: more diverse board views required')


if __name__=='__main__':main()
