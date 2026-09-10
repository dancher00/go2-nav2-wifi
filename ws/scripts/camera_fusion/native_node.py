"""Native DDS Point-LIO consumer. It never publishes poses or SLAM inputs."""
import argparse,json,os,time
from collections import deque
from pathlib import Path
import cv2,numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from core import PoseHistory,ColorMap,load_calibration,rectify,project_visible,supported_depth_points,write_ply


def atomic_npz(path,**arrays):
    temp=path.with_suffix('.tmp')
    with temp.open('wb') as f: np.savez(f,**arrays)
    os.replace(temp,path)


def xyz_from_cloud(msg):
    fields={f.name:f for f in msg.fields}
    if any(name not in fields or fields[name].datatype!=7 for name in ('x','y','z')): raise ValueError('Expected FLOAT32 XYZ')
    endian='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[n].offset for n in ('x','y','z')],'itemsize':msg.point_step})
    points=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=bytes(msg.data),strides=(msg.row_step,msg.point_step))
    xyz=np.column_stack([points[n].ravel() for n in ('x','y','z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--ipc',required=True)
    p.add_argument('--session',required=True)
    p.add_argument('--calibration',required=True)
    p.add_argument('--allow-unverified-color',action='store_true')
    p.add_argument('--da3',action='store_true')
    p.add_argument('--resume',action='store_true')
    a=p.parse_args()
    cfg,k,d,sensor_camera=load_calibration(a.calibration)
    verified=cfg['calibration_verified'] and cfg['timing_verified']
    if not verified and not a.allow_unverified_color: p.error('Calibration/timing not verified; use preview mode explicitly')
    if a.da3 and not verified: p.error('DA3 densification requires verified calibration and timing')
    ipc=Path(a.ipc);session=Path(a.session)
    session.mkdir(parents=True,exist_ok=False)
    (session/'calibration.json').write_text(json.dumps(cfg,indent=2)+'\n')
    history=PoseHistory();clouds=deque(maxlen=12);mapping=ColorMap();dense=ColorMap(voxel=.03,limit=150000)
    state={'images':0,'clouds':0,'odom':0,'colored_frames':0,'colored_points':0,'rejected':0,'last_error':'waiting for inputs','calibration_verified':verified,'da3_enabled':a.da3,'geometry_source':'Point-LIO registered measured points','profile':cfg['profile']}
    last_image=None;last_publish=0.;last_da3=None;contexts={};gid=None
    rclpy.init(args=[]);node=rclpy.create_node('pointlio_camera_fusion')
    if a.resume:
        from source_identity import identity
        previous=json.loads((ipc/'seed-source.json').read_text())
        current=identity(node,cfg)
        if current!=previous:
            node.destroy_node();rclpy.try_shutdown()
            raise RuntimeError('Cannot resume color map: SLAM publisher or frames changed')
        mapping.restore(ipc/'seed.npz')
        gid=bytes.fromhex(current['gid'])
        state['restored_points']=len(mapping.cells)
    trace=(session/'frames.jsonl').open('x',buffering=1)

    def stamp(msg): return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def odom(msg):
        if msg.header.frame_id!=cfg['map_frame'] or msg.child_frame_id!=cfg['sensor_frame']: raise RuntimeError('Unexpected Point-LIO pose frames')
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        history.add(stamp(msg),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);state['odom']+=1

    def cloud(msg):
        if msg.header.frame_id!=cfg['map_frame']: raise RuntimeError('Cloud frame differs from configured map frame')
        xyz=xyz_from_cloud(msg)
        clouds.append((stamp(msg),xyz));state['clouds']+=1
        mapping.add(xyz,[],[],stamp(msg))
        state['last_cloud_stamp_ns']=stamp(msg)

    node.create_subscription(Odometry,cfg.get('odom_topic','/lidar3d/odom'),odom,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,cfg.get('cloud_topic','/lidar3d/registered'),cloud,qos_profile_sensor_data)

    def tick():
        nonlocal last_image,last_publish,last_da3
        try:
            with np.load(ipc/'camera.npz',allow_pickle=False) as data:
                image_stamp=int(data['stamp_ns']);jpeg=data['jpeg'].copy();receipt=int(data['receipt_ns'])
            if image_stamp==last_image: return
            if not 0<=(time.monotonic_ns()-receipt)/1e9<1.5: raise ValueError('Camera input stale')
            adjusted=image_stamp+int(cfg['image_time_offset_seconds']*1e9)
            if history.samples:
                lag = (adjusted-history.samples[-1][0])/1e9
                state['camera_minus_latest_odom_seconds'] = lag
                if lag > .5:
                    raise ValueError(f'Point-LIO output behind camera by {lag:.2f} s')
            world_sensor,speed,angular=history.at(adjusted)
            if not clouds: raise ValueError('No Point-LIO registered cloud')
            cloud_stamp,xyz=min(clouds,key=lambda item:abs(item[0]-adjusted))
            difference=abs(cloud_stamp-adjusted)/1e9
            if difference>cfg['max_time_difference_seconds']: raise ValueError('Camera/cloud timestamp mismatch')
            last_image=image_stamp;state['images']+=1
            if not verified: speed,angular=history.motion(adjusted)
            state.update(speed_m_s=speed,angular_speed_rad_s=angular)
            if not verified and (speed>cfg['unverified_max_speed_m_s'] or angular>cfg['unverified_max_angular_speed_rad_s']):
                raise ValueError('Unverified timing: color update paused during motion')
            image=cv2.imdecode(jpeg,cv2.IMREAD_COLOR)
            if image is None or image.shape[:2]!=(cfg['height'],cfg['width']): raise ValueError('Camera resolution does not match calibration')
            rectified,small_k,valid=rectify(image,k,d)
            world_camera=world_sensor@sensor_camera
            indices,uv,z=project_visible(xyz,world_camera,small_k,640,360,valid)
            rgb=rectified[uv[:,1],uv[:,0],::-1]
            painted=mapping.color_view(world_camera,small_k,rectified,valid,image_stamp)
            state['map_projected_points']=painted
            state.update(colored_frames=state['colored_frames']+1,last_error='',last_image_stamp_ns=image_stamp,
                         last_pair_difference_seconds=difference,speed_m_s=speed,angular_speed_rad_s=angular,
                         projected_points=len(indices),last_success_monotonic_ns=time.monotonic_ns())
            overlay=rectified.copy()
            for pixel,depth in zip(uv[::2],z[::2]):
                color=(0,int(min(255,depth*35)),255)
                cv2.circle(overlay,tuple(pixel),1,color,-1)
            label='Point-LIO projection' if verified else 'NOMINAL CALIBRATION - stationary color preview'
            cv2.putText(overlay,label,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
            ok,encoded=cv2.imencode('.jpg',overlay)
            if ok: atomic_npz(ipc/'overlay.npz',jpeg=encoded,stamp_ns=np.int64(image_stamp))
            if state['colored_frames']<=30:
                cv2.imwrite(str(session/(str(image_stamp)+'.jpg')),image)
                atomic_npz(session/(str(image_stamp)+'.npz'),xyz=xyz,t_world_camera=world_camera,t_world_sensor=world_sensor,cloud_stamp_ns=np.int64(cloud_stamp),image_stamp_ns=np.int64(image_stamp))
            trace.write(json.dumps({key:state[key] for key in ('last_image_stamp_ns','last_pair_difference_seconds','speed_m_s','angular_speed_rad_s','projected_points')})+'\n')
            if a.da3:
                low=cv2.resize(rectified,(308,182),interpolation=cv2.INTER_AREA)
                low_k=small_k.copy();low_k[0]*=308/640;low_k[1]*=182/360
                ii,uu,zz=project_visible(xyz,world_camera,low_k,308,182)
                contexts[image_stamp]=(uu,zz,low_k,world_camera)
                contexts.pop(next(iter(contexts)),None) if len(contexts)>5 else None
                ok,encoded=cv2.imencode('.jpg',low)
                if ok: atomic_npz(ipc/'input.npz',jpeg=encoded,stamp_ns=np.int64(image_stamp),receipt_monotonic_ns=np.int64(time.monotonic_ns()))
        except FileNotFoundError:
            state['last_error']='Waiting for camera bridge'
        except ValueError as error:
            state['last_error']=str(error);state['rejected']+=1

    def consume_da3():
        nonlocal last_da3
        if not a.da3: return
        try:
            with np.load(ipc/'output.npz',allow_pickle=False) as data:
                stamp_ns=int(data['stamp_ns'])
                if stamp_ns==last_da3 or stamp_ns not in contexts: return
                last_da3=stamp_ns
                uv,z,kk,tc=contexts.pop(stamp_ns)
                xyz,rgb,report=supported_depth_points(data['predicted_depth'].squeeze(),data['confidence'].squeeze(),uv,z,kk,tc,data['rgb'])
            dense.add(xyz,np.arange(len(xyz)),rgb,stamp_ns)
            points,colors,_=dense.arrays()
            atomic_npz(ipc/'da3_map.npz',xyz=points,rgb=colors,stamp_ns=np.int64(stamp_ns))
            state['da3_validation']=report;state['da3_points']=len(points)
        except FileNotFoundError: pass
        except ValueError as error: state['da3_rejection']=str(error)

    def heartbeat():
        nonlocal gid,last_publish
        now=time.monotonic()
        if now-last_publish>=2:
            last_publish=now
            xyz,rgb,colored=mapping.arrays()
            state.update(map_points=len(xyz),colored_points=int(colored.sum()),map_capacity_reached=mapping.saturated)
            atomic_npz(ipc/'map.npz',xyz=xyz,rgb=rgb,colored=colored,stamp_ns=np.int64(state.get('last_cloud_stamp_ns',0)))
            temp=ipc/'status.tmp';temp.write_text(json.dumps(state));os.replace(temp,ipc/'status.json')
        publishers=node.get_publishers_info_by_topic(cfg.get('odom_topic','/lidar3d/odom'))
        if len(publishers)>1: raise RuntimeError('Multiple Point-LIO pose publishers; refusing ambiguous input')
        if publishers:
            current=bytes(publishers[0].endpoint_gid)
            if gid is not None and gid!=current: raise RuntimeError('Point-LIO publisher changed; start a new fusion session')
            gid=current
            state['source_gid']=gid.hex()
        if (ipc/'save.request').exists():
            (ipc/'save.request').unlink()
            prefix=session/('snapshot-'+str(time.time_ns()))
            xyz,rgb,colored=mapping.arrays()
            atomic_npz(prefix.with_suffix('.npz'),xyz=xyz,rgb=rgb,colored=colored)
            write_ply(prefix.with_suffix('.ply'),xyz,rgb)
            prefix.with_suffix('.json').write_text(json.dumps(state,indent=2)+'\n')
            state['last_saved']=str(prefix.with_suffix('.ply'))
        temp=ipc/'status.tmp';temp.write_text(json.dumps(state));os.replace(temp,ipc/'status.json')

    node.create_timer(.05,tick);node.create_timer(.1,consume_da3);node.create_timer(1,heartbeat)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        xyz,rgb,colored=mapping.arrays()
        atomic_npz(session/'colored_map.npz',xyz=xyz,rgb=rgb,colored=colored)
        write_ply(session/'colored_map.ply',xyz,rgb)
        if a.da3:
            xyz,rgb,_=dense.arrays();write_ply(session/'da3_supported.ply',xyz,rgb)
        (session/'summary.json').write_text(json.dumps(state,indent=2)+'\n')
        trace.close();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__': main()
