import json,math,sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws/scripts/camera_fusion'))
from core import PoseHistory,ColorMap,pose_matrix,project_visible,fit_depth_scale,load_calibration

class FusionTests(unittest.TestCase):
    def test_interpolates_translation_and_rotation_without_extrapolation(self):
        history=PoseHistory();history.add(10**9,[0,0,0],[0,0,0,1]);history.add(1100000000,[1,0,0],[0,0,math.sqrt(.5),math.sqrt(.5)])
        pose,_,_=history.at(1050000000)
        np.testing.assert_allclose(pose[:3,3],[.5,0,0])
        np.testing.assert_allclose(pose[:3,:3]@[1,0,0],[math.sqrt(.5),math.sqrt(.5),0],atol=1e-6)
        with self.assertRaises(ValueError): history.at(900000000)
        with self.assertRaises(ValueError): history.add(1,[0,0,0],[0,0,0,1])

    def test_window_rejects_motion_but_tolerates_millimeter_pose_noise(self):
        for velocity,expected in [(0,False),(.2,True)]:
            h=PoseHistory()
            for i in range(31):
                h.add(i*20000000,[velocity*i*.02+(.001 if i%2 else -.001),0,0],[0,0,0,1])
            speed,angular=h.motion(600000000)
            self.assertEqual(speed>.05,expected)
            self.assertAlmostEqual(angular,0)
        short=PoseHistory();short.add(0,[0,0,0],[0,0,0,1])
        with self.assertRaises(ValueError): short.motion(0)

    def test_geometry_accumulates_without_color_and_retains_existing_color(self):
        m=ColorMap()
        m.add(np.array([[0,0,1]]),[0],np.array([[255,0,0]],np.uint8),1)
        m.add(np.array([[0,0,1],[1,0,1]]),[],[],2)
        xyz,rgb,colored=m.arrays()
        self.assertEqual(len(xyz),2)
        self.assertEqual(colored.tolist(),[True,False])
        self.assertEqual(rgb[0].tolist(),[255,0,0])

    def test_projection_uses_inverse_camera_pose_and_nearest_surface(self):
        k=np.array([[100,0,50],[0,100,50],[0,0,1]])
        pose=pose_matrix([1,0,0],[0,0,0,1])
        points=np.array([[1,0,2],[1,0,4],[1,0,-2],[20,0,1]])
        ids,uv,z=project_visible(points,pose,k,100,100)
        self.assertEqual(ids.tolist(),[0]);self.assertEqual(uv.tolist(),[[50,50]]);self.assertEqual(z.tolist(),[2])

    def test_coloring_does_not_move_measured_geometry_and_is_bounded(self):
        mapping=ColorMap(voxel=1,limit=1)
        mapping.add(np.array([[.1,0,1]]),[0],np.array([[255,0,0]],np.uint8),1)
        mapping.add(np.array([[.2,0,1],[10,0,1]]),[0],np.array([[0,255,0]],np.uint8),2)
        xyz,rgb,colored=mapping.arrays()
        np.testing.assert_allclose(xyz,[[.1,0,1]])
        self.assertEqual(rgb.tolist(),[[0,255,0]]);self.assertTrue(colored[0]);self.assertTrue(mapping.saturated)

    def test_da3_scale_has_independent_spatial_holdout(self):
        v,u=np.mgrid[0:64:4,0:64:4];uv=np.column_stack([u.ravel(),v.ravel()])
        relative=np.ones((64,64));z=np.full(len(uv),2.)
        scale,stats=fit_depth_scale(relative,uv,z)
        self.assertEqual(scale,2);self.assertEqual(stats['holdout_median_abs_m'],0)
        z[((uv[:,0]//8+uv[:,1]//8)%2)==1]=4
        with self.assertRaises(ValueError): fit_depth_scale(relative,uv,z)

    def test_rejects_noncanonical_camera_matrix(self):
        path=Path(__file__).resolve().parents[1]/'go2_nav2/config/camera_fusion_preview.json'
        cfg=json.loads(path.read_text());cfg['camera_matrix'][2][0]=1
        with tempfile.TemporaryDirectory() as folder:
            invalid=Path(folder)/'bad.json';invalid.write_text(json.dumps(cfg))
            with self.assertRaises(ValueError): load_calibration(invalid)

if __name__=='__main__':unittest.main()
