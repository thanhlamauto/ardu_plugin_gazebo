import unittest
from types import SimpleNamespace

import numpy as np

from scripts.evaluate_monocular_lidar_depth import (
    error_metrics,project_lidar_to_image,sample_bilinear,
)
from scripts.log_lidar_gt_gz import decode_cloud
from scripts.evaluate_multiframe_ekf_lidar import ekf_rotation_enu,stable_lidar_match
from scipy.spatial import cKDTree


class LidarDepthEvaluationTests(unittest.TestCase):
    def test_colocated_lidar_projection_uses_axial_depth(self):
        points = np.array([[7.,0.,0.],[7.,-1.,1.],[7.,1.,-1.],[-1.,0.,0.]])
        pixels,depth = project_lidar_to_image(points,640,360)
        self.assertEqual(len(depth),3)
        np.testing.assert_allclose(depth,[7.,7.,7.])
        self.assertAlmostEqual(pixels[0,0],319.5)
        self.assertGreater(pixels[1,0],pixels[0,0])
        self.assertLess(pixels[1,1],pixels[0,1])

    def test_bilinear_sampling_and_metric_error(self):
        image = np.array([[2.,4.],[6.,8.]])
        self.assertAlmostEqual(sample_bilinear(image,np.array([[.5,.5]]))[0],5.)
        metrics=error_metrics(np.array([5.,7.]),np.array([4.,8.]))
        self.assertAlmostEqual(metrics['mae_m'],1.)
        self.assertAlmostEqual(metrics['bias_m'],0.)

    def test_lidar_cloud_field_stride(self):
        raw=np.array([[7.,0.,0.,11.],[5.,-1.,1.,22.]],dtype='<f4')
        fields=[SimpleNamespace(name=name,datatype=6,offset=i*4)
                for i,name in enumerate(('x','y','z'))]
        msg=SimpleNamespace(field=fields,width=2,height=1,point_step=16,
                            row_step=32,is_bigendian=False,data=raw.tobytes())
        np.testing.assert_allclose(decode_cloud(msg),raw[:,:3])

    def test_ekf_ned_heading_east_maps_body_to_enu(self):
        np.testing.assert_allclose(ekf_rotation_enu(0.,0.,np.pi/2),np.eye(3),atol=1e-12)

    def test_sparse_lidar_match_rejects_depth_edges(self):
        pixel=np.array([100.,100.])
        image_points=np.array([[100.,100.],[101.,100.],[99.,100.],[100.,101.]])
        tree=cKDTree(image_points)
        self.assertIsNotNone(stable_lidar_match(tree,np.array([6.,6.1,5.9,6.]),pixel))
        self.assertIsNone(stable_lidar_match(tree,np.array([6.,6.1,5.9,23.]),pixel))


if __name__=='__main__':
    unittest.main()
