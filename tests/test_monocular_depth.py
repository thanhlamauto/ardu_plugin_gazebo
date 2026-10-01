import unittest
import numpy as np
from mppi_ardupilot.monocular_depth import depth_to_points, depth_metrics


class MonocularGeometryTests(unittest.TestCase):
    def test_camera_axis_and_mount_translation(self):
        points = depth_to_points(np.full((3, 3), 4.), horizontal_fov=np.pi/2, stride=1)
        np.testing.assert_allclose(points[4], [4.09, 0, 0], atol=1e-6)
        self.assertGreater(points[0, 1], 0)  # image left -> FLU left
        self.assertGreater(points[0, 2], 0)  # image top -> FLU up
        self.assertLess(points[-1, 1], 0)
        self.assertLess(points[-1, 2], 0)

    def test_invalid_and_out_of_range_depth_excluded(self):
        depth = np.array([[np.nan, np.inf, -1, 0, .1, 26, 4.]])
        points = depth_to_points(depth, stride=1)
        self.assertEqual(points.shape, (1, 3))
        self.assertAlmostEqual(float(points[0,0]), 4.09, places=5)

    def test_metrics_do_not_align_scale_to_ground_truth(self):
        metrics = depth_metrics(np.full((2,2), 8.), np.full((2,2), 4.))
        self.assertEqual(metrics['abs_rel'], 1.)
        self.assertEqual(metrics['rmse_m'], 4.)
        self.assertEqual(metrics['delta1'], 0.)

class GazeboContractTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        from pathlib import Path
        spec = importlib.util.spec_from_file_location('monocular_gz',
            Path(__file__).resolve().parents[1]/'scripts/monocular_depth_gz.py')
        self.backend = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.backend)

    def test_padded_rgb_rows(self):
        from gz.msgs10.image_pb2 import Image, RGB_INT8
        msg = Image(width=1, height=2, step=4, pixel_format_type=RGB_INT8,
                    data=bytes([10,20,30,0,40,50,60,0]))
        np.testing.assert_array_equal(self.backend.decode_image(msg), [[[10,20,30]],[[40,50,60]]])

    def test_cloud_preserves_acquisition_stamp_and_flu_frame(self):
        from gz.msgs10.header_pb2 import Header
        header = Header(); header.stamp.sec = 12; header.stamp.nsec = 34
        points = np.array([[4., -1., 2.]], dtype='<f4')
        msg = self.backend.cloud_message(points, header)
        self.assertEqual(msg.header.stamp, header.stamp)
        self.assertEqual(msg.header.data[0].value[0], 'sensor_suite_link')
        self.assertEqual(msg.row_step, 12)
        np.testing.assert_array_equal(np.frombuffer(msg.data,dtype='<f4').reshape(-1,3),points)

class MonocularOnlyConfigurationTests(unittest.TestCase):
    def test_simulated_model_has_no_depth_or_lidar_even_in_includes(self):
        from pathlib import Path
        import xml.etree.ElementTree as ET
        root = Path(__file__).resolve().parents[1]
        seen=set()
        def sensor_types(name):
            if name in seen:return []
            seen.add(name)
            model=ET.parse(root/'models'/name/'model.sdf')
            sensors=[e.get('type') for e in model.findall('.//sensor')]
            for uri in model.findall('.//include/uri'):
                self.assertTrue(uri.text.startswith('model://'))
                sensors.extend(sensor_types(uri.text[len('model://'):]))
            return sensors
        types=sensor_types('iris_with_monocular_camera')
        self.assertEqual(types.count('camera'),1)
        self.assertFalse(any('lidar' in t or 'depth' in t for t in types))

    def test_only_camera_obstacles_and_manual_route_in_experiment(self):
        from pathlib import Path
        import yaml
        root=Path(__file__).resolve().parents[1]
        cfg=yaml.safe_load((root/'config/monocular_mppi.yaml').read_text())
        self.assertEqual(cfg['lidar_topic'],'/perception/obstacles_camera')
        self.assertNotIn('known_geometry_sdf',cfg)
        self.assertNotIn('global_map_sdf',cfg)
        self.assertGreater(cfg['recovery_speed_m_s'],0)
        self.assertLessEqual(cfg['vmax'],1.)



class CameraLocalMapTests(unittest.TestCase):
    def test_pose_transform_and_sensor_mount(self):
        from mppi_ardupilot.camera_local_map import CameraLocalMap
        grid=CameraLocalMap(min_hits=1,voxel_size=.1,altitude=3.)
        points=grid.update(np.array([[4.,0,0]]),np.array([1,2,3]),np.eye(3),1.)
        np.testing.assert_allclose(points[0],[5.05,2.05,3.15],atol=1e-5)

    def test_duplicate_pixels_do_not_fake_temporal_support(self):
        from mppi_ardupilot.camera_local_map import CameraLocalMap
        grid=CameraLocalMap(min_hits=2,altitude=3.)
        cloud=np.tile([4.,0,0],(10,1))
        self.assertEqual(len(grid.update(cloud,[0,0,3],np.eye(3),1)),0)
        self.assertEqual(len(grid.update(cloud,[0,0,3],np.eye(3),1)),0)
        self.assertEqual(len(grid.update(cloud,[0,0,3],np.eye(3),1.2)),1)
        self.assertEqual(len(grid.update(np.empty((0,3)),[0,0,3],np.eye(3),5)),0)

    def test_floor_and_ceiling_do_not_block_planar_flight(self):
        from mppi_ardupilot.camera_local_map import CameraLocalMap
        grid=CameraLocalMap(min_hits=1,altitude=3.)
        points=grid.update([[4,0,-3],[4,0,3],[4,0,0]],[0,0,3],np.eye(3),1)
        self.assertEqual(len(points),1)

if __name__ == "__main__":
    unittest.main()
