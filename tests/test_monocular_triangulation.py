import unittest
import numpy as np
from mppi_ardupilot.monocular_triangulation import triangulate_matches,OPTICAL_TO_FLU,CAMERA_OFFSET_FLU

class TriangulationTests(unittest.TestCase):
    def test_known_metric_baseline_recovers_world_points_under_rotation(self):
        k=np.array([[381.,0,319.5],[0,381.,179.5],[0,0,1]])
        world=np.array([[7.,0,3.],[8.,1.,3.2],[6.,-1.,2.8]])
        poses=[(np.array([0.,0,3.]),np.eye(3)),(np.array([.1,.4,3.]),np.array([[np.cos(.08),-np.sin(.08),0],[np.sin(.08),np.cos(.08),0],[0,0,1]]))]
        pixels=[]
        for position,rotation in poses:
            center=position+rotation@CAMERA_OFFSET_FLU
            optical=(world-center)@rotation@OPTICAL_TO_FLU
            q=optical@k.T;pixels.append(q[:,:2]/q[:,2,None])
        points,valid=triangulate_matches(*pixels,*poses[0],*poses[1],k)
        self.assertTrue(valid.all());np.testing.assert_allclose(points,world,atol=1e-5)
    def test_no_baseline_is_unknown_not_obstacle_or_free(self):
        pixels=np.array([[320.,180.],[300.,150.]])
        points,valid=triangulate_matches(pixels,pixels,[0,0,3],np.eye(3),[0,0,3],np.eye(3),np.eye(3))
        self.assertEqual(len(points),0);self.assertFalse(valid.any())

if __name__=='__main__':unittest.main()
