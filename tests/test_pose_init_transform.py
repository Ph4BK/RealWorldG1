"""Run with system Python after sourcing ROS; no robot or ROS graph needed."""
import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from g1_pose_init import compose

class TransformTest(unittest.TestCase):
    def test_upside_down_example(self):
        T=compose(3,-2,np.pi/2,1.26,np.diag([1,-1,-1]))
        np.testing.assert_allclose(T@np.array([0,0,1.26,1]),[3,-2,0,1],atol=1e-12)
        np.testing.assert_allclose(T@np.array([1,0,0,1]),[3,-1,1.26,1],atol=1e-12)
    def test_upstream_euler_order(self):
        L=R.from_euler('xyz',[3.1,.08,.02]).as_matrix()
        T=compose(2,-1,1.3,1.26,L)
        a=R.from_matrix(T[:3,:3]).as_euler('XYZ')
        reconstructed=R.from_euler('x',a[0]).as_matrix()@R.from_euler('y',a[1]).as_matrix()@R.from_euler('z',a[2]).as_matrix()
        np.testing.assert_allclose(reconstructed,T[:3,:3],atol=1e-12)
if __name__=='__main__':unittest.main()
