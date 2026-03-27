import numpy as np
from scipy.spatial.transform import Rotation as R

matrix_3x3 = np.array([
    [-0.02319628,  0.99960674, -0.01575728],
    [-0.99969834, -0.0233199,  -0.00770726],
    [-0.00807168,  0.01557375,  0.99984614]
])

try:
    rot = R.from_matrix(matrix_3x3)
except AttributeError:
    rot = R.from_dcm(matrix_3x3)

quat = rot.as_quat()
x, y, z = -0.05236895, 0.03666473, -0.07956362

print(f"\nKopiraj ovu komandu:")
print(f"rosrun tf2_ros static_transform_publisher {x} {y} {z} {quat[0]} {quat[1]} {quat[2]} {quat[3]} fr3_EE camera_link_proxy")