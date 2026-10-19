import numpy as np
from drone_perception.depth_utils import (
    backproject_depth_pixel, depth_to_meters, project_rgb_pixel_to_depth)


def test_depth_units_uint16_mm():
    arr = np.array([[1000, 2500]], dtype=np.uint16)
    out = depth_to_meters(arr, '16UC1')
    assert np.allclose(out, [[1.0, 2.5]])


def test_depth_units_float_metres():
    arr = np.array([[1.0, 2.5]], dtype=np.float32)
    out = depth_to_meters(arr, '32FC1')
    assert np.allclose(out, arr)


def test_ray_projection_identity_intrinsics():
    k = (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0)
    u, v = project_rgb_pixel_to_depth(400.0, 260.0, k, k)
    assert abs(u - 400.0) < 1e-6
    assert abs(v - 260.0) < 1e-6


def test_backproject_center():
    k = (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0)
    x, y, z = backproject_depth_pixel(320.0, 240.0, 3.0, k)
    assert (x, y, z) == (0.0, 0.0, 3.0)
