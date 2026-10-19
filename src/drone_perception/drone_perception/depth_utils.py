from __future__ import annotations

import numpy as np


def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
    """Normalize common ROS depth encodings to float32 metres."""
    enc = (encoding or '').upper()
    arr = np.asarray(depth)
    if enc in {'16UC1', 'MONO16'} or arr.dtype == np.uint16:
        return arr.astype(np.float32) * 0.001
    if enc in {'32FC1', 'TYPE_32FC1'} or np.issubdtype(arr.dtype, np.floating):
        return arr.astype(np.float32)
    raise ValueError(f'Unsupported depth encoding/dtype: {encoding!r} / {arr.dtype}')


def project_rgb_pixel_to_depth(u: float, v: float, rgb_k, depth_k):
    """Project a pixel ray between co-located RGB and depth pinhole cameras.

    The PX4 OakD-Lite simulation places the RGB and depth sensors at the same
    pose but gives them different focal lengths/resolutions. Mapping the ray,
    rather than simply scaling pixel indices, respects those intrinsics.
    """
    fx_r, fy_r, cx_r, cy_r = rgb_k[0], rgb_k[4], rgb_k[2], rgb_k[5]
    fx_d, fy_d, cx_d, cy_d = depth_k[0], depth_k[4], depth_k[2], depth_k[5]
    xn = (u - cx_r) / fx_r
    yn = (v - cy_r) / fy_r
    return fx_d * xn + cx_d, fy_d * yn + cy_d


def backproject_depth_pixel(u: float, v: float, depth_m: float, depth_k):
    """Return optical-frame XYZ (x right, y down, z forward)."""
    fx, fy, cx, cy = depth_k[0], depth_k[4], depth_k[2], depth_k[5]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return float(x), float(y), float(depth_m)
