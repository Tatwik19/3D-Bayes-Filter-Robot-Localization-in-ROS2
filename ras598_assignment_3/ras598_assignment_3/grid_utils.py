import math
import numpy as np


def wrap_angle_deg(angle):
    """Wrap angle to [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def real_to_grid(x, y, theta_deg, world_min, world_max, spatial_res, theta_res, grid_dim, theta_dim):
    """Convert real-world pose to grid indices (ix, iy, ith).

    Grid x increases left to right (same as world x).
    Grid y index 0 is the top of the world (world y = world_max).
    """
    ix  = int(math.floor((x - world_min) / spatial_res))
    iy  = int(math.floor((world_max - y) / spatial_res))
    ith = int(math.floor((theta_deg % 360.0) / theta_res))

    ix  = int(np.clip(ix,  0, grid_dim  - 1))
    iy  = int(np.clip(iy,  0, grid_dim  - 1))
    ith = ith % theta_dim

    return ix, iy, ith


def grid_to_real(world_min, world_max, spatial_res, theta_res, grid_dim, theta_dim):
    """Precompute real-world centre coordinates for every grid cell.

    Returns three arrays (rx, ry, rth) of shape (grid_dim, grid_dim, theta_dim).
    """
    ix  = np.arange(grid_dim)
    iy  = np.arange(grid_dim)
    ith = np.arange(theta_dim)

    gx, gy, gth = np.meshgrid(ix, iy, ith, indexing='ij')

    rx  = world_min + (gx + 0.5) * spatial_res
    ry  = world_max - (gy + 0.5) * spatial_res
    rth = (gth + 0.5) * theta_res

    return rx, ry, rth


def roll_no_wrap_2d(arr, shift_x, shift_y):
    """Shift a 2D array by (shift_x, shift_y) without wrapping at borders.

    Probability that leaves the grid is lost (absorbed by walls).
    """
    result = np.zeros_like(arr)

    src_x0 = max(0, -shift_x) 
    src_x1 = min(arr.shape[0], arr.shape[0] - shift_x)
    dst_x0 = max(0,  shift_x)
    dst_x1 = min(arr.shape[0], arr.shape[0] + shift_x)

    src_y0 = max(0, -shift_y);  src_y1 = min(arr.shape[1], arr.shape[1] - shift_y)
    dst_y0 = max(0,  shift_y);  dst_y1 = min(arr.shape[1], arr.shape[1] + shift_y)

    if src_x1 > src_x0 and src_y1 > src_y0:
        result[dst_x0:dst_x1, dst_y0:dst_y1] = arr[src_x0:src_x1, src_y0:src_y1]

    return result