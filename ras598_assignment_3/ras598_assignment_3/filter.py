import math
import numpy as np
from scipy.ndimage import gaussian_filter

from .grid_utils import real_to_grid, grid_to_real, roll_no_wrap_2d, wrap_angle_deg
from . import config as cfg


class HistogramFilter:
    def __init__(self, landmarks):
        self.landmarks = landmarks

        self.grid_dim  = int(cfg.WORLD_SIZE / cfg.SPATIAL_RES)
        self.theta_dim = int(360.0 / cfg.THETA_RES)

        # Precompute real-world cell centres once
        self.rx, self.ry, self.rth = grid_to_real(
            cfg.WORLD_MIN, cfg.WORLD_MAX,
            cfg.SPATIAL_RES, cfg.THETA_RES,
            self.grid_dim, self.theta_dim,
        )

        self.belief = None
        self.init_belief(cfg.INITIAL_POSE)

    # ------------------------------------------------------------------
    # Belief management
    # ------------------------------------------------------------------

    def init_belief(self, pose=None):
        """Initialise belief. Pass a [x, y, theta_deg] pose for a localised
        start, or None for a uniform distribution over the whole grid."""
        shape = (self.grid_dim, self.grid_dim, self.theta_dim)

        if pose is None:
            self.belief = np.ones(shape, dtype=np.float64)
        else:
            self.belief = np.zeros(shape, dtype=np.float64)
            ix, iy, ith = real_to_grid(
                pose[0], pose[1], pose[2],
                cfg.WORLD_MIN, cfg.WORLD_MAX,
                cfg.SPATIAL_RES, cfg.THETA_RES,
                self.grid_dim, self.theta_dim,
            )
            self.belief[ix, iy, ith] = 1.0
            # Spread a little so we don't start with a single impossible point mass
            self.belief = gaussian_filter(
                self.belief, sigma=(1.0, 1.0, 0.6),
                mode=('constant', 'constant', 'wrap'),
            )

        self._normalize()

    def _normalize(self):
        total = np.sum(self.belief)
        if total <= cfg.EPS or not np.isfinite(total):
            # Belief collapsed — reset to uniform so the filter can recover
            self.belief = np.ones_like(self.belief) / self.belief.size
        else:
            self.belief /= total

    # ------------------------------------------------------------------
    # Predict step (Turn-Go-Turn odometry model)
    # ------------------------------------------------------------------

    def predict(self, d_rot1, d_trans, d_rot2):
        """Shift the belief according to the Turn-Go-Turn decomposition."""

        # --- First rotation ---
        self._rotate_belief(d_rot1)

        # --- Translation: each theta slice shifts independently ---
        moved = np.zeros_like(self.belief)
        for ith in range(self.theta_dim):
            theta_rad = math.radians(ith * cfg.THETA_RES)
            shift_x = int(round((d_trans * math.cos(theta_rad)) / cfg.SPATIAL_RES))
            # Negative because grid y-index 0 is the top of the world
            shift_y = int(round((-d_trans * math.sin(theta_rad)) / cfg.SPATIAL_RES))
            moved[:, :, ith] = roll_no_wrap_2d(self.belief[:, :, ith], shift_x, shift_y)
        self.belief = moved

        # --- Second rotation ---
        self._rotate_belief(d_rot2)

        # --- Diffusion: model growing uncertainty over time ---
        self.belief = gaussian_filter(
            self.belief,
            sigma=(cfg.MOTION_SIGMA_XY, cfg.MOTION_SIGMA_XY, cfg.MOTION_SIGMA_THETA),
            mode=('constant', 'constant', 'wrap'),
        )

        self._normalize()

    def _rotate_belief(self, delta_deg):
        shift = int(round(delta_deg / cfg.THETA_RES))
        if shift != 0:
            self.belief = np.roll(self.belief, shift=shift, axis=2)

    # ------------------------------------------------------------------
    # Update step (Gaussian likelihood field)
    # ------------------------------------------------------------------

    def update(self, marker_id, measured_range, measured_bearing_deg):
        """Weight the belief by how well each cell explains the observation."""
        if marker_id not in self.landmarks:
            return

        lx, ly = self.landmarks[marker_id]

        dx = lx - self.rx
        dy = ly - self.ry

        expected_range   = np.sqrt(dx * dx + dy * dy)
        expected_bearing = wrap_angle_deg(np.degrees(np.arctan2(dy, dx)) - self.rth)

        range_err   = measured_range - expected_range
        bearing_err = wrap_angle_deg(measured_bearing_deg - expected_bearing)

        range_like   = np.exp(-0.5 * (range_err   / cfg.SIGMA_RANGE)   ** 2)
        bearing_like = np.exp(-0.5 * (bearing_err / cfg.SIGMA_BEARING) ** 2)

        self.belief *= (range_like * bearing_like) + cfg.EPS
        self._normalize()

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    def best_estimate(self):
        """Return the (ix, iy, ith) index of the highest-probability cell."""
        return np.unravel_index(np.argmax(self.belief), self.belief.shape)