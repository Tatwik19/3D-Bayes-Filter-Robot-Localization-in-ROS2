# All tunable parameters for the 3D Bayes filter.

# World geometry
WORLD_SIZE = 16.0       # meters, square world
WORLD_MIN  = -8.0
WORLD_MAX  =  8.0

# Grid resolution
SPATIAL_RES = 0.2       # meters per cell
THETA_RES   = 5.0      # degrees per angular bin

# Robot starting pose [x, y, heading_deg]
INITIAL_POSE = [-7.0, -7.0, 90.0]

# Measurement noise (tighten to sharpen updates, loosen if belief collapses)
SIGMA_RANGE   = 0.5     # meters
SIGMA_BEARING = 15.0     # degrees

# Motion blur applied after every predict step.
# Higher = more spread (more uncertainty), lower = belief stays sharp but may diverge.
MOTION_SIGMA_XY    = 0.5
MOTION_SIGMA_THETA = 0.3

# Numerical safety floor (prevents division by zero / NaN)
EPS = 1e-12