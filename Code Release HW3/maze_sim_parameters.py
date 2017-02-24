import numpy as np
from ExtractLines import FitLine

LineExtractionParams = {'MIN_SEG_LENGTH': 0.1,             # minimum length of each line segment (m)
                        'LINE_POINT_DIST_THRESHOLD': 0.05, # max distance of pt from line to split
                        'MAX_P2P_DIST': 0.4,               # max distance between two adjent pts within a segment
                        'MIN_POINTS_PER_SEGMENT': 3}       # minimum number of points per line segment

NoiseParams = {'P0': 0.01*np.eye(3),  # initial state covariance (x0 comes from ground truth; nonzero in case of timing mismatch)
               'Q': 0.1*np.eye(2),    # control noise covariance (corresponding to dt = 1 second)
               'var_theta': 0.1,      # laser scan noise variance in theta measurement (per point)
               'var_rho': 0.1,        # laser scan noise variance in rho measurement (per point)
               'g': 3.}               # validation game (essentially maximum z-score)

MAZE = [
  ((5, 5), (-5, 5)),
  ((-5, 5), (-5, -5)),
  ((-5,-5), (5, -5)),
  ((5, -5), (5, 5)),
  ((-3, -3), (-3, -1)),
  ((-3, -3), (-1, -3)),
  ((3, 3), (3, 1)),
  ((3, 3), (1, 3)),
  ((1, -1), (3, -1)),
  ((3, -1), (3, -3)),
  ((-1, 1), (-3, 1)),
  ((-3, 1), (-3, 3)),
  ((-1, -1), (1, -3)),
  ((-1, 5), (-1, 2)),
  ((0, 0), (1, 1))
]

MapParams = np.array([FitLine(np.array([np.arctan2(p1[1], p1[0]), np.arctan2(p2[1], p2[0])]),
                              np.array([np.linalg.norm(p1), np.linalg.norm(p2)])) for p1, p2 in MAZE]).T