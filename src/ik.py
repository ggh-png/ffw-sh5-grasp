"""Backward-compatible entry point for the MJCF-tree kinematics solver.

New code can import :class:`kinematics.KinematicsSolver` directly.  The historical
``InverseKinematics`` name remains so phase tests and external scripts do not need an API
migration; FK, the geometric Jacobian, and IK are all implemented by the same tree solver.
"""

import kinematics


DEFAULT_DAMPING = kinematics.DEFAULT_DAMPING
DEFAULT_MAX_JOINT_DELTA = kinematics.DEFAULT_MAX_JOINT_DELTA
DEFAULT_MAX_ITER = kinematics.DEFAULT_MAX_ITER
POS_TOL = kinematics.POSITION_TOLERANCE
ORI_TOL = kinematics.ORIENTATION_TOLERANCE


class InverseKinematics(kinematics.KinematicsSolver):
    """Compatibility alias for :class:`kinematics.KinematicsSolver`."""
