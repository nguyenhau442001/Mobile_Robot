"""Abstract base class for mobile robot velocity controllers."""
from abc import ABC
from abc import abstractmethod


class RobotController(ABC):
    """Interface for a velocity profile / control law.

    Each concrete subclass implements a specific control strategy (P, PID,
    trapezoidal profile, etc.). The go_to_goal server calls only these two
    methods per tick, so controllers are interchangeable without touching
    the server logic.

    Both methods receive pre-computed scalar errors and return the desired
    velocity command. Clamping to hardware limits is the controller's
    responsibility.
    """

    @abstractmethod
    def compute_linear(self, distance: float, dt: float) -> float:
        """Return the desired forward velocity [m/s] for the given distance error.

        Args:
            distance: Straight-line distance to the goal [m]. Always >= 0.
            dt:       Elapsed time since the last call [s].

        Returns:
            Forward velocity command [m/s].
        """

    @abstractmethod
    def compute_angular(self, heading_error: float, dt: float) -> float:
        """Return the desired angular velocity [rad/s] for the given heading error.

        Args:
            heading_error: Signed angle to the goal [rad], normalized to [-pi, pi].
            dt:            Elapsed time since the last call [s].

        Returns:
            Angular velocity command [rad/s].
        """
