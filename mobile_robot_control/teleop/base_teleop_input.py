"""Abstract base class for all teleop input sources."""
from abc import ABC
from abc import abstractmethod

from .body_velocity import BodyVelocity


class BaseTeleopInput(ABC):
    """Contract for all teleop input sources.

    A differential drive robot is fully described by two scalars:
      v  — linear velocity  (m/s)   forward (+) / backward (-)
      ω  — angular velocity (rad/s) counter-clockwise (+) / clockwise (-)

    Any input device (keyboard, gamepad, joystick) must map its raw events
    onto this (v, ω) pair. Concrete subclasses own their input loop and
    update an internal cache.
    """

    def __init__(self) -> None:
        """Initialise with the input source marked active."""
        self._active = True

    @abstractmethod
    def read_input(self) -> BodyVelocity:
        """Return current (v, ω). Must be non-blocking."""

    def is_active(self) -> bool:
        """Return True while the input source is alive."""
        return self._active

    def start(self) -> None:
        """Start input loop (spawn threads, init devices). Override as needed."""

    def cleanup(self) -> None:
        """Permanently release resources and mark inactive.

        Not reversible — call this only on shutdown, not to temporarily
        pause input. Once called, start() should not be called again.
        """
        self._active = False
