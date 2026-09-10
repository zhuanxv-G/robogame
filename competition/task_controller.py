"""Competition task-level adapter for the verified arm action module."""

from __future__ import annotations

from typing import Protocol


class ArmActions(Protocol):
    """Existing ArmSequence operations used by the competition layer."""

    def run_all(self) -> None: ...

    def safe_home(self) -> None: ...


class TaskController:
    """Schedule verified arm actions without owning a serial connection.

    The supplied object is normally :class:`arm_sequence.ArmSequence`, itself
    constructed with the competition process's single ``Stm32Link``.  Merely
    constructing this controller never executes an arm action.
    """

    def __init__(self, arm: ArmActions) -> None:
        self._arm = arm

    def grab_orange(self) -> object:
        """Run one orange-block pickup when a verified action becomes known.

        The existing arm groups combine pickup and placement, and none is
        identified as the standalone orange pickup.  Do not guess a mapping.
        """
        return NotImplemented

    def grab_purple(self) -> object:
        """Reserve the purple-block pickup interface without inventing motion."""
        return NotImplemented

    def build(self) -> None:
        """Execute the existing, hardware-verified three-group build flow."""
        self._arm.run_all()

    def reset_arm(self) -> None:
        """Return the arm through the existing verified safe-home action."""
        self._arm.safe_home()
