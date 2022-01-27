"""Module for HalReady event."""

from launch.event import Event


class HalReady(Event):
    """Event emitted when `hal_mgr` has finished bringing up HAL."""

    name = "hal_hw_interface.launch.hal_ready_event"
