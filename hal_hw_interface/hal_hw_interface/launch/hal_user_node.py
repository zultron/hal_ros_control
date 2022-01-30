from typing import List, Optional
from machinekit import hal

from launch import logging
from launch.action import Action
from launch.actions import TimerAction, OpaqueFunction, EmitEvent
from launch.launch_context import LaunchContext

from .hal_ready import HalReady
from .hal_ordered_action import HalOrderedNode


class HalUserNode(HalOrderedNode):
    """
    Action that loads a HAL "user" (non-RT) component ROS node.

    The component isn't started until after `execution_deferred()`
    returns, and it may take more time to become ready, so the trigger
    for next action cannot fire immediately.

    Instead, this action periodically checks if component has become
    ready before triggering next action.  If still not ready before
    a timeout, it aborts launch.
    """

    # Don't automatically trigger when execute_deferred() returs
    auto_trigger = False
    # Ready check interval in seconds
    ready_check_timeout = 0.1

    def __init__(
        self, *, executable, name=None, wait_timeout=5.0, **kwargs
    ) -> None:
        """
        Construct a HalUserNode action.

        Arguments are similar to :class:`launch.actions.Node`, with
        the following differences:

        :param: name After starting, wait until the HAL component with
            this name becomes ready.  Defaults to `executable`
            argument.

        :param: wait_timeout After starting, if component does not
            become ready within this number of seconds, abort the
            launch.  Defaults to 5.0.
        """
        self.__logger = logging.get_logger(__name__)
        kwargs.setdefault("on_exit", self.shutdown_action("Exited"))
        name = name or executable
        super().__init__(name=name, executable=executable, **kwargs)
        self.__executable = executable
        self.__wait_timeout = wait_timeout
        self.__ready_checker = None  # ready check timer
        self.__ready_timeout = None  # wait_timeout timer

    def check_ready(self, context):
        # Check if component is registered and become ready
        ready = True
        if self.name not in hal.components:
            self.__logger.debug("...HAL comp {self.name} not yet registered")
            ready = False
        elif hal.components[self.name].state != hal.COMP_READY:
            self.__logger.debug("...HAL comp {self.name} not yet ready")
            ready = False
        if not ready:
            # Not ready; schedule another check
            return [
                TimerAction(
                    period=self.ready_check_timeout,
                    actions=[OpaqueFunction(function=self.check_ready)],
                ),
            ]
        # Component ready; cancel timers and trigger next event
        self.__ready_timeout.cancel()
        self.__logger.info(f"HAL component {self.name} started successfully")
        return [EmitEvent(event=HalReady(self.triggers))]

    def cancel(self, context, warn=True):
        for t in [self.__ready_timeout, self.__ready_checker]:
            if t is not None:
                t.cancel()
        super().cancel(context, warn=warn)

    def ready_event(self):
        if not self.triggers:
            return []
        # Periodic ready check timer:  trigger next action when comp is ready
        self.__ready_checker = TimerAction(
            period=self.ready_check_timeout,
            actions=[OpaqueFunction(function=self.check_ready)]
        )
        # Wait timeout timer:  shut down launch
        self.__ready_timeout = TimerAction(
            period=self.__wait_timeout,
            actions=self.shutdown_action(
                f"HAL component {self.name} startup timed out"
            ),
        )
        return [self.__ready_timeout, self.__ready_checker]

    def __repr__(self):
        return f"<HalUserNode '{self.__executable}'>"
