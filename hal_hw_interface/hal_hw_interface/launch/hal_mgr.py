import functools

from launch import logging
from launch.action import Action
from launch.actions import GroupAction, OpaqueFunction
from launch.event import Event
from launch.event_handler import EventHandler
from launch.events import Shutdown

from launch_ros.actions.node import Node
from launch_ros.ros_adapters import get_ros_node

from std_msgs.msg import Bool


class HalReady(Event):
    """Event emitted when parts of HAL bringup are complete."""

    name = "hal_hw_interface.launch.hal_ready_event"

    def __init__(self, name):
        """Create a HalReady event."""
        self.__name = name

    @property
    def name(self):
        """Getter for name."""
        return self.__name

    def __repr__(self):
        return f"<HalReady('{self.name}') event object>"


class HalOrderedAction(Action):
    """
    Mixin for deferred actions.

    This action is deferred until a HalReady event named by
    `triggered_by`, and after execution emits a HalReady event named
    by `triggers`.

    This mixin class must precede the base class in subclass
    definitions, since it overrides the `execute()` method.

    Subclasses must do the actual work normally done by `execute()` in
    the `execute_deferred()` method instead.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)
        self.__canceled = False

    def cancel(self, context):
        """
        Cancel this Action.

        This function is not thread-safe and should be called only from under
        another coroutine.
        """
        self.__canceled = True
        if hasattr(self, "handler"):
            self.__logger.warn(f"Canceling handler {self}")
            context.unregister_event_handler(self.handler)

    def set_triggers(self, *, triggered_by, triggers, cancel_all):
        self.__logger.debug(
            f"{self}:  triggered_by {triggered_by}; triggers {triggers}"
        )
        self.triggered_by = triggered_by
        self.triggers = triggers
        self.cancel_all = cancel_all

    def execute_deferred(self, context):
        raise NotImplementedError("Subclasses must define execute_deferred")

    def on_ready_event(self, context):
        try:
            self.execute_deferred(context)
        except Exception:
            self.__logger.error(f"{self}:  Canceling")
            self.cancel_all()
            raise
        if not self.triggers:
            return
        context.asyncio_loop.call_soon_threadsafe(
            lambda: context.emit_event_sync(HalReady(self.triggers))
        )

    def matcher(self, event):
        return isinstance(event, HalReady) and event.name == self.triggered_by

    def execute(self, context):
        """
        Execute the action.

        This does the following:
        - Create a task for the coroutine that waits until event or canceled
        - Coroutine asynchronously fires event after event, if not canceled
        """
        assert hasattr(self, "triggered_by"), f"{self}: set_triggers() uncalled"

        self.__logger.info(f"HalOrderedAction.execute() called for {self.name}")

        # No `triggered_by` name means ready NOW, just do it.
        if self.triggered_by is None:
            self.on_ready_event(context)
            return

        if self.__canceled:
            self.__logger.warn(f"HAL action {self} canceled before execution")
            return

        self.handler = EventHandler(
            matcher=self.matcher,
            entities=OpaqueFunction(function=self.on_ready_event),
        )
        context.register_event_handler(self.handler)

        # Cancel execution at the 'shutdown' event so as to not hold
        # up the launch process
        context.register_event_handler(
            EventHandler(
                matcher=lambda event: isinstance(event, Shutdown),
                entities=OpaqueFunction(function=self.cancel),
            )
        )


class HalConfig(GroupAction):
    """
    Action that starts Machinekit HAL and loads a HAL configuration.

    This group action starts the `hal_mgr` node.  Once it has brought
    up Machinekit HAL, any deferred `HalRTNode` or `HalNode` actions
    will execute, loading their respective HAL components.  Once those
    are all ready, a deferred `HalConfig` action will signal the
    `hal_mgr` to load the configured HAL file.

    This action is necessary for two main reasons:

    - Traditional HAL file configuration for loading components can't
      encompass the full complexity of ROS node launch configuration.
      This scheme separates out the loading of HAL components running
      ROS nodes into familiar ROS launch actions, while leaving the
      more standard portion of the HAL configuration its usual HAL
      file form.

    - Machinekit HAL startup sequence requires the RTAPI/HAL
      environment start before the HAL configuration can load, and in
      the HAL configuration, HAL components must be loaded before pins
      may be netted.  This action enforces that sequence.
    """

    def __init__(
        self,
        actions,
        *,
        scoped=True,
        launch_configurations=None,
        **hal_mgr_kwargs,
    ):
        """
        Create the HalConfig action.

        This action passes all the usual keyword args to the `hal_mgr`
        node constructor.

        The `actions` arg is a list of `HalRTNode`, `HalNode` and
        `HalConfig` actions; the `hal_mgr` node will be inserted at
        the beginning.  The `actions` and `launch_configurations` args
        are used as arguments to the `GroupAction`.
        """
        self.actions = actions

        # Create the hal_mgr node action
        hal_mgr_kwargs.setdefault("name", "hal_mgr")
        triggers = (actions[0].name if actions else None) or "action0"
        hal_mgr_kwargs.setdefault("triggers", triggers)
        hal_mgr_action = HalMgr(**hal_mgr_kwargs)

        # Add ordering to actions
        for i, action in enumerate(actions):
            # This action is triggered_by previous action's triggers
            triggered_by = triggers
            # Make up a trigger name for the next action
            if len(actions) - i == 1:  # Last action in list
                triggers = None
            else:
                triggers = actions[i + 1].name or f"action{i}"
            # Set ordering for this action
            action.set_triggers(
                triggered_by=triggered_by,
                triggers=triggers,
                cancel_all=self.cancel_all,
            )

        # Create the group action
        actions.insert(0, hal_mgr_action)
        super().__init__(
            actions, scoped=scoped, launch_configurations=launch_configurations
        )

    def cancel_all(self):
        for action in self.actions():
            action.cancel()


class HalMgr(Node):
    """Action that starts Machinekit HAL with the `hal_mgr` ROS node."""

    def __init__(
        self,
        *,
        package="hal_hw_interface",
        executable="hal_mgr",
        triggers=None,
        ready_timeout=10,
        **kwargs,
    ) -> None:
        """
        Construct a HalMgr action.

        Launch the `hal_mgr` node as in :class:`launch.actions.Node`.
        When the `hal_mgr/ready` topic is `True`, emit a
        :class:`hal_hw_interface.launch.hal_ready_event.HalReady`
        event.
        """
        self.__logger = logging.get_logger(__name__)
        self.triggers = triggers

        super().__init__(package=package, executable=executable, **kwargs)

    def _on_ready_event(self, context, msg):
        # If the hal_mgr/ready topic is True, emit an event; no more
        # to do, so destroy the subscriber
        if msg.data:
            context.asyncio_loop.call_soon_threadsafe(
                lambda: context.emit_event_sync(HalReady(self.triggers))
            )
            self.__hal_mgr_ready_subscription.destroy()

    def execute(self, context):
        """Execute the action."""

        # Start hal_mgr
        res = super().execute(context)

        # Create a subscription to monitor when RTAPI/HAL is running
        node = get_ros_node(context)
        self.__hal_mgr_ready_subscription = node.create_subscription(
            Bool,
            "hal_mgr/ready",
            functools.partial(self._on_ready_event, context),
            10,
        )

        return res
