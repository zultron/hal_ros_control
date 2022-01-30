from typing import List, Dict, Union

from launch import logging
from launch.action import Action
from launch.actions import OpaqueFunction, EmitEvent, ExecuteProcess
from launch.event_handler import EventHandler
from launch.events import Shutdown

from launch_ros.actions import Node

from machinekit import rtapi
from .hal_ready import HalReady


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

    # By default, cancel on Shutdown event
    cancel_on_shutdown = True
    # By default, don't do node argument processing
    run_add_ros_specific_arguments = False

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)
        self.__canceled = False

    def cancel(self, context, warn=True):
        """
        Cancel this Action's handlers.

        This function is not thread-safe and should be called only from under
        another coroutine.
        """
        self.__canceled = True
        if hasattr(self, "handler"):
            msg = f"Canceling handler {self}"
            getattr(self.__logger, "warn" if warn else "debug")(msg)
            try:
                context.unregister_event_handler(self.handler)
            except Exception:
                pass

    def set_triggers(self, *, triggered_by, triggers):
        self.__logger.debug(
            f"{self}:  triggered_by {triggered_by}; triggers {triggers}"
        )
        self.triggered_by = triggered_by
        self.triggers = triggers

    def add_ros_specific_arguments(self, context):
        if not self.run_add_ros_specific_arguments:
            return

        # Run before execute_deferred(); launch params break w/o it.
        # Cribbed from launch_ros.actions.node.Node.execute().
        #
        # Prepare the ros_specific_arguments list and add it to the
        # context so that the LocalSubstitution placeholders added to
        # the the cmd can be expanded using the contents.
        self._perform_substitutions(context)  # expand self.node_name
        ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
        node_name = super(Node, self).__node_name
        if node_name is not None:
            name_arg = f"__node:={node_name}"
            ros_specific_arguments["name"] = name_arg
        if self.expanded_node_namespace != "":
            ns_arg = f"__ns:={self.expanded_node_namespace}"
            ros_specific_arguments["ns"] = ns_arg
        local_args = dict(ros_specific_arguments=ros_specific_arguments)
        context.extend_locals(local_args)

    def init_rtapi(self):
        # Be sure rtapi is initialized
        if not getattr(rtapi, "__rtapicmd", False):
            rtapi.init_RTAPI()
        self.rtapi = rtapi

    def execute_deferred(self, context):
        """Execute the action."""
        return self.__execute(context)

    @property
    def ready_event(self):
        """Return `HalReady` event to trigger next Action."""
        if self.triggers:
            return [EmitEvent(event=HalReady(self.triggers))]
        else:
            return []

    def on_ready_event(self, context):
        """
        `HalReady` event handler for executing deferred action.

        Do the following:

        - Finalize command arguments
        - Initialize the `rtapi` object
        - Cancel the event handler
        - Execute the deferred action
        - Trigger the next action
        """
        self.init_rtapi()
        self.cancel(context, warn=False)
        try:
            res = self.execute_deferred(context)
        except Exception as e:
            shutdown = Shutdown(
                reason=f"{self} canceling on error:  '{e}'",
                due_to_sigint=False
            )
            return [shutdown]
        return (res or []) + self.ready_event

    def matcher(self, event):
        """Match `HalReady` event corresponding to this action."""
        return isinstance(event, HalReady) and event.name == self.triggered_by

    def execute(self, context):
        """
        Defer action execution.

        - Create a task for the coroutine that waits until event or
          canceled

        - Coroutine asynchronously fires this action after previous
          actions complete, unless canceled first.
        """
        assert hasattr(self, "triggered_by"), f"{self}: set_triggers() uncalled"

        self.__logger.info(f"HalOrderedAction.execute() called for {self.name}")

        if context.is_shutdown:
            # If shutdown starts before execution can start, don't
            # start execution.
            return None

        # No `triggered_by` name means ready NOW, just do it.
        if self.triggered_by is None:
            return self.on_ready_event(context)

        if self.__canceled:
            self.__logger.warn(f"HAL action {self} canceled before execution")
            return

        self.handler = EventHandler(
            matcher=self.matcher,
            entities=OpaqueFunction(function=self.on_ready_event),
        )
        context.register_event_handler(self.handler)

        if self.cancel_on_shutdown:
            # Cancel execution at the 'shutdown' event so as to not hold
            # up the launch process
            context.register_event_handler(
                EventHandler(
                    matcher=lambda event: isinstance(event, Shutdown),
                    entities=OpaqueFunction(function=self.cancel),
                )
            )

        # Save the execute() method we would have run for later
        self.__execute = super().execute

class HalOrderedNode(Node, HalOrderedAction, ExecuteProcess):
    """
    Class for ordered HAL Nodes.

    This class inserts :class:`HalOrderedAction` between
    :class:`launch_ros.actions.Node` and
    :class:`launch.actions.ExecuteProcess` in order to hijack the call
    to the latter's :meth:`execute` method.

    This lets us get away with subclassing
    :class:`launch_ros.actions.node.Node` and reusing its `execute()`
    method, which contains non-optional logic that uses munged/private
    parameters and uses the deprecated (??!?) `__node_name` attribute.
    """
