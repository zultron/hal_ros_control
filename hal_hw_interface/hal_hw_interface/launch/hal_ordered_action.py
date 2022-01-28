from typing import List, Dict, Union

from launch import logging
from launch.action import Action
from launch.actions import OpaqueFunction
from launch.event_handler import EventHandler
from launch.events import Shutdown

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

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)
        self.__canceled = False

    def cancel(self, context, warn=True):
        """
        Cancel this Action.

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

    def set_triggers(self, *, triggered_by, triggers, cancel_all):
        self.__logger.debug(
            f"{self}:  triggered_by {triggered_by}; triggers {triggers}"
        )
        self.triggered_by = triggered_by
        self.triggers = triggers
        self.cancel_all = cancel_all

    def add_ros_specific_arguments(self, context):
        # Run before execute_deferred(); launch params break w/o it.
        # Copied from launch_ros.actions.node.Node.execute().
        #
        # Prepare the ros_specific_arguments list and add it to the
        # context so that the LocalSubstitution placeholders added to
        # the the cmd can be expanded using the contents.
        self._perform_substitutions(context)  # expand self.node_name
        ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
        if self.node_name is not None:
            name_arg = f"__node:={self._Node__node_name}"
            ros_specific_arguments["name"] = name_arg
        if self.expanded_node_namespace != "":
            ns_arg = f"__ns:={self.expanded_node_namespace}"
            ros_specific_arguments["ns"] = ns_arg
        local_args = dict(ros_specific_arguments=ros_specific_arguments)
        context.extend_locals(local_args)

    def execute_deferred(self, context):
        raise NotImplementedError("Subclasses must implement execute_deferred")

    def on_ready_event(self, context):
        try:
            self.add_ros_specific_arguments(context)
            self.execute_deferred(context)
            self.cancel(context, warn=False)
        except Exception:
            self.__logger.error(f"{self}:  Canceling")
            self.cancel_all(context)
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

        if context.is_shutdown:
            # If shutdown starts before execution can start, don't
            # start execution.
            return None

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

        if self.cancel_on_shutdown:
            # Cancel execution at the 'shutdown' event so as to not hold
            # up the launch process
            context.register_event_handler(
                EventHandler(
                    matcher=lambda event: isinstance(event, Shutdown),
                    entities=OpaqueFunction(function=self.cancel),
                )
            )
