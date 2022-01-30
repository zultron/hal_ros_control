from machinekit import hal
from launch import logging
from .hal_ordered_action import HalOrderedNode, HalAsyncReadyAction


class HalUserNode(HalOrderedNode, HalAsyncReadyAction):
    """
    Action that loads a HAL "user" (non-RT) component ROS node.

    The component isn't started until after `execution_deferred()`
    returns, and it may need time before the HAL component is marked
    ready.
    """

    def __init__(self, *, executable, name=None, **kwargs) -> None:
        """
        Construct a HalUserNode action.

        :param: name After starting, wait until the HAL component with
            this name becomes ready.  Defaults to `executable`
            argument.
        """
        self.__logger = logging.get_logger(__name__)
        kwargs.setdefault("on_exit", self.shutdown_action("Exited"))
        name = name or executable
        super().__init__(name=name, executable=executable, **kwargs)
        self.__executable = executable

    def is_ready(self, context):
        # Check if component is registered and become ready
        if self.name not in hal.components:
            self.__logger.debug("...HAL comp {self.name} not yet registered")
            return False
        elif hal.components[self.name].state != hal.COMP_READY:
            self.__logger.debug("...HAL comp {self.name} not yet ready")
            return False
        else:
            return True
