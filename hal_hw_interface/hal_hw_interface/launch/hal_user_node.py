from typing import List, Optional

from launch import logging
from launch.action import Action
from launch.actions import Shutdown
from launch.launch_context import LaunchContext
from launch_ros.actions.node import Node

from .hal_ordered_action import HalOrderedAction


class HalUserNode(HalOrderedAction, Node):
    """Action that loads a HAL "user" (non-RT) component ROS node."""

    def __init__(self, *, executable, **kwargs) -> None:
        """
        Construct a HalUserNode action.

        Similar to :class:`launch.actions.Node`, this action launches
        a ROS node executable with a non-real time HAL "user"
        component.
        """
        self.__logger = logging.get_logger(__name__)
        kwargs.setdefault("on_exit", Shutdown(reason="Exited"))
        super().__init__(executable=executable, **kwargs)
        self.__executable = executable

    def execute_deferred(
        self, context: LaunchContext
    ) -> Optional[List[Action]]:
        """Execute the action."""

        super(Node, self).execute(context)

    def __repr__(self):
        return f"<HalUserNode '{self.__executable}'>"
