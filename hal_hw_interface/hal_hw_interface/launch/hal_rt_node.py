import os

from typing import List, Optional

from launch import SomeSubstitutionsType, logging
from launch.utilities import perform_substitutions
from launch.action import Action
from launch.launch_context import LaunchContext
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

from machinekit import rtapi
from .hal_ordered_action import HalOrderedNode


class HalRTNode(HalOrderedNode):
    """Action that loads a HAL RT component ROS node."""

    def __init__(
        self,
        *,
        component: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        **kwargs,
    ) -> None:
        """
        Construct a HalRTNode action.

        Similar to :class:`launch.actions.Node`, this action launches
        a ROS node, but as a real time HAL component.
        """
        self.__logger = logging.get_logger(__name__)

        if package is not None:
            pkg_prefix = FindPackagePrefix(package=package)
            path = PathJoinSubstitution([pkg_prefix, "lib", component])
        else:
            path = component

        # Pretend the component (a loadable plugin) is the full path
        # of an executable, thereby forced into the executable `Node`
        # scheme.  "Forced," for sure.
        super().__init__(package=None, executable=path, **kwargs)
        self.__package = package
        self.__component = component
        self.__component_path = path

    def execute_deferred(
        self, context: LaunchContext
    ) -> Optional[List[Action]]:
        """Execute the action."""
        # Expand command substitutions
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        comp_path = cmd[0]
        self.__logger.info(f"Executing HAL RT component {comp_path}")
        self.comp_args = ",".join(cmd[1:])
        self.__logger.info(f"  args: {self.comp_args}")

        # Load the component
        rtapi.loadrt(comp_path, ARGV=self.comp_args)
