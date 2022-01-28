from typing import List, Dict, Optional, Union

from launch import SomeSubstitutionsType, logging
from launch.utilities import perform_substitutions
from launch.action import Action
from launch.launch_context import LaunchContext
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix
from launch_ros.actions.node import Node

from machinekit import rtapi
from .hal_mgr import HalOrderedAction


class HalRTNode(HalOrderedAction, Node):
    """Action that loads a HAL RT component ROS node."""

    # HAL RT comps are shut down with HAL by hal_mgr
    cancel_on_shutdown = False

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
        # of an executable to force this into the executable `Node`
        # scheme.  It's forced, for sure.
        super().__init__(package=None, executable=path, **kwargs)
        self.__package = package
        self.__component = component
        self.__component_path = path

    def execute_deferred(
        self, context: LaunchContext
    ) -> Optional[List[Action]]:
        """Execute the action."""
        if context.is_shutdown:
            # If shutdown starts before execution can start, don't
            # start execution.
            return None

        # Expand command substitutions
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        comp_path = cmd[0]
        self.__logger.info(
            f"Executing HAL RT component {comp_path}"
        )
        self.comp_args = ",".join(cmd[1:])
        self.__logger.info(f"  args: {self.comp_args}")

        # Be sure rtapi is initialized
        if not getattr(rtapi, "__rtapicmd") or not rtapi.__rtapicmd:
            rtapi.init_RTAPI()

        # Load the component
        rtapi.loadrt(comp_path, ARGV=self.comp_args)
