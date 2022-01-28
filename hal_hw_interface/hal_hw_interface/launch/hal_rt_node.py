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
            path_subs = [pkg_prefix, "lib", component]
            path = PathJoinSubstitution(path_subs)
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

        # Next bit copied from launch_ros.actions.node.Node.execute()
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

        # Be sure rtapi is initialized
        if not getattr(rtapi, "__rtapicmd") or not rtapi.__rtapicmd:
            rtapi.init_RTAPI()

        # Expand substitutions
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        comp_path = cmd[0]
        self.__logger.info(f"Executing HAL component {comp_path}")
        comp_args = ",".join(cmd[1:])
        self.__logger.info(f"  args: {comp_args}")

        # Load the component
        rtapi.loadrt(comp_path, ARGV=comp_args)
