import os
import subprocess
from machinekit import config, rtapi
from .ros_hal_component import RosHalComponent

# ROS
from std_msgs.msg import Bool


class HalMgr(RosHalComponent):
    compname = "hal_mgr"
    READY_TOPIC = "hal_mgr/ready"
    shutdown_begun = False
    update_rate = 0.1  # sec
    default_node_kwargs = dict(
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    def init_hal_comp(self):
        # Don't actually set up any HAL comp
        pass

    def setup_component(self):
        # Not actually setting up HAL comp; don't call
        # super().setup_component()
        self.init_ready_topic_publisher()
        self.start_realtime()
        self.load_hal_config()

    def init_ready_topic_publisher(self):
        """Set up "ready" status publisher."""
        self.ready_pub = self.node.create_publisher(Bool, self.READY_TOPIC, 1)
        self.ready_msg = Bool()
        self.publish_ready_topic(value=False)

    def start_realtime(self):
        """
        Start new RTAPI/HAL session.

        Start Machinekit `realtime` environment.  Register the
        :py:func:`stop_realtime` shutdown callback.
        """
        # Set up console debug output and config
        d_out = "1" if self.get_ros_param("hal_debug_output", True) else "0"
        d_lev = str(self.get_ros_param("hal_debug_level", 1))
        self.logger.info(f"Starting hal_mgr; debug={d_out}/level={d_lev}")
        env = dict(
            DEBUG=d_lev,
            SYSLOG_TO_STDERR=d_out,
            MACHINEKIT_INI=config.Config().MACHINEKIT_INI,
            **os.environ,
        )
        subprocess.check_call(["realtime", "start"], env=env)
        self.add_shutdown_callback(self.stop_realtime, 999)  # run last

        # Signal readiness
        self.publish_ready_topic(value=True)
        self.logger.info("hal_mgr:  Started realtime")

    def stop_realtime(self):
        self.logger.warn("Stopping realtime")
        subprocess.check_call("realtime stop", shell=True)
        self.logger.info("Realtime stopped")

    def load_hal_config(self):
        # Be sure rtapi is initialized (used in python HAL configs)
        if not getattr(rtapi, "__rtapicmd") or not rtapi.__rtapicmd:
            rtapi.init_RTAPI()

        # Load the hal_hw_interface
        hal_comp_dir = self.get_ros_param("hal_comp_dir")
        assert hal_comp_dir is not None
        hal_hw_interface_path = os.path.join(hal_comp_dir, "hal_control_node")
        self.logger.info(f"hal_hw_interface_path:  {hal_hw_interface_path}")
        self.logger.info(f"hal_hw_interface_args:  {str(self.argv)}")
        rtapi.loadrt(hal_hw_interface_path, ARGV=",".join(self.argv))
        self.logger.info("rosctl_intf comp loaded successfully")

        # Get HAL config directory and filename list from ROS parameters
        hal_file_dir = self.get_ros_param("hal_file_dir")
        assert hal_file_dir is not None
        hal_files = self.get_ros_param("hal_files")
        assert isinstance(hal_files, list)

        # Munge directory and filenames into paths and load HAL files
        params = {
            p: self.get_ros_param(p)
            for p in self.node.get_parameters_by_prefix("")
        }
        params["argv"] = self.argv
        for hal_file in hal_files:
            hal_file_path = os.path.join(hal_file_dir, hal_file)
            self.logger.info(f"Loading HAL file '{hal_file_path}'")
            assert os.path.exists(hal_file_path)
            if hal_file_path.endswith(".py"):
                with open(hal_file_path, "r") as f:
                    data = compile(f.read(), hal_file_path, "exec")
                globals_ = dict(params=params)
                exec(data, globals_)
            else:
                subprocess.check_call(["halcmd", "-f", hal_file_path])

        self.logger.info("HAL configuration loaded")

    def update(self):
        # FIXME Watch realtime; shut down if it shuts down
        self.publish_ready_topic()

    def publish_ready_topic(self, value=None):
        """
        Publish the HAL config "ready" status.

        This topic coming `True` indicates RTAPI/HAL is running.

        Change published value by setting the `value` param to `True`
        or `False.
        """
        if value is not None:
            if self.ready_msg.data != value:
                self.logger.info(f"hal_mgr/ready topic new value:  {value}")
            self.ready_msg.data = value
        self.ready_pub.publish(self.ready_msg)
