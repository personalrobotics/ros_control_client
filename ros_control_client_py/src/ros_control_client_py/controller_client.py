class ROSControlError(Exception):
    pass


class SwitchError(ROSControlError):
    pass


class ControllerSwitcher(object):
    def __init__(self, mode_switcher, controller_names):
        """ Construct a context manager for switching controllers.

        @param mode_switcher: mode switcher object
        @type  mode_switcher: ModeSwitcher
        @param controller_names: requested controller names
        @type  controller_names: [str]
        """
        self._mode_switcher = mode_switcher
        self._requested_controllers = set(controller_names)
        self._started_controllers = None
        self._stopped_controllers = None

    def __enter__(self):
        self._started_controllers, self._stopped_controllers = self.switch()

    def __exit__(self, type, value, tb):
        self.unswitch()

    def switch(self):
        """ Switch to the requested controllers.

        Any controllers that conflict with the resources used by the requested
        controllers are unloaded. This operation is performed atomically.

        @return tuple containing the loaded and unloaded controllers
        @rtype  [str], [str]
        """
        from controller_manager_msgs.srv import SwitchControllerRequest

        controller_infos_msg = self._mode_switcher._list_controllers_srv()
        controller_infos = controller_infos_msg.controller

        # Figure out what resources the requested controllers need.
        required_resources = set(sum([
            controller_info.resources
            for controller_info in controller_infos
            if controller_info.name in self._requested_controllers
        ], []))

        # Start any of the requested controllers that are not already running.
        start_controllers = [
            controller_info.name
            for controller_info in controller_infos
            if controller_info.name in self._requested_controllers
            if controller_info.state != 'running'
        ]

        # Stop any non-requested controllers that conflict with our resources.
        stop_controllers = [
            controller_info.name
            for controller_info in controller_infos
            if controller_info.name not in self._requested_controllers
            if controller_info.state == 'running'
            if not required_resources.isdisjoint(controller_info.resources)
        ]

        ok = self._mode_switcher._switch_controllers_srv(
            start_controllers=start_controllers,
            stop_controllers=stop_controllers,
            strictness=SwitchControllerRequest.STRICT
        )
        if ok:
            return start_controllers, stop_controllers
        else:
            raise SwitchError('Switching controllers failed.')

    def unswitch(self):
        """ Reverts to the controllers loaded before switch() was called. """

        if (self._started_controllers is None
                or self._stopped_controllers is None):
            raise ROSControlError('Unknown state. Did you call __enter__?')

        ok = self._mode_switcher._switch_controllers_srv(
            start_controllers=self._stopped_controllers,
            stop_controllers=self._started_controllers,
            strictness=SwitchControllerRequest.STRICT
        )

        self._started_controllers = None
        self._stopped_controllers = None

        if not ok:
            raise SwitchError('Reverting controllers failed.')


class ControllerManagerClient(object):
    def __init__(self, ns=''):
        """ Constructs a client to a ros_control ControllerManager.

        @param ns: ROS namespace containing the ControllerManager
        @type  ns: str
        """
        from controller_manager_msgs.srv import (ListControllers,
                                                 SwitchController)
        from rospy import ServiceProxy

        self._list_controllers_srv = ServiceProxy(
            ns + '/controller_manager/list_controllers', ListControllers,
            persistent=True)
        self._switch_controllers_srv = ServiceProxy(
            ns + '/controller_manager/switch_controller', SwitchController,
            persistent=True)

    def request(self, *controller_names):
        """ Returns a ControllerSwitcher for the requested controllers.

        @param controller_names: list of controller names to request
        @type  controller_names: [str]
        @return controller switcher for the requested controllers
        @rtype  ControllerSwitcher
        """
        return ControllerSwitcher(self, controller_names)


class JointStateClient(object):
    def __init__(self, robot, topic_name):
        from rospy import Subscriber
        from sensor_msgs.msg import JointState

        self._robot = robot
        self._subscriber = Subscriber(topic_name, JointState, self._callback)
        self._dof_mapping = {
            joint.GetName(): joint.GetDOFIndex() for joint in robot.GetJoints()
        }

    def _callback(self, joint_msg):
        from openravepy import KinBody

        # Map from joint names to DOF indices.
        dof_indices = []
        dof_values = []

        for name, position in zip(joint_msg.name, joint_msg.position):
            dof_index = self._dof_mapping.get(name)
            if dof_index is not None:
                dof_indices.append(dof_index)
                dof_values.append(position)

        # Update joint values.
        with self._robot.GetEnv():
            self._robot.SetDOFValues(dof_values, dof_indices,
                                     KinBody.CheckLimitsAction.Nothing)
