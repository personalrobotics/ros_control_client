from .futures import Future, FutureError


class TrajectoryExecutionFailed(FutureError):
    def __init__(self, message, requested, executed):
        super(TrajectoryExecutionFailed, self).__init__(message)

        self.requested = requested
        self.executed = executed


class TrajectoryFuture(Future):
    def __init__(self, traj_requested):
        """ Construct a future that represents the execution of a trajectory.

        @param traj_requested: requested trajectory
        @type  traj_requested: trajectory_msgs.msg.JointTrajectory
        """
        super(TrajectoryFuture, self).__init__()

        from actionlib import CommState
        from copy import deepcopy
        from trajectory_msgs.msg import JointTrajectory

        self._prev_state = CommState.PENDING
        self._traj_requested = deepcopy(traj_requested)
        self._traj_executed = JointTrajectory(
            joint_names=traj_requested.joint_names
        )

    def cancel(self):
        self._handle.cancel()

    def requested(self):
        """ Returns the trajectory requested to be executed.

        @return requested trajectory
        @type   trajectory_msgs.msg.JointTrajectory
        """
        from copy import deepcopy

        return deepcopy(self._traj_requested)

    def partial_result(self):
        """ Returns the trajectory as executed so far.

        This trajectory reports the controller's actual state over time, which
        may deviate from the requested trajectory. The header.stamp field will
        be zero until execution begins, then will change to indicate the time
        at which the trajectory began execution. All time_from_start values are
        relative to this timestamp.

        @return requested trajectory
        @type   trajectory_msgs.msg.JointTrajectory
        """
        from copy import deepcopy

        with self.lock:
            return deepcopy(self._traj_executed)

    def on_transition(self, handle):
        """ Transition callback for the FollowJointTrajectoryAction client.

        @param handle: actionlib goal handle
        @type  handle: actionlib.ClientGoalHandle
        """
        from actionlib import CommState

        state = handle.get_comm_state()

        # Transition to the "done" state. This occurs when the trajectory
        # finishes for any reason (including an error).
        if state == self._prev_state:
            pass
        elif state == CommState.DONE:
            self._on_done(handle.get_terminal_state(), handle.get_result())

        self._prev_state = state

    def on_feedback(self, feedback_msg):
        """ Feedback callback for the FollowJointTrajectoryAction client.

        @param msg: feedback message
        @type msg: control_msgs.msg.FollowJointTrajectoryFeedback
        """
        msg = feedback_msg.feedback

        with self.lock:
            if not self._traj_executed.header.stamp:
                self._traj_executed.header.stamp = (msg.header.stamp
                                                  - actual.time_from_start)

            self._traj_executed.points.append(msg.actual)

    def _on_done(self, terminal_state, result):
        from actionlib import TerminalState, get_name_of_constant
        from copy import deepcopy
        from control_msgs.msg import FollowJointTrajectoryResult

        exception = TrajectoryExecutionFailed(
            'Trajectory execution failed ({:s}): {:s}'.format(
                get_name_of_constant(TerminalState, terminal_state),
                get_name_of_constant(FollowJointTrajectoryResult,
                                     result.error_code)
            ),
            executed=self.partial_result(),
            requested=deepcopy(self._traj_requested)

        )

        if terminal_state == TerminalState.SUCCEEDED:
            # Trajectory execution succeeded. Return the trajectory.
            if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
                self.set_result(self._traj_executed)
            # Trajectory execution failed. Raise an exception.
            else:
                self.set_exception(exception)
        # Goal was cancelled. Note that this could have been one by another
        # thread or process, so _cancelled may be False.
        elif terminal_state in [TerminalState.PREEMPTED, TerminalState.RECALLED]:
            self.set_cancelled()
        else:
            self.set_exception(exception)


class FollowJointTrajectoryClient(object):
    def __init__(self, ns, timeout=0.0):
        """ Constructs a client that executes JointTrajectory messages.

        @param ns: namespace for the FollowJointTrajectoryAction server
        @type  ns: str
        """
        from actionlib import ActionClient
        from control_msgs.msg import FollowJointTrajectoryAction
        from rospy import Duration

        self._client = ActionClient(ns, FollowJointTrajectoryAction)
        if not self._client.wait_for_server(Duration(timeout)):
            raise Exception('Could not connect to action server %s' % ns)

    def execute(self, traj_msg):
        """ Execute a JointTrajectory message and return a TrajectoryFuture.

        @param  traj_msg: requested trajectory
        @type   traj_msg: trajectory_msgs.msg.JointTrajectory
        @return future representing the execution of the trajectory
        @rtype  TrajectoryFuture
        """
        from control_msgs.msg import FollowJointTrajectoryGoal

        goal_msg = FollowJointTrajectoryGoal()
        goal_msg.trajectory = traj_msg

        traj_future = TrajectoryFuture(traj_msg)
        traj_future._handle = self._client.send_goal(
            goal_msg,
            transition_cb=traj_future.on_transition,
            feedback_cb=traj_future.on_feedback
        )
        return traj_future
