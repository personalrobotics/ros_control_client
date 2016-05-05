import logging
import rospy
from .futures import Future, FutureError


class SetPositionFailed(FutureError):
    def __init__(self, message, requested, executed):
        super(SetPositionFailed, self).__init__(message)

        self.requested = requested
        self.executed = executed


class SetPositionFuture(Future):
    def __init__(self, position_cmd):
        """Constructs a future that pends on the execution of a SetPosition command

        @param position_cmd: position command to execute
        @type  position_cmd: sensor_msgs.msg.JointState
        """
        super(SetPositionFuture, self).__init__()

        from actionlib import CommState
        from copy import deepcopy

        self._prev_state = CommState.PENDING
        self._position_cmd = deepcopy(position_cmd)

    def cancel(self):
        self._handle.cancel()

    def on_transition(self, handle):
        """Transition callback for the SetPositionAction client.

        @param handle: actionlib goal handle
        @type handle: actionlib.ClientGoalHandle
        """
        from actionlib import CommState

        state = handle.get_comm_state()

        # no change
        if state == self._prev_state:
            pass
        # Transition to the "done" state. This occurs when the
        # action completes for any reason (including an error).
        elif state == CommState.DONE:
            self._on_done(handle.get_terminal_state(), handle.get_result())

        self._prev_state = state

    def on_feedback(self, handle):
        """Dummy feedback callback fo rthe SetPositionAction client. No-op."""
        pass

    def _on_done(self, terminal_state, result):
        from actionlib import TerminalState, get_name_of_constant
        from pr_control_msgs.msg import SetPositionActionResult

        exception = SetPositionFailed(
            'SetPosition action failed ({:s}): {:s}'.format(
                get_name_of_constant(TerminalState, terminal_state),
                get_name_of_constant(SetPositionActionResult, result.success)),
            executed=self._position_cmd,
            requested=self._position_cmd)

        if terminal_state == TerminalState.SUCCEEDED:
            if result.success is True:
                self.set_result(result)
            else:
                self.set_exception(exception)
        elif terminal_state in [TerminalState.REJECTED,
                                TerminalState.RECALLED,
                                TerminalState.PREEMPTED]:
            self.set_cancelled()
        else:
            self.set_exception(exception)


class SetPositionClient(object):
    def __init__(self, ns, controller_name, timeout=0.0):
        """Consructs a client that sends pr_control_msgs/SetPosition actions

        @param ns: namespace for the ActionServer
        @type  ns: str
        @param controller_name: name of the controller
        @type  controller_name: str
        """

        from actionlib import ActionClient
        from pr_control_msgs.msg import SetPositionAction

        self.log = logging.getLogger(__name__)
        as_name = ns + '/' + controller_name + '/set_position'
        self._client = ActionClient(as_name, SetPositionAction)
        if not self._client.wait_for_server(rospy.Duration(timeout)):
            raise Exception('Could not connect to action server {}'.format(
                as_name))

    def execute(self, joint_state):
        """Execute a SetPosition action and return a SetPositionFuture

        @param  positional_joint_state: requested position
        @type   positional_joint_state: sensor_msgs.JointState
        @return future pending on the completion of the action
        @type   SetPositionFuture
        """
        from pr_control_msgs.msg import SetPositionGoal

        goal_msg = SetPositionGoal()
        goal_msg.command.header.stamp = rospy.Time.now()
        goal_msg.command.position = joint_state

        self.log.info('Sending SetPositionGoal: {}'.format(goal_msg))
        action_future = SetPositionFuture(joint_state)
        action_future._handle = self._client.send_goal(
            goal_msg,
            transition_cb=action_future.on_transition,
            feedback_cb=action_future.on_feedback
        )
        return action_future
