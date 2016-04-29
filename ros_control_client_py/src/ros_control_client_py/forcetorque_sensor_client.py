from .futures import Future, FutureError


class TriggerFailed(FutureError):
    def __init__(self, message, requested, executed):
        super(TriggerFailed, self).__init__(message)

        self.requested = requested
        self.executed = executed


class TriggerFuture(Future):
    def __init__(self):
        """Constructs a future that pends on the execution of a Trigger command
        """
        super(TriggerFuture, self).__init__()

        from actionlib import CommState

        self._prev_state = CommState.PENDING

    def cancel(self):
        self._handle.cancel()

    def on_transition(self, handle):
        """Transition callback for the TriggerAction client.

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
        """Dummy feedback callback for the TriggerAction client. No-op."""
        pass

    def _on_done(self, terminal_state, result):
        from actionlib import TerminalState, get_name_of_constant
        from pr_control_msgs.msg import TriggerActionResult

        exception = TriggerFailed(
            'Trigger action failed ({:s}): {:s}'.format(
                get_name_of_constant(TerminalState, terminal_state),
                get_name_of_constant(TriggerActionResult, result.success)))

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


class TriggerClient(object):
    def __init__(self, ns, controller_name, timeout=0.0):
        """Consructs a client that sends pr_control_msgs/Trigger actions

        @param ns str: namespace for the ActionServer
        @param controller_name str: name of the controller
        """

        from actionlib import ActionClient
        from pr_control_msgs.msg import TriggerAction
        from rospy import Duration

        self._client = ActionClient(ns + '/' + controller_name, TriggerAction)
        if not self._client.wait_for_server(Duration(timeout)):
            raise Exception('Could not connect to action server %s' % ns)

    def execute(self):
        """Trigger the action and return a TriggerFuture

        @return future pending on the completion of the action
        @type   TriggerFuture
        """
        from pr_control_msgs.msg import TriggerActionGoal

        goal_msg = TriggerActionGoal()

        action_future = TriggerFuture()
        action_future._handle = self._client.send_goal(
            goal_msg,
            transition_cb=action_future.on_transition,
            feedback_cb=action_future.on_feedback
        )
        return action_future
