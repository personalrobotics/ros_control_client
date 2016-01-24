import logging

logger = logging.getLogger('ros_control_client_py.util')

# TODO is this already implemented in prpy?
def find_resource(relative_path, package):
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project=package, search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading resource "{:s}" failed.'.format(
                      relative_path))


def or_to_ros_trajectory(robot, traj, dof_indices = None, time_tolerance=0.01):
    """ Convert an OpenRAVE trajectory to a ROS trajectory.

    @param robot: OpenRAVE robot
    @type  robot: openravepy.Robot
    @param traj: input trajectory
    @type  traj: openravepy.Trajectory
    @param dof_indices: indices to convert
    @type dof_indices: [int]
    @param time_tolerance: minimum time between two waypoints
    @type  time_tolerance: float
    """
    import numpy
    from rospy import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    assert time_tolerance >= 0.

    if traj.GetEnv() != robot.GetEnv():
        raise ValueError(
            'Robot and trajectory are not in the same environment.')

    cspec = traj.GetConfigurationSpecification()

    if dof_indices == None:
        dof_indices, _ = cspec.ExtractUsedIndices(robot)
    else:
        used_indices, _ = cspec.ExtractUsedIndices(robot)
        dof_indices = list(set(used_indices) & set(dof_indices))

    traj_msg = JointTrajectory(
        joint_names=[ robot.GetJointFromDOFIndex(dof_index).GetName()
                      for dof_index in dof_indices ]
    )

    time_from_start = 0.
    prev_time_from_start = 0.
    
    for iwaypoint in xrange(traj.GetNumWaypoints()):
        waypoint = traj.GetWaypoint(iwaypoint)

        dt = cspec.ExtractDeltaTime(waypoint)
        q = cspec.ExtractJointValues(waypoint, robot, dof_indices, 0)
        qd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 1)
        qdd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 2)

        if dt is None:
            raise ValueError('Trajectory is not timed.')
        elif q is None:
            raise ValueError('Trajectory does not contain joint values')
        elif qdd is not None and qd is None:
            raise ValueError('Trajectory contains accelerations,'
                             ' but not velocities.')

        # Duplicate waypoints break trajectory execution, so we explicitly
        # filter them out. Note that we check the difference in time between
        # the current and the previous waypoint, not the raw "dt" value. This
        # is necessary to support very densely sampled trajectories.
        time_from_start += dt
        deltatime = time_from_start - prev_time_from_start

        if iwaypoint > 0 and deltatime < time_tolerance:
            logger.warning('Skipped waypoint %d because deltatime is %.3f < %.3f.',
                deltatime, time_tolerance)
            continue

        prev_time_from_start = time_from_start

        # Create the waypoint.
        traj_msg.points.append(
            JointTrajectoryPoint(
                positions=list(q),
                velocities=list(qd) if qd is not None else [],
                accelerations=list(qdd) if qdd is not None else [],
                time_from_start=Duration.from_sec(time_from_start)
            )
        )

    assert abs(time_from_start - traj.GetDuration()) < time_tolerance
    return traj_msg

def pad_ros_trajectory(robot, traj_ros, joint_names):
    """ Add constant values for DOFs missing from a ROS trajectory.

    @param robot: OpenRAVE robot
    @type  robot: openravepy.Robot
    @param traj_ros: ROS trajectory message
    @type  traj_ros: trajectory_msgs.msg.JointTrajectory
    @param joint_names: list of joint names to ensure are present
    @type  joint_names: [str]
    """
    missing_names = set(joint_names).difference(traj_ros.joint_names)
    missing_values = [
        robot.GetJoint(joint_name).GetValue(0)
        for joint_name in missing_names
    ]
    zero_values = [0.0] * len(missing_values)

    for waypoint in traj_ros.points:
        if len(waypoint.positions) > 0:
            waypoint.positions.extend(missing_values)
        if len(waypoint.velocities) > 0:
            waypoint.velocities.extend(zero_values)
        if len(waypoint.accelerations) > 0:
            waypoint.accelerations.extend(zero_values)

    traj_ros.joint_names.extend(missing_names)
    return traj_ros
