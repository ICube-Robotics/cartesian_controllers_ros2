# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations

std_msgs/Header header

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the Cartesian frame at each time point.

cartesian_control_msgs/CartesianTrajectoryPoint[] cartesian_trajectory_points
cartesian_control_msgs/CartesianCompliance[] compliance_at_points
