std_msgs/Header  header

std_msgs/String control_mode # admittance / impedance

# Cartesian compliance reference
geometry_msgs/Pose desired_pose
geometry_msgs/Twist desired_velocity
geometry_msgs/Accel desired_acceleration # optional, set to zero if unused
geometry_msgs/Wrench desired_wrench # optional, set to zero if unused

std_msgs/Float64MultiArray desired_inertia
std_msgs/Float64MultiArray desired_damping
std_msgs/Float64MultiArray desired_stiffness

# Actual Cartesian robot state
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration # optional, set to zero if unused
bool has_valid_wrench
geometry_msgs/Wrench wrench

std_msgs/Float64MultiArray rendered_inertia  # optional
std_msgs/Float64MultiArray rendered_damping  # optional
std_msgs/Float64MultiArray rendered_stiffness  # optional

# Commands
float64[] joint_command_position  # optional
float64[] joint_command_velocity  # optional
float64[] joint_command_acceleration  # optional
float64[] joint_command_effort  # optional

# Misc.
std_msgs/Float64MultiArray natural_inertia  # optional
cartesian_control_msgs/KeyValues diagnostic_data
