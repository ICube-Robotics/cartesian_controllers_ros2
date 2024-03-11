std_msgs/Header  header

# Cartesian compliance reference
geometry_msgs/Pose desired_pose
geometry_msgs/Twist desired_velocity
geometry_msgs/Accel desired_acceleration # optional

std_msgs/Float64MultiArray desired_inertia
std_msgs/Float64MultiArray desired_damping
std_msgs/Float64MultiArray desired_stiffness

# Actual Cartesian robot state
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration # optional
geometry_msgs/Wrench wrench

std_msgs/Float64MultiArray rendered_inertia  # optional
std_msgs/Float64MultiArray rendered_damping  # optional
std_msgs/Float64MultiArray rendered_stiffness  # optional

# Commands
geometry_msgs/Twist robot_command_twist
float64[] joint_command_position
float64[] joint_command_velocity
float64[] joint_command_acceleration

# Misc.
cartesian_control_msgs/KeyValues diagnostic_data
