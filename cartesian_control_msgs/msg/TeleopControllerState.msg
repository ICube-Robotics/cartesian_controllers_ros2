std_msgs/Header  header

# Teleop controller input data
std_msgs/Bool workspace_is_engaged

geometry_msgs/Pose x_1
geometry_msgs/Twist x_1_dot
geometry_msgs/Accel x_1_ddot
geometry_msgs/Wrench f_1

std_msgs/Float64MultiArray desired_inertia_1
std_msgs/Float64MultiArray desired_damping_1
std_msgs/Float64MultiArray desired_stiffness_1

geometry_msgs/Pose x_2
geometry_msgs/Twist x_2_dot
geometry_msgs/Accel x_2_ddot
geometry_msgs/Wrench f_2

std_msgs/Float64MultiArray desired_inertia_2
std_msgs/Float64MultiArray desired_damping_2
std_msgs/Float64MultiArray desired_stiffness_2

# Teleop controller ref
geometry_msgs/Pose x_1_d
geometry_msgs/Twist x_1_dot_d
geometry_msgs/Accel x_1_ddot_d
geometry_msgs/Wrench f_1_d

std_msgs/Float64MultiArray rendered_inertia_1
std_msgs/Float64MultiArray rendered_damping_1
std_msgs/Float64MultiArray rendered_stiffness_1

geometry_msgs/Pose x_2_d
geometry_msgs/Twist x_2_dot_d
geometry_msgs/Accel x_2_ddot_d
geometry_msgs/Wrench f_2_d

std_msgs/Float64MultiArray rendered_inertia_2
std_msgs/Float64MultiArray rendered_damping_2
std_msgs/Float64MultiArray rendered_stiffness_2

# Misc.
cartesian_control_msgs/KeyValues diagnostic_data
