std_msgs/Header  header

geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration # optional
geometry_msgs/Wrench wrench

# Desired time from the trajectory start to arrive at this trajectory point.
builtin_interfaces/Duration time_from_start