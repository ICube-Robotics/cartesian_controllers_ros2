ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/src/cartesian_controllers_ros2
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip
RUN apt install -y git tree clang-format-14 cppcheck pre-commit
RUN cd ros2_dev/src && \
    vcs import . < cartesian_controllers_ros2/cartesian_controllers_ros2.repos
