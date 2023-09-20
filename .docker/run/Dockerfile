ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/src/cartesian_controllers_ros2
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip
RUN apt install -y git clang-format-14 cppcheck
RUN pip install pre-commit
RUN apt update && \
    cd ros2_dev/src && \
    vcs import . < cartesian_controllers_ros2/cartesian_controllers_ros2.repos
RUN cd ros2_dev/ && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r
RUN cd ros2_dev/ && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
# COPY .docker/run/ros_entrypoint.sh /ros_entrypoint.sh
# RUN chmod 755 /ros_entrypoint.sh
# ENTRYPOINT ["ros_entrypoint.sh"]
CMD ["bash"]
