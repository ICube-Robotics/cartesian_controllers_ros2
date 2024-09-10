# cartesian_controllers_ros2
Set of cartesian controllers for ros2_control


***The current devs are based on the jazzy ROS 2 distribution (Ubuntu 24.04 LTS)***

[![CI](https://github.com/ICube-Robotics/cartesian_controllers_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/ICube-Robotics/cartesian_controllers_ros2/actions/workflows/ci.yml)



### Installation of the package

```bash
# Create a ros2 workspace with a source dir inside
mkdir -p ws_cartesian_controllers/src

# GOTO source dir
cd ws_cartesian_controllers/src

# clone this repos
git clone https://github.com/ICube-Robotics/cartesian_controllers_ros2.git

# Use VCS to clone (and checkout the correct branches) the repositories of the deps
vcs import . < cartesian_controllers_ros2/cartesian_controllers_ros2.repos

# GOTO source dir
cd ..

# Source ROS2 distro
source /opt/ros/jazzy/setup.bash

# Install the dependencies of ALL packages
# sudo rosdep init && rosdep update
rosdep install --ignore-src --from-paths . -y -r

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Source this workspace
source install/setup.bash
```


### Examples

See the repos [ICube-Robotics/cartesian_controllers_ros2_examples](https://github.com/ICube-Robotics/cartesian_controllers_ros2_examples).
