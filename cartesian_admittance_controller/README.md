# cartesian_admittance_controller

This is a cartesian version of the package `admittance_controller` from [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/master).

## Control law

The controller imposes the following cartesian behavior

$$
\begin{align}
  M \ddot{e} + D \dot{e} + K e = - f_{ext}
\end{align}
$$

where $e = p^d - p$, $M$ and $D$ are the (positive definite) desired inertia and damping matrix, respectively, and $K^d$ is the (positive semi-definite) desired stiffness matrix.

The cartesian acceleration command $\ddot{p}^c$ is therefore computed as

$$
\begin{align}
  \ddot{p}^c = \ddot{p}^d + M^{-1} \left( D \dot{e} + K \dot{e} + f_{ext} \right)
\end{align}
$$

resulting in a joint acceleration command $\ddot{q}^c$ computed using the `kinematics_interface`.

In order to accomodate for `position` and `velocity` command interfaces, the joint position command $q^c$ and velocity command $\dot{q}^c$ are computed using Euler integration such that

$$
\begin{align}
  \dot{q}^c &= \dot{q} + T_s \ddot{q}^c \\
  {q}^c &= q + T_s \dot{q}^c
\end{align}
$$

where $q$ and $\dot{q}$ are the **measured** joint position and velocity, respectively, and $T_s$ is the controller sampling time.

## Topics

- `~/compliant_frame_reference` (input topic) [`cartesian_control_msgs::msg::CompliantFrameTrajectory`]

  Target (cartesian) compliant frame containing at least a pose, a twist and a desired compliance (stiffness, damping, inertia).

- `~/state` (output topic) [`control_msgs::msg::AdmittanceControllerState`]

  Topic publishing controller internal states.

  **TODO(tpoignonec)** Use custom message `VicControllerState`


## ros2_control interfaces

### States

The state interfaces are defined with ``joints`` and ``state_interfaces`` parameters as follows: ``<joint>/<state_interface>``.
The necessary state interfaces are ``position`` and ``velocity``.
**Both are mandatory!**

Additionally, the `Force Torque Sensor` semantic component is used and requires force state interfaces.

### Commands

Ideally, the control interface is a joint ``velocity`` group.
The controller can also use ``position`` or ``acceleration``.
