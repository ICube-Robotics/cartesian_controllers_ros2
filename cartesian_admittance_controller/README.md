# cartesian_admittance_controller

This is a cartesian version of the package `admittance_controller` from [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/master).

> **Warning**
>
> This package is currently under development and possibly unsafe if used to control an actual robot!

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

and integrated to obtain the cartesian velocity command $\dot{p}^c$.

The joint velocity command $\dot{q}^c$ is computed from $\dot{p}^c$ using the `kinematics_interface`.

In order to accommodate for both the `position` and `velocity` command interfaces, the joint velocity command $\dot{q}^c$ is integrated to provide a joint position command ${q}^c$ such that

$$
\begin{align}
  {q}^c &= q + T_s \dot{q}^c
\end{align}
$$

where $q$ is the **measured** joint position and $T_s$ is the controller sampling time.

## Topics

- `~/compliant_frame_reference` (input topic) [`cartesian_control_msgs::msg::CompliantFrameTrajectory`]

  Target (cartesian) compliant frame containing at least a pose, a twist.

  Additionally, a desired compliance (stiffness, damping, inertia) can be provided for each reference cartesian state in the trajectory. Otherwise, the ros parameters are used. For instance you could provide the following ros parameters:

  ```yaml
  <controller_name>:
    admittance:
      frame:
        id: <admittance_reference_frame> # e.g., tool0 or world

      inertia: [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]
      damping_ratio: [1., 1., 1., 1., 1., 1.]
      stiffness: [200., 200., 200., 50., 50., 50.]
  ```

- `~/state` (output topic) [`control_msgs::msg::AdmittanceControllerState`]

  Topic publishing controller internal states.

  **TODO(tpoignonec)** Implements + use custom message `VicControllerState`


## ros2_control interfaces

### States

The state interfaces are defined with ``joints`` and ``state_interfaces`` parameters as follows: ``<joint>/<state_interface>``.
The necessary state interfaces are ``position`` and ``velocity``.
**Both are mandatory!**

Additionally, the `Force Torque Sensor` semantic component is used and requires force state interfaces.

### Commands

Ideally, the control interface is a joint ``velocity`` group.
The controller can also use a ``position`` command interface.
