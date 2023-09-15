# cartesian_admittance_controller

## Control law

The controller imposes the following cartesian behavior

$$
\begin{align}
  M^d \ddot{e} + D^d \dot{e} + K^d e = - f_{ext}
\end{align}
$$

where $e = p^d - p$, $M$ and $D$ are the (positive definite) desired inertia and damping matrix, respectively, and $K^d$ is the (positive semi-definite) desired stiffness matrix.

The cartesian target velocity $\dot{p}^r$ is therefore computed as

$$
\begin{align}
  \dot{p}^r = \dot{p}^d + D^{-1} \left( M\ddot{e} + K\dot{e} + f_{ext} \right)
\end{align}
$$

resulting in a joint target velocity $\dot{q}^r$ computed using the `kinematics_interface`.

In order to accomodate for `position` and `acceleration` command interfaces as well as for the `velocity` interface, the target joint position $q^r$ and acceleration $\ddot{q}^r$ are computed as

$$
\begin{align}
  {q}^r &= q + T_s \dot{q}^r && \text{\small(N.B., .)}\\
  \ddot{q}^r &= \cfrac{\dot{q}^r - \dot{q}^r_{k-1}}{T_s}
\end{align}
$$

where $q$ is the **measured** joint position, $\dot{q}^r_{k-1}$ is the joint velocity target sent at the previous time step, and $T_s$ is the controller sampling time.

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

This controller expects a downstream velocity controller to take care of the inverse kinematics computation.
The command interface is then the target cartesian twist with components named

- ``<velocity_cmd_interface>/lin_vx``
- ``<velocity_cmd_interface>/lin_vz``
- ``<velocity_cmd_interface>/lin_vy``
- ``<velocity_cmd_interface>/ang_vx``
- ``<velocity_cmd_interface>/ang_vy``
- ``<velocity_cmd_interface>/ang_vz``
