# cartesian_admittance_controller

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
