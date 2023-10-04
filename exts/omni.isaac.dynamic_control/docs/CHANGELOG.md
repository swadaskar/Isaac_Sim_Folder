# Changelog

## [1.2.3] - 2023-01-21
### Fixed
- Fix when multiple objects shared the same prim name using the isaac:nameOverride attribute for get_name
## [1.2.2] - 2022-10-20
### Fixed
- test golden values


## [1.2.1] - 2022-10-17
### Fixed
- explicitly handle prim deletion

## [1.2.0] - 2022-09-27
### Changed
- tests to use nucleus assets

### Removed
- usd files local to extension

## [1.1.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.1.0] - 2022-08-12

### Added 
- cMassLocalPose to DcRigidBodyProperties

## [1.0.1] - 2022-08-09

### Changed 
- Removed simple_articulation.usd, test_articulation_simple uses Nucleus asset

## [1.0.0] - 2022-05-11

### Changed
- non-backwards compatible change: dof indexing matches physx tensor API

## [0.2.2] - 2022-04-29

### Fixed
- Handle physx unwrapped revolute joints

## [0.2.1] - 2022-02-13

### Fixed
- Properly delete handles on prim deletion

## [0.2.0] - 2022-01-14

### Fixed
- Error message when waking up a kinematic rigid body
- Error message when setting linear velocity on a body with simulation disabled
- Error message when setting angular velocity on a body with simulation disabled

## [0.1.8] - 2021-08-16

### Added
- get_effort
- get_articulation_dof_efforts
- apply_body_torque

### Fixed
- inconsistent return types
- crash when stepping with a zero timestep as first step

### Changed
- apply_effort -> set_effort
- apply_articulation_dof_efforts -> set_articulation_dof_efforts
- handle refresh messages are printed out as info messages, instead of always printing
- apply_body_force now has a bool to specify if the force is global or local

## [0.1.7] - 2021-08-16

### Added
- Sleep functions for rigid bodies and articulations

### Changed
- return types use size_t instead of int where appropriate

## [0.1.6] - 2021-08-04

### Changed
- DriveMode is now either DRIVE_FORCE or DRIVE_ACCELERATION, default is acceleration
- Position/Velocity drive is not specified via DriveMode
- All API calls verify if simulating, return otherwise
- set_dof_properties will not enable or change drive limits
- set_dof_state takes StateFlags to apply specific states
- get_dof_state takes StateFlags to set which states to get

### Added
- State variables can be printed
- ArticulationProperties to control articulation settings
- RigidBodyProperties can control iteration counts and contact impulse settings
- get_articulation_properties
- set_articulation_properties
- get_articulation_dof_position_targets
- get_articulation_dof_velocity_targets
- get_articulation_dof_masses
- set_rigid_body_properties
- get_dof_properties
- unit tests for most articulation, rigid body, dof and joint apis
- utilities for common scene setup and testing

### Removed
- get_articulation_dof_state_derivatives
- DriveModes DRIVE_NONE, DRIVE_POS, DRIVE_VEL

### Fixed
- apply_body_force now applies a force at a point
- set_dof_properties does not break position/velocity drives
- dof efforts report correct forces/torques due to gravity
- when changing state of a dof or a root link, unrelated state values are not applied anymore
- set_dof_state applies efforts now
- get_dof_properties works correctly now

## [0.1.5] - 2021-07-23

### Added
- Split samples from extension

## [0.1.4] - 2021-07-14

### Added
- now works when running without editor/timeline and only physx events. 
- fixed crash with setting dof properties

## [0.1.3] - 2021-05-24

### Added
- force and torque sensors

## [0.1.2] - 2021-02-17

### Added
- update to python 3.7
- update to omni.kit.uiapp

## [0.1.1] - 2020-12-11

### Added
- Add unit tests to extension

## [0.1.0] - 2020-12-03

### Added
- Initial version of Isaac Sim Dynamic Control Extension
