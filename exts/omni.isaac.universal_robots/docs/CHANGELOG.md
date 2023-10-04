# Changelog

## [0.3.2] - 2023-02-01

### Changed
- Values for surface griper torque and force limits for bin filling so that the gripper breaks after filling the bin.

## [0.3.1] - 2022-12-09

### Changed
- Values for surface griper torque and force limits.

## [0.3.0] - 2022-07-26

### Removed
- Removed GripperController class and used the new ParallelGripper class instead.

### Changed
- Changed gripper_dof_indices argument in PickPlaceController to gripper.
- Changed gripper_dof_indices argument in StackingController to gripper.

## [0.2.3] - 2022-07-22

### Fixed
- Bug with adding a custom usd for manipulator

## [0.2.2] - 2022-05-27

### Fixed
- Fixed bug: typo when loading UR10 RMPflow controller without suction gripper

## [0.2.1] - 2022-05-04

### Changed

- backwards compatible change to UR10's post_reset() implementation. Removed the hard coded override, and added the hard coded config as a default config in initialize(). Functionality equivalent but uses the underlying Articulation objects's default config functionality.

## [0.2.0] - 2022-05-02

### Changed
- Changed InverseKinematicsSolver class to KinematicsSolver class, using the new LulaKinematicsSolver class in motion_generation

## [0.1.5] - 2022-04-21

### Changed
-Updated RmpFlowController class init alongside modifying motion_generation extension

## [0.1.4] - 2022-04-07

### Fixed
- Adding a gripper to UR10

## [0.1.3] - 2022-03-25

### Changed
- Updated RmpFlowController class alongside changes to motion_generation extension

## [0.1.2] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.1] - 2021-12-02

### Changed
- Propagation of core api changes

## [0.1.0] - 2021-09-01

### Added
- Added UR10 class
