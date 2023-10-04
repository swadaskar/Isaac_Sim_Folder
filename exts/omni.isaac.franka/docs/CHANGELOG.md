# Changelog

## [0.4.0] - 2022-09-27

### Removed
- usd files local to extension

## [0.3.0] - 2022-07-26

### Removed
- Removed GripperController class and used the new ParallelGripper class instead.

### Changed
- Changed gripper_dof_indices argument in PickPlaceController to gripper
- Changed gripper_dof_indices argument in StackingController to gripper

### Added
- Added deltas argument in Franka class for the gripper action deltas when openning or closing.

## [0.2.1] - 2022-07-22

### Fixed
- Bug with adding a custom usd for manipulator

## [0.2.0] - 2022-05-02

### Changed
- Changed InverseKinematicsSolver class to KinematicsSolver class, using the new LulaKinematicsSolver class in motion_generation

## [0.1.4] - 2022-04-21

### Changed
- Updated RmpFlowController class init alongside modifying motion_generation extension

## [0.1.3] - 2022-04-13

### Changed
- Fix Franka units in gripper open config.

## [0.1.2] - 2022-03-25

### Changed
- Updated RmpFlowController class alongside changes to motion_generation extension

## [0.1.1] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.0] - 2021-09-01

### Added
- Added Franka class and and Franka Task Follower Class
