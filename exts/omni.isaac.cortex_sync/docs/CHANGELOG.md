# Changelog

## [0.2.1] - 2023-02-23

### Fixed
- CortexControlRos joint states callback queried for dof_names before robot is initialized.

## [0.2.0] - 2023-02-14

### Fixed
- Cleaning pass: Docstrings, comments, type hints
- Pruned some unused utils.
- Minor API update to `CortexObjectsRos` and `CortexSimObjectsRos` to fix hardcoded `in_coords`.
- Uses defaults for backward compatibility. 

## [0.1.5] - 2023-01-27

### Fixed
- extension was not importing module, added python.module to extension.toml


## [0.1.4] - 2022-12-12

### Changed
- `CortexControlRos` on sync with external controller now does a soft reset on the `MotionCommander` object rather than a hard reset of the entire cortex pipeline.

## [0.1.3] - 2022-12-08

### Changed
- Remove old extension versions of `cortex_{ros,sim}`

## [0.1.2] - 2022-12-08

### Fixed
- Changed CortexControlRos to reset the entire cortex pipeline when synchronizing with a sim/physical robot rather than just the commanders.

## [0.1.1] - 2022-10-28

### Changed
- Modularize `cortex_{ros,sim}` into simple objects wrapping robots rather than extensions.
- Improvements/bugfixes to cortex control (including low-level controller) to handle pausing/stopping and restarting the simulator with the controller running.


## [0.1.0] - 2022-10-24

### Added
- Initial version of `cortex_sync`. Pulled ROS components from `omni.isaac.cortex`.
