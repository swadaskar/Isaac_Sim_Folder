# Changelog

## [1.2.0] - 2023-03-08
### Changed
- moved standalone script to Isaac standalone examples

## [1.1.6] - 2023-01-19
### Fixed
- missing button error when running tests

## [1.1.5] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.1.4] - 2022-11-10
### Changed
- Updated determinism settings to include omni.isaac.core World

## [1.1.3] - 2022-10-21
### Changed
- Updated UR10 tests to use Nucleus Asset

## [1.1.2] - 2022-10-21
### Changed
- Updated Golden values due to modified PD gains in Franka asset
- Updated settings to add determinacy to test case in Kit 104

## [1.1.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.1.0] - 2022-08-30

### Changed
- Remove direct legacy viewport calls
## [1.0.1] - 2022-07-18

### Changed
- Removed debug_tools folder with ScreenPrinter class to use omni.isaac.ui.ScreenPrinter instead

## [1.0.0] - 2022-07-06

### Changed
- Modified workflow for adding robots and controllers to robot_benchmark extension to be mostly code-based
- Modified standalone script to run every permutation of (environment,robot,controller) that are passed in as arguments
- Modigified robot_benchmark.py to teleport robots to a constant starting position that can be configuring in ./benchmark_config

### Added
- Added RRT planner
- Added RRT+RMP hybrid planner
- Added example for adding example robot and controller

## [0.4.0] - 2022-05-09

### Changed
- Updated all hard coded USD object values to meters

## [0.3.2] - 2022-05-01

### Changed
- Removed redundant robot initialization that caused warnings in test case

## [0.3.1] - 2022-04-01

### Changed
- Updated test golden values with change to motion_generation extension
- Added verbosity to test failures

## [0.3.0] - 2022-03-25

### Changed
- Updated extension alongside motion_generation to use MotionPolicy directly

## [0.2.1] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.2.0] - 2022-2-10

### Changed
- Updated extension alongside motion_generation to use Core API

## [0.1.1] - 2021-10-09

### Changed
- Restructure files

## [0.1.0] - 2021-08-13

### Added
- Initial version of Isaac Sim Robot Benchmark Extension
