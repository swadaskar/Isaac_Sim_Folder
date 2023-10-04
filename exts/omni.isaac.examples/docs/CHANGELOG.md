# Changelog
## [1.5.6] - 2023-02-24
### Fixed
- Fix for 6 bolts in Nut and Bolt Demo

## [1.5.5] - 2023-02-20
### Fixed
- Improved Screw Controller in Nut and Bolt Demo

## [1.5.4] - 2023-02-08
### Fixed
- fixed vibrating table in Nut and Bolts Demo

## [1.5.3] - 2023-01-19
### Fixed
- missing button error when running tests

## [1.5.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.5.1] - 2022-12-09

### Changed
- Docs url for bin filling extension
## [1.5.0] - 2022-12-04

### Added
- Add Nut and Bolt example

## [1.4.0] - 2022-08-30

### Changed
- Remove direct legacy viewport calls

## [1.3.0] - 2022-08-29

### Removed
- Deprecated joint control and read articulation Dynamic control examples. articulation and articulation view provide similar functionality in core and are already documented. 

## [1.2.0] - 2022-06-17

### Added
- Path Planning Example with resiazable and movable walls

## [1.1.0] - 2022-06-16

### Added
- Added keep_window_open parameter to BaseSampleExtension to keep a sample's window visible after hot-reloading.

## [1.0.0] - 2022-05-20

### Removed
- ROS examples

## [0.3.0] - 2022-05-05

### Changed
- stage setting changed from cm to m.
- robofactory and roboparty uses hard coded position in meters (instead of cm)

## [0.2.0] - 2022-05-05

### Changed
- Jetbot keyboard example replaced by omnigraph_keyboard, using scripting omnigraph to resizing a cube instead of moving a robot

## [0.1.22] - 2022-04-21

### Changed
- Changed init functions for Franka, UR10, and DofBot controller classes alongside changes to motion_generation

## [0.1.21] - 2022-04-14

### Changed
- Replaced kaya holonomic controller with the generic controller

## [0.1.20] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()
- Jetbot Keyboard example and Kaya Gamepad example are now powered by Omnigraph

## [0.1.19] - 2022-2-10

### Changed
- Updated references to MotionGeneration

## [0.1.18] - 2022-01-27

### Added
- Cleaned up BaseSample UI
- Added Toggle Buttons to FollowTarget Example

## [0.1.17] - 2021-12-09

### Added
- Added a replay follow target example to showcase data logging and how to replay data in simulation.

## [0.1.16] - 2021-12-08

### Added
- Stop button greys out the buttons of the sample so the user presses reset for a proper reset.

## [0.1.15] - 2021-12-07

### Changed
- post_reset is not called after load anymore

### Added
- pre_reset function in base sample

### Fixed
- Follow Target example when adding an obstacle and then resetting

## [0.1.14] - 2021-12-02

### Changed
- Propagation of core api changes
- Rename kaya joystick to kaya gamepad

## [0.1.13] - 2021-11-05

### Changed
- Moved setting world settings logic to BaseSample instead of BaseSampleExtension
- Added pause after load button is pressed.

## [0.1.12] - 2021-11-01

### Changed
- renamed extension to omni.isaac.examples

## [0.1.11] - 2021-11-01

### Changed
- Added RoboFactory sample
- Changed name of multiple tasks to RoboParty Sample
- Added Follow Target sample
- Added Hello World Sample
- Added Simple Stack Sample

## [0.1.10] - 2021-07-26

### Changed
- New UI for Kaya Joystick and Jetbot Keyboard exampls

## [0.1.9] - 2021-07-23

### Changed
- Moved dofbot rmp config to lula package

## [0.1.8] - 2021-07-12

### Added
- add UI Utils to Import URDF

## [0.1.7] - 2021-07-08

### Added
- add dofbot rmp sample

## [0.1.6] - 2021-05-24

### Added
- Added dofbot sample
- Updated to latest physics api

## [0.1.5] - 2021-03-06

### Added
- Franka Replay Sample

## [0.1.4] - 2021-02-17

### Added
- update to python 3.7
- update to omni.kit.uiapp
- Update RMP sample to save data

## [0.1.3] - 2021-01-13

### Added
- Add support for 6DOF RMP target

## [0.1.2] - 2020-12-16

### Added
- RMP sample errors when adding obstacles

## [0.1.1] - 2020-12-14

### Added
- Fix issue with franka sample rmp config files not being found

## [0.1.0] - 2020-12-11

### Added
- Initial version of Isaac Sim Samples Extension
