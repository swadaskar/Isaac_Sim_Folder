# Changelog

## [1.4.1] - 2023-02-22
### Added
- make sure replicator is stopped before calling wait_until_complete on closing application

## [1.4.0] - 2023-02-13
### Added
- add minimal app framework class

## [1.3.0] - 2023-02-07
### Changed
- call replicator wait_until_complete on closing application

## [1.2.3] - 2023-01-20
### Fixed
- Startup warnings

## [1.2.2] - 2023-01-18
### Fixed
- Error when viewport extension was not loaded


## [1.2.1] - 2022-12-11
### Fixed
- Error message when closing stage before closing simulation app


## [1.2.0] - 2022-10-25
### Changed
- Prepare UI focuses on content tab and hides samples to improve startup times. 

## [1.1.0] - 2022-10-14
### Added
- fast shutdown config option
### Fixed
- issue where fast shutdown caused jupyter notebooks to crash

## [1.0.2] - 2022-10-03
### Fixed
- Fixes for kit 104.0

## [1.0.1] - 2022-10-02

### Fixed
- Crash when closing

## [1.0.0] - 2022-09-12

### Removed
- memory_report config flag

## [0.2.1] - 2022-07-25

### Added
- Increase hang detection timeout (OM-55578)

## [0.2.0] - 2022-06-22

### Deprecated

- deprecated memory report in favor of using statistics logging utility

## [0.1.10] - 2022-06-13

### Added
- added physics device parameter for setting CUDA device for GPU physics simulation

## [0.1.9] - 2022-04-27

### Changed
- a .kit experience file can now reference other .kit files from the apps folder

## [0.1.8] - 2022-04-13

### Fixed
- Comment in simulation_app.py

## [0.1.7] - 2022-03-31

### Fixed
- Dlss is now loaded properly on startup

## [0.1.6] - 2022-03-24

### Added
- Multi gpu flag to config

### Changed
- Make startup/close logs timestamped

## [0.1.5] - 2022-02-22

### Added
- Windows support

## [0.1.4] - 2022-01-27

### Added
- memory_report to launch config. The delta memory usage is printed when the app closes.
- automatically add allow-root if running as root user

## [0.1.3] - 2021-12-21

### Changed
- Simulation App starts in cm instead of m to be consistent with the rest of isaac sim.

## [0.1.2] - 2021-12-07

### Added
- reset_render_settings API to reset render settings after loading a stage.
- fix docstring for antialiasing

## [0.1.1] - 2021-11-30

### Changed
- Remove omni.isaac.core and omni.physx dependency
- Changed shutdown print statements to make them consistent with startup

## [0.1.0] - 2021-11-30

### Changed
- Tagged Initial version of SimulationApp
