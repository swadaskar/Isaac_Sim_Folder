# Changelog

## [2.0.0] - 2022-10-17
### Deprecated
- Deprecated omni.isaac.isaac_sensor and renamed to omni.isaac.sensor

## [0.1.14] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.1.13] - 2022-08-17

### Fixed
- Fix warnings generated on stage close

## [0.1.12] - 2022-07-20

### Added
- Check for setting to disable viewport extension

## [0.1.11] - 2022-06-13

### Added
- Pass physics device ID to simulation app for GPU physics
- Added support for headless gym app

## [0.1.10] - 2022-05-18

### Added
- Set omnihydra scene graph instancing setting for instanced assets

## [0.1.9] - 2022-05-17

### Changed
- Add device ID when setting GPU device to World

## [0.1.8] - 2022-05-14

### Changed
- Add __init__.py to module root to import correctly

## [0.1.7] - 2022-05-12

### Changed
- Start simulation automatically for multi-threaded script
- Terminate process when running multi-threaded script in headless mode

## [0.1.6] - 2022-05-11

### Fixed
- Assgin device to World based on config dictionary

## [0.1.5] - 2022-05-05

### Changed
- Updated vec_env_mt to enable flatcache when self._world.get_physics_context()._use_flatcache is set to True
- Moved enable_flatcache call from vec_env_base to physics_context in omni.isaac.core

## [0.1.4] - 2022-05-03

### Fixed
- Fixed flag for world reset when simulation restarts.

## [0.1.3] - 2022-05-02

### Fixed
- Fixed RL restart in multi-threaded VecEnv when simulation is stopped from UI.

## [0.1.2] - 2022-04-29

### Changed
- Refactor base VecEnv class to support more general usage.

## [0.1.1] - 2022-04-28

### Added
- Enabled omni.physx.flatcache when running RL with GPU pipeline

### Removed
- Moved RL Base Task to examples repo

### Fixed
- Fixed variable naming in VecEnvMT

## [0.1.0] - 2022-03-30

### Added
- Added Initial Classes