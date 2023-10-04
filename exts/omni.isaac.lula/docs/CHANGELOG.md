# Changelog

## [1.4.0] - 2023-02-23

### Changed

- Upgraded Lula from release 0.8.1 to release 0.8.2.  This fixes a bug in the trajectory generator's task-space path conversion that could result in suboptimal interpolation of orientations.  In addition, a new option was added to the trajectory generator allowing user specification of time values at waypoints.

## [1.3.1] - 2022-11-30

### Changed

- Upgraded Lula from release 0.8.0 to release 0.8.1, fixing a couple minor bugs and adding a `__version__` string to the Lula python module.

## [1.3.0] - 2022-11-16

### Changed

- Upgraded Lula from release 0.7.1 to 0.8.0.  Among other improvements, this adds a flexible trajectory generator and collision sphere generator.  In addition, the robot description file format has been simplified, with "root_link" and "cspace_to_urdf_rules" now optional and "composite_task_spaces" and "subtree_root_link" removed/ignored.

## [1.2.1] - 2022-10-09

### Fixed

- Issue where linux version of extension was being loaded on windows

## [1.2.0] - 2022-10-06

### Changed

- Changed default log level to WARNING

## [1.1.0] - 2022-04-16

### Changed

- Removed wheel from extension, provide installed wheel as part of extension instead. This removes the need for runtime installation.

## [1.0.0] - 2022-01-13

### Changed

- Updated Lula from release 0.7.0 to 0.7.1.  This fixes a bug in Lula's kinematics that had the potential to cause a segfault for certain robots.

## [0.1.0] - 2021-09-20

### Added

- Initial version of Isaac Sim Lula Extension
