# Changelog

## [0.2.7] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.2.6] - 2022-11-28
### Fixed
- Fix duplicate symbol issue by statically linking against octomap for linux

## [0.2.5] - 2022-09-07
### Fixed
- Fixes for kit 103.5
## [0.2.4] - 2022-05-24

### Fixed
- block world default to meters

## [0.2.3] - 2022-05-16

### Fixed
- scale_to_meters parameter

## [0.2.2] - 2022-05-14

### Fixed
- Deadlock when generating data

## [0.2.1] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.2.0] - 2022-03-07

### Added
- Add ability to generate 3d occupancy data

### Changed
- Api's always return 3d point data

## [0.1.1] - 2020-09-15

### Added

- 3D Occupancy Map support that allow 2D maps to be generated from 3D volumes
- Block World Extension that allows a 2D map image to be converted to 3D geometry

### Changed

- Made UI Simpler
- Improved performance
- Added debug visualization

## [0.1.0] - 2020-07-08

### Added
- Initial version of Isaac Sim Occupancy Map Extension
