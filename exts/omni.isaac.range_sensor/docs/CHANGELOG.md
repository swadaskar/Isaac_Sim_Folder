# Changelog

## [0.8.1] - 2023-02-08
### Fixed
- Lidar Semantic IDs were incorrect when rotation rate was not zero

## [0.8.0] - 2023-01-25
### Added
- Fabric support

## [0.7.2] - 2023-01-25
### Fixed
- remove un-needed cpp ogn files from extension
## [0.7.1] - 2023-01-05
### Fixed
- Bug where parent xform scale would cause lidar beams to be incorrect. 
- onclick_fn warning when creating UI

## [0.7.0] - 2022-12-12
### Added
- Option to either stream data or repeat data for generic sensor

## [0.6.4] - 2022-12-12
### Fixed
- Data streaming in menu for lidar and ultrasonic sensor are now updating and refreshed when new sensors gets loaded

## [0.6.3] - 2022-11-17
### Fixed
- Zenith Range should be 0,0 if high lod is false and lidar is 2d

## [0.6.2] - 2022-11-03
### Fixed
- Multiple lidars now have the same semantic colors, using a fixed random seed for debug semantic color generation

## [0.6.1] - 2022-10-03
### Fixed
- Fixes for kit 104.0

## [0.6.0] - 2022-09-27
### Changed
- tests to use nucleus assets

### Removed
- usd files local to extension

## [0.5.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.5.0] - 2022-09-01

### Changed
- Remove legacy viewport calls from samples

## [0.4.4] - 2022-08-31

### Fixed
- Generic Sensor API updated to use getPointCloud instead of getHitPosData

### Changed
- Remove direct legacy viewport calls
## [0.4.3] - 2022-08-14

### Fixed
- Semantic APIs that ended with a random id are supported now, only the first semantic API applied is used. 
- Active semantic IDs only updated if Lidar was moved/changed, IDs are now cleared each frame to fix this

## [0.4.2] - 2022-05-26

### Fixed
- Lidar semantics not getting visualized properly

## [0.4.1] - 2022-05-02

### Changed
- Output data type from float to pointf in Isaac Read Lidar Point Cloud OG Node

## [0.4.0] - 2022-04-27

### Added
- Isaac Read Lidar Point Cloud OG Node

### Changed
- Improved perf for Isaac Read Lidar Beams OG Node

## [0.3.0] - 2022-04-26

### Added
- Isaac Read Lidar Beams OG Node

## [0.2.2] - 2022-03-10

### Changed
- Use orient op for commands that create sensors

## [0.2.1] - 2022-03-04

### Changed
- Lidar sensor is now fully multithreaded per sensor and per ray
- Improved use of tasking framework and simplified scan logic
- added fabric support to lidar

## [0.2.0] - 2022-02-11

### Added
- Sequence number to track frame count
- AzimuthRange and ZenithRange in Lidar sensor to track beginning and end angles of a scan

### Fixed
- Lidar sensor to prevent first frame from being outputted

## [0.1.5] - 2021-12-13

### Fixed
- generic sensor outputting sensor pattern image file

## [0.1.4] - 2021-07-23

### Added
- USS material support

## [0.1.3] - 2021-07-12

### Added
- New UI

## [0.1.2] - 2021-06-03

### Added
- Add generic, lidar, radar, ultrasonic range sensors

## [0.1.1] - 2020-12-11

### Added
- Add lidar unit tests to extension

## [0.1.0] - 2020-12-07

### Added
- Initial version of Isaac Sim Range sensor extension
