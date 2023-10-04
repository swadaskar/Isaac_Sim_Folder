# Changelog
## [5.6.3] - 2023-03-13
### Fixed
- Fix issue where lidar flatscan node as accessing data before it was ready

## [5.6.2] - 2023-03-06
### Fixed
- Default physics scene gravity is not read correctly by IMU

## [5.6.1] - 2023-03-02
### Fixed
- IMU sensor was not reading physics scene gravity correctly

## [5.6.0] - 2023-03-01
### Added
- Unlabeled points can be ignored when enabling pointcloud
### Changed
- removing an annotator detaches it
- update rtx lidar on app update 
### Fixed
- occlusion could not be enabled
- RTX lidar not returning data

## [5.5.1] - 2023-02-20
### Fixed
- ComputeFlatscan disconnected upon activation

## [5.5.0] - 2023-02-14
### Fixed
- RTX point cloud publishers publishing twice per frame by removing extra simulation gate nodes
- Sensor classes should only subscribe to the type of stage event they need

### Changed
- Use SdRenderVarPtr node instead of IsaacRenderVarToCpuPointer

## [5.4.4] - 2023-02-05
### Fixed
- Test failures, extra test warnings

## [5.4.3] - 2023-02-01
### Fixed
- Test failures, disabled solid state lidar test due to crash

## [5.4.2] - 2023-01-25
### Fixed
- remove un-needed cpp ogn files from extension

## [5.4.1] - 2023-01-19
### Fixed
- crashes during testing

## [5.4.0] - 2023-01-17
### Added
- normal at hit for rtx lidar

## [5.3.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [5.3.1] - 2022-12-14
### Fixed
- crash when deleting
- test_rtx_lidar passes now

## [5.3.0] - 2022-12-10

### Changed
- Switch debug draw nodes to use replicator writer backend
- hide rtx lidar menu from windows as rtx sensor is not supported
- disable rtx sensor tests on windows
## [5.2.4] - 2022-12-11
### Fixed
- IMU sensor example not working
- Broken docs link for imu sensor example

## [5.2.3] - 2022-12-09
### Fixed
- Crash when deleting rtx_lidar, again.

### Changed
- RTX nodes pass reasonable defaults if sensor is not found.

## [5.2.2] - 2022-12-05
### Fixed
- Crash when deleting rtx_lidar

## [5.2.1] - 2022-12-01
### Fixed
- IsaacSensorCreateContactSensor, IsaacSensorCreateImuSensor, IsaacSensorCreateRtxLidar and IsaacSensorCreateRtxRadar commands .do() only returns the created prim and not a tuple

## [5.2.0] - 2022-11-29

### Added
- Added contact sensor and IMU sensor wrappers.

## [5.1.1] - 2022-11-28

### Fixed
- crash with Solid State Lidar.

## [5.1.0] - 2022-11-22

### Added
- Added RTX lidar and Rotating physics lidar wrappers.

## [5.0.0] - 2022-11-21

### Added
- Camera class that provides many utilities to interact with a camera prim in stage

## [4.0.0] - 2022-11-16

### Added
- node template for rtx_radar
- nodes for rtx_radar: PrintRTXRadarInfo, ComputeRTXRadarPointCloud
- ReadRTXLidarData node for getting lidar data without computing point cloud
- Added profile support for Lidar Point Cloud creation
- IsaacSensorCreateRtxRadar command

### Changed
- changed node template name for rtx_lidar
- renamed ReadRTXLidar nodes to ComputeRTXLidar
- nvlidar dep to nvsensor and updated version.

## [3.0.1] - 2022-11-14

### Fixed
- Removed extra copy of BaseResetNode and use the one from core_nodes

## [3.0.0] - 2022-11-01

### Added
- IsaacRenderVarToCpuPointer node to replace rtx_lidar need for SdRenderVarToRawArray

### Removed
- ReadRTXRaw node and moved pointer pass through functionality to IsaacRenderVarToCpuPointer

### Changed
- inputs to ReadRTXLidar[PointCloud|FlatScan] nodes to use IsaacRenderVarToCpuPointer cpuPointer

## [2.1.0] - 2022-11-01

### Added
- ReadRTXRaw node
- PrintRTXLidarInfo node

## [2.0.0] - 2022-10-19

### Changed
- Extension name to omni.isaac.sensor

## [1.6.2] - 2022-10-19

### Changed
- ReadRTXLidarPointCloud code doc and ignore 0 values.

### Fixed
- accuracy error calculation in ReadRTXLidarPointCloud 

## [1.6.1] - 2022-10-18

### Added
- ReadRTXLidarPointCloud has transform lidarToWorld output
- ReadRTXLidarPointCloud has output on demand for all possible attributes

### Changed
- ReadRTXLidarPointCloud outputs in lidar coords

## [1.6.0] - 2022-10-09

### Added
- IsaacRtxLidarSensorAPI applied schema to differential regular cameras from RTX lidar cameras

## [1.5.1] - 2022-10-07

### Changed
- Changed the backend contact api to use updated batched update instead of notifications

## [1.5.0] - 2022-10-06

### Added
- keepOnlyPositiveDistance flag to ReadRTXLidarPointCloud Node
- intensity output to ReadRTXLidarPointCloud Node
- accuracy error post process to ReadRTXLidarPointCloud Node
- synthetic data template for DebugDrawPointCloud

### Fixed
- positions of points in ReadRTXLidarPointCloud 


## [1.4.0] - 2022-09-28

### Added
- ReadRTXLidarFlatScan Node

## [1.3.0] - 2022-09-27
### Changed
- tests to use nucleus assets
### Removed
- usd files local to extension

## [1.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.2.0] - 2022-09-02

### Changed
- Remove RTX tests from windows
- Disable failing contact sensor tests from windows
- Cleanup contact sensor tests
- Use xform utilities instead of XformPrim for commands

## [1.1.1] - 2022-09-01

### Changed
- Remove legacy viewport calls from tests

## [1.1.0] - 2022-08-24

### Added
- Lidar Config file location as data/lidar_configs

## [1.0.2] - 2022-08-09

### Changed 
- Removed simple_articulation.usd, test_imu_sensor uses Nucleus asset

## [1.0.1] - 2022-07-29

### Changed
- Added an exec out on the ReadContact and ReadIMU nodes
### Fixed
- Removed extra print statement

## [1.0.0] - 2022-07-22

### Added
- ReadRTXLidarPointCloud Node

### Changed
- IsaacSensorCreateContactSensor, renamed offset to translation to be consistent with core
- IsaacSensorCreateImuSensor, renamed offset to translation to be consistent with core
- Use XformPrim to initialize sensors for consistency with core
- Make return values for commands consistent, they now return: command_status, (success, prim)

## [0.5.1] - 2022-07-15

### Changed
- Renamed BindingsContactSensorPython to BindingsIsaacSensorPython 

## [0.5.0] - 2022-07-11

### Added
- Read contact sensor omnigraph node and tests 
- Orientation reading to Imu sensor sample 

### Changed
- Contact sensor resets on stop/start 

## [0.4.0] - 2022-06-24

### Added
- Absolute orientation output to Imu sensor + tests 
- Read Imu node

### Fixed 
- Imu mRawBuffer resets upon stop/start

## [0.3.4] - 2022-05-24

### Fixed
- Property orientation loading bug

## [0.3.3] - 2022-04-22

### Changed
- Moved sensor data aquisition function from tick to onPhysicsStep

## [0.3.2] - 2022-04-14

### Fixed
- Fixed component visualization

## [0.3.1] - 2022-04-07

### Changed
- Draw function runs onUpdate instead of physics call back

### Fixed
- Fixed visualization error of the isaac sensors

## [0.3.0] - 2022-04-04

### Added 
- Added Imu sensor

### Changed
- Extension name to omni.isaac.isaac_sensor
- Imu sensor getSensorReadings to output the readings from the last frame
- Updated index.rst documentation for contact sensor and imu sensors

## [0.2.1] - 2022-03-28

### Added
- Add UI element to create contact sensor 

### Changed
- Converted contact sensor namespaces to isaac sensor namespaces
- Modified draw function to use USD util's global pose

## [0.2.0] - 2022-03-18

### Changed
- Converted contact sensors into usdSchemas

### Fixed
- Enable visualization of contact sensors in the stage

## [0.1.3] - 2022-03-16

### Fixed
- Bugfix for failing tests and missing updates

## [0.1.2] - 2022-01-26

### Changed
- Compatibility for sdk 103

## [0.1.1] - 2021-07-26

### Added
- New UI

## [0.1.0] - 2021-07-08

### Added
- Initial version of Isaac Sim Contact Sensor Extension
