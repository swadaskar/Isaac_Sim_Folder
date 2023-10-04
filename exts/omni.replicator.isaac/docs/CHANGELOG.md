# Changelog

## [1.7.4] - 2023-01-10
### Fixed
- Fix OgnDope node to return proper value for occlusion 

## [1.7.3] - 2022-12-14
### Fixed
- Fix OgnDope node to have proper return type for return_data_dtype_bbox_3d

## [1.7.2] - 2022-12-13
### Changed
- PytorchWriter annotator changed to cuda version of LdrColor annotator

## [1.7.1] - 2022-12-08
### Changed
- PytorchWriter registered on creation

## [1.7.0] - 2022-11-17
### Added
- Random3f node

## [1.6.1] - 2022-11-16
### Fixed
- Fixed device for ArticulationView max_efforts

## [1.6.0] - 2022-11-11
### Changed
- Allow run-time mass randomization with GPU pipeline

## [1.5.2] - 2022-10-25
### Fixed
- Fixed OgnPose.py node to read height and width from imageWidth and imageHeight 

## [1.5.1] - 2022-10-20
### Fixed
- Change DOPE writer to write to folder within s3 bucket instead of root dir

## [1.5.0] - 2022-10-19
### Changed
- Change DOPE writer to use BackendDispatch from omni.replicator.core for s3 writes 

## [1.4.0] - 2022-10-14
### Added
- save_mesh_vertices to ycb_video writer

## [1.3.5] - 2022-09-10
### Fixed
- Add mising version field to pytorch writer
- Update render product paths for pytorch writer test
- cleanup extension.toml

## [1.3.4] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.3.3] - 2022-08-31
### Fixed
- Fixed register_rigid_prim_view for RigidPrimViews that are non root-link

## [1.3.2] - 2022-08-17
### Fixed
- YCB Video writer's pose node (OgnPose) writing out-of-frame poses

## [1.3.1] - 2022-08-12
### Added
- Added contact offset and rest offset properties for articulation view randomization

### Fixed
- Fixed rest offset typo in rigid prim view

## [1.3.0] - 2022-08-11
### Added
- Custom annotator node for YCBVideo writer 
- Custom annotator node for DOPE writer

## [1.2.5] - 2022-08-11
### Added
- Added YCB Video writer using OV Replicator API
- Added pose node (OgnPose)

## [1.2.4] - 2022-08-05
### Added
- Added documentation and testing for pytorch wrtier
## [1.2.3] - 2022-08-03
### Added
- Added documentation and example demo script
### Changed
- Changed articulation body mass and body intertia randomization to CPU pipeline only

## [1.2.2] - 2022-07-29
### Fixed
- Fixed an issue where distribution parameters in the write nodes are not updated when the distribution is modified

## [1.2.1] - 2022-07-29
### Added
- Added bucketing support for material properties to avoid exceeding 64k material limit

## [1.2.0] - 2022-07-27
### Added
- Added simulation context randomization such as gravity
### Changed
- Changed the behaviour of on_reset randomization such that on_interval modifies the values set at on_reset instead of initial values
### Fixed
- Fixed a bug regarding lower and upper dof limits where randomization would change initial values

## [1.1.2] - 2022-07-26
### Changed
- Changed articulation tendon properties write nodes to be sequential.
### Added
- Added articulation material properties randomization.

## [1.1.1] - 2022-07-25
### Fixed
- Fixed the tests by moving away from using omnigraph bundles and just doing static type resolution on the output of the distribution nodes 

## [1.1.0] - 2022-07-22
### Added
- Added mass, inertia, material properties, rest offset, and contact offset for rigid prim view randomization
- Added mass, inertia, and tendon properties for articulation view randomization
- Added additive and scaling operations for orientation randomization
- Added pytorch rgb writer with replicator API for isaac gym
- Added pytorch listener for provide direct access to batched pytorch tensors from gym simulations

## [1.0.1] - 2022-07-20
### Changed
- Changed rigid_body_view to rigid_prim_view

## [1.0.0] - 2022-07-07
### Added
- Added tensor API node that interface with omni.replicator.core for RL domain randomization