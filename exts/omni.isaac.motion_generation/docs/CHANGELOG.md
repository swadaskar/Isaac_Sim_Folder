# Changelog

## [4.5.6] - 2023-01-06
### Fixed
- Typo in variable name in ArticulationTrajectory.get_robot_articulation()

## [4.5.5] - 2022-12-12
### Changed
- Updates to API docs

## [4.5.4] - 2022-12-04
### Changed
- Small change to Cobotta RmpFlow configs for consistency with tutorials.

## [4.5.3] - 2022-12-01
### Changed
- Moved Cortex UR10 RMPflow config file and corresponding policy config to new directory (was only in legacy directory and unused).

## [4.5.2] - 2022-11-29
### Changed
- Updated old robot_description YAML files for Franka, UR10, DOFbot, and Cobotta to remove unecessary fields that had no effect.

## [4.5.1] - 2022-11-28
### Added
- Updated file paths to Nucleus assets in RmpFlow tests for Kawasaki, Flexiv, and Festo robots.

## [4.5.0] - 2022-11-28
### Added
- Added RmpFlow config and test for FestoCobot

## [4.4.0] - 2022-11-28
### Added
- Added RmpFlow configs and tests for Kawasaki and Flexiv robots

## [4.3.1] - 2022-11-22
### Added
- Cortex UR10 configs for UR10 bin supporting stacking demo

## [4.3.0] - 2022-11-22
### Changed
- Updated ArticulationSubset to handle sparse ArticulationActions. Previously, it None-padded the ArticulationAction.
- Some modifications to ArticulationSubset to simplify the error checking code and change member names.
- Updates ArticulationMotionPolicy to use the sparse API.
- Moved ArticulationSubset to omni.isaac.core

## [4.2.0] - 2022-11-18
### Added
- Added RmpFlow configs for universal robots

## [4.1.1] - 2022-11-18
### Fixed
- Fixed missing import statement for ArticulationTrajectory in MotionGeneration __init__

## [4.1.0] - 2022-11-17
### Added
- Added Trajectory interface, ArticulationTrajectory, and Lula Trajectory Generators

## [4.0.3] - 2022-11-10
### Changed
- Updated determinism settings to include omni.isaac.core World

## [4.0.2] - 2022-10-24

### Changed
- Moved Test cases using UR10 asset to use USD from Nucleus

## [4.0.1] - 2022-10-20

### Changed
- Moved Test cases using Franka asset to use USD from Nucleus

## [4.0.0] - 2022-10-17

### Changed
- Allow user to variable physics dt on each frame to an ArticulationMotionPolicy or set a default value.
- Change RmpFlow parameter 'evaluations_per_frame' to 'maximum_substep_size' to account for a possibly varying framerate

## [3.6.4] - 2022-10-06

### Changed
- Updated outdated Franka URDF with new joint limits on joint 7

## [3.6.3] - 2022-09-02

### Added
- Added function to get default rmpflow cspace target
- Added test case for setting rmpflow cspace target

## [3.6.2] - 2022-09-01

### Changed
- Remove legacy viewport calls from tests

## [3.6.1] - 2022-08-16

### Changed

- Updated RMPflow parameters in config YAML files for Denso robots: Turned on velocity_cap_rmp

## [3.6.0] - 2022-08-10

### Added

- Added Cobotta Pro 900 and Cobotta Pro 1300 as supported robots with provided RMPflow config files and test cases.

## [3.5.1] - 2022-08-03

### Fixed

- `ArticulationSubset.get_joint_subset_indices()` fixed (was returning function rather than return value of function call.)

## [3.5.0] - 2022-07-26

### Changed

- Changed gripper_controller argument to gripper in the PickPlaceController.
- moved PickPlaceController and StackingController to omni.isaac.manipulators

## [3.4.0] - 2022-07-20

### Added

- Added set_param() function to Lula RRT implementation.

### Changed

- Changed docstrings for PathPlannerVisualizer and Lula RRT implementation

### Fixed

- Fixed unreliable test case for lula RRT by reducing the RRT step size

## [3.3.1] - 2022-07-19

### Fixed

- Fixed bug in RmpFlow.set_cspace_target() which changed the end effector target when it shouldn't have
- Fixed bug in RmpFlow.get_internal_robot_joint_states() which resulted in a TypeError

## [3.3.0] - 2022-07-18

### Changed

- Updated ArticulationSubset to wait until robot joint states are queried to access the Articulation object.  This avoids annoying errors when attempting to initialize an ArticulationMotionPolicy before the "play" button has been pressed.

## [3.2.1] - 2022-06-28

### Changed

- Updated MotionPolicy to not assume a default orientation.  It now passes None to the MotionPolicy.

## [3.2.0] - 2022-06-17

### Added

- Added PathPlanningInterface with Lula RRT implementation and simple class for aiding visualization

## [3.1.2] - 2022-05-23

### Added

- Added conversion to numpy if articulation backend is GPU/torch

## [3.1.1] - 2022-05-18

### Added

- Added getter to get the MotionPolicy from a MotionPolicyController.

## [3.1.0] - 2022-05-09

### Changed

- Updated all hard coded USD object values to meters in motion_generation tests 

### Fixed

- Fixed bug in RmpFlow.create_ground_plane() related to unit conversion

## [3.0.1] - 2022-05-02

### Added

- Added some accessors to ArticulationMotionPolicy and ArticulationSubset.

## [3.0.0] - 2022-04-29

### Added

- Added Kinematics interface with a Lula implementation
- Added ArticulationKinematicsSolver wrapper for interfacing kinematics with USD robot

### Changed

- Replaced InverseKinematicsSolver(BaseController) object with ArticulationKinematicsSolver

## [2.0.0] - 2022-04-21

### Changed

- Renamed MotionGenerator to ArticulationMotionPolicy

### Added

- Created ArticulationSubset class to handle index mapping between Articulation and MotionPolicy

## [1.3.1] - 2022-04-27

### Added

- Added RmpFlowSmoothed to lula/motion_policies.py to support cortex.

## [1.3.0] - 2022-04-18

### Changed

- Extracted methods from MotionPolicy to form a WorldInterface class.  This has no functional effect on any code outside MotionGeneration

## [1.2.0] - 2022-04-15

### Changed

- Obstacles are now marked as static explicitly when added to MotionPolicy

## [1.1.0] - 2022-04-14

### Added

- Separated RmpFlow visualization functions for end effector and collision spheres
- Added test case for visualization
- Added Sdf.ChangeBlock() to visualization functions for efficiency

## [1.0.3] - 2022-04-13

### Changed

- Fixed typo in interface_config_loader.py.

## [1.0.2] - 2022-04-01

### Changed

- modified default RmpFlow configs have fewer updates per frame (10 was unnecessary) and to not ignore robot state updates by default
- updated golden values in tests as a direct result of config change

## [1.0.1] - 2022-04-01

### Added

- test case for motion_generation extension: test for proper behavior when add/enable/disable/remove objects to RmpFlow

### Fixed

- ground plane handling: enable/disable/remove ground_plane didn't work
- static obstacle handling: dictionary key error when enable/disable/remove static obstacles

## [1.0.0] - 2022-03-25

### Changed

- Restructured MotionGeneration extension to place emphasis on MotionPolicy over MotionGeneration.  The user is now expected to interact directly with a MotionPolicy for adding/editing obstacles, and setting targets.  MotionGeneration is a light utility class for interfacing the simulated USD robot to the MotionPolicy (get USD robot state and appropriately map the joint indeces).  
- RmpFlowController -> MotionPolicyController: 
    - The RmpFlowController wrapper that was used to interface Core examples with RmpFlow has been expanded to wrap any MotionPolicy
- omni.isaac.motion_generation/policy_configs -> omni.isaac.motion_generation/motion_policy_configs: changed folder containing config files for MotionPolicies to be named "motion_policy_configs" to leave room for future interfaces to have config directories
- Path to RmpFlow: omni.isaac.motion_generation.LulaMotionPolicies.RmpFlow -> omni.isaac.motion_generation.lula.motion_policies.RmpFlow

### Added

- interface_config_loader: a set of helper functions for checking what config files exist directly in the motion_generation extension and loading the configs as keyword arguments to the appropriate class e.g. RmpFlow(**loaded_config_dict)

## [0.2.1] - 2022-02-15

- Updated internal RMPflow implementation to allow for visualizing Lula collision spheres as prims on the stage

## [0.2.0] - 2022-02-10

### Changed

- Updated MotionGeneration to use Core API to query prim position and control the robot

## [0.1.5] - 2022-02-10

### Fixed

- Undefined joint in dofbot USD referenced by RMPflow config 

## [0.1.4] - 2022-01-20

### Added

- moved kinematics.py from omni.isaac.core.utils to this extension

## [0.1.3] - 2021-12-13

### Changed

- Removed deprecated fields from the Lula robot description files and RMPflow configuration files for the DOFBOT and Franka robots.  This also corrects an oversight in the Franka robot description file that had resulted in a lack of collision spheres (and thus obstacle avoidance) for panda_link6.

## [0.1.2] - 2021-12-02

### Changed

- event_velocities to events_dt in PickPlaceController
- Added new phase of wait in PickPlaceController

## [0.1.1] - 2021-08-04

### Added

- Added a simple wheel base pose controller.

## [0.1.0] - 2021-08-04

### Added

- Initial version of Isaac Sim Motion Generation Extension
