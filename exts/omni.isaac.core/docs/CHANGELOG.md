# Changelog

## [1.46.3] - 2023-03-01
### Fixed
- Added joint efforts to the get_applied_action method in ArticulationView and Articulation (was returning None previously)

## [1.46.2] - 2023-02-14
### Fixed
- Simulation context should only subscribe to the type of stage event it needs


## [1.46.1] - 2023-02-09
### Added
- Added unit test for clear_instance() causing SimulationContext to self-delete on construction

## [1.46.0] - 2023-02-08
### Added
- texture_translate to omni_pbr.py

## [1.45.2] - 2023-02-05
### Fixed
- several unit test errors/warnings

## [1.45.1] - 2023-02-03
### Fixed
- fixed minor issues in cloth prims 

## [1.45.0] - 2023-01-31
### Added
- destroy_all_viewports util
## [1.44.0] - 2023-01-23
### Fixed
- Warnings about no hydra render context when running tests
- Warning about SimulationContext/World needing re-init after a new stage should only happen if they were previously initialized. 


## [1.44.0] - 2023-01-17
### Changed
- Make omni.kit.viewport.window and omni.kit.viewport.utility optional dependencies

## [1.43.0] - 2023-01-06
### Changed
- Update asset paths to 2022.2.1

## [1.42.0] - 2022-12-12
### Changed
- Update asset paths to 2022.2.0

## [1.41.1] - 2022-12-10
### Fixed
- refactored cloth API for consistency with other core prims 

## [1.41.0] - 2022-12-10
### Added
- Added set_targets to prim utils 

## [1.40.4] - 2022-12-09
### Fixed
- Data Logger mutating the data frames variable when saving is called.
- Simulation Context/ World used to revert the frame rate to the cached value before initializing SimulationContext/ World when a new stage is created. In kit 104 the setting is reset automatically when a new stage is loaded. So the explicit  caching and reverting to the original frame rate is removed from Simulation Context/ World.

## [1.40.3] - 2022-12-08
### Fixed
- deprecated get_joint_efforts in favor of get_applied_joint_efforts 

## [1.40.2] - 2022-12-07
### Fixed
- Fixes getter/ setter for sleep threshold 

## [1.40.1] - 2022-12-05
### Fixed
- RigidContactView for GeometryPrim/GeometryPrimView
- get_joint_efforts in Articulation is now consistent with that of ArticulationView class
- API doc fixes 

## [1.40.0] - 2022-12-02
### Added
- Sleep threshold setter and getter in RigidPrimView

## [1.39.0] - 2022-11-30
### Added
- Particle cloth simulation support through physics.tensors extension

## [1.38.0] - 2022-11-29
### Added
- Added articulationView APIs to get the commanded and computed joint efforts

## [1.37.1] - 2022-11-29

### Fixed

- unit test for get_body_index function

## [1.37.0] - 2022-11-23

### Added

- get_body_index to return queried body index

## [1.36.1] - 2022-11-23
### Fixed
- API fixes and unit test improvements

## [1.36.0] - 2022-11-22
### Added
- Moved ArticulationSubset here from `omni.isaac.motion_generation`
- Updated ArticulationSubset to handle sparse ArticulationActions. Previously, it None-padded the ArticulationAction.
- Some modifications to ArticulationSubset to simplify the error checking code and change member names.
- Updates ArticulationMotionPolicy to use the sparse API.


## [1.35.0] - 2022-11-15
### Added
- Base Sensor Class

## [1.34.0] - 2022-11-15
### Added
- Backend utils in torch and numpy: pas, stack, matmul, sin, cos, inverse and transpose_2d 

## [1.33.0] - 2022-11-18
### Changed
- moved util functions from core/utils/pose_generation.py to coreutils /transformations.py, /mesh.py, /random.py
- updated ycb_video_writer.py get_mesh_vertices_relative_to utils import to mesh.py

### Added
- mesh.py and random.py


## [1.32.2] - 2022-11-18
### Fixed
- Fixed set_local_poses when indices are provided

## [1.32.1] - 2022-11-16
### Fixed
- Fixed device for ArticulationView max_efforts


## [1.32.0] - 2022-11-09
### Added
- omni.isaac.core.utils.render_product 

## [1.31.1] - 2022-11-03
### Fixed
- Fixed order of matrix multiplications for variable source_to_target_column_major_tf in function get_relative_transform() in utils/pose_generation.py

## [1.31.0] - 2022-10-31
### Added
- Added RigidContactView class and APIs within RigidPrimView to retrieve net contact forces and contact forces from a subset of prims (OM-64746)

## [1.30.2] - 2022-10-25
### Changed
- Set persistent.isaac.asset_root.nvidia to main NVIDIA asset path (OM-64173)

## [1.30.1] - 2022-10-24
### Changed
- Changed default values of shape sizes to be 1.0 instead of 0.05

## [1.30.0] - 2022-10-17
### Added 
- allow applying rigid body forces in local coordinates and also at a position

## [1.29.0] - 2022-10-05
### Added 
- moved standalone pose estimation example utils to core.utils

## [1.28.2] - 2022-10-15
### Fixed 
- bug in sphere.py and cylinder.py where incorrect prim type was used in IsA check

## [1.28.1] - 2022-10-03
### Changed 
- Cuboids default size parameter from 0.05 to 1.

## [1.28.0] - 2022-09-29
### Changed 
- Allow manual dt to be set if loop runner is available outside of SimulationApp

## [1.27.1] - 2022-09-28
### Changed 
- Use blocking update_simulation call in warm_start

### Fixed
- disable_rigid_body_physics

## [1.27.0] - 2022-09-12
### Added 
- get_id_from_index to convert a legacy viewport id index into a proper viewport id

## [1.26.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.26.0] - 2022-09-02
### Changed
- reset_xform_ops now resets to isaac sim defaults
### Added
- clear_xform_ops, reset_and_set_xform_ops
- set_prim_hide_in_stage_window, set_prim_no_delete
- add_aov_to_viewport
## [1.25.0] - 2022-08-31

### Changed

- removed unused velocity argument from set_camera_view
- removed default arguments from set_camera_view to make it more general 
- switch to omni.kit.viewport.utility instead of viewport legacy

### Added

- viewport helper functions: get_viewport_names and get_window_from_id

## [1.24.4] - 2022-08-31

### Changed

- Update paths to 2022.2

### Added

- get_window_from_id to viewport.py

## [1.24.3] - 2022-08-17

### Fixed

- fixes `set_max_efforts` function: device must be on cpu

## [1.24.2] - 2022-08-17

### Fixed

- Reshape jacobian shape to match with shape of jacobian tensor.

## [1.24.1] - 2022-08-15

### Fixed

- Articulation Controller bugfix: `get_applied_action` was indexing joint_positions even if simulation is not running.

## [1.24.0] - 2022-08-14

### Added

- get_semantics to return all semantic APIs applied onto a prim


## [1.23.3] - 2022-08-09

### Fixed

- Articulation bugfix: `get_max_efforts` was always returning the `max_efforts` from PhysX instead of the joint-indices result when `clone=True`.

## [1.23.2] - 2022-08-09

### Fixed

- Articulation bugfix: `get_linear_velocity`, `get_angular_velocity` and `get_joint_velocities` was calling view's method twice once with indices then once without.  The second time should be selecting the single element of the batch array from the `result` rather than calling the method again.

## [1.23.1] - 2022-08-03

### Fixed

- Articulation bugfix: `get_joint_positions` was calling view's method twice once with indices then once without.  The second time should be selecting the single element of the batch array from the `result` rather than calling the method again.

## [1.23.0] - 2022-07-26

### Added
- Added joint_indices to the different get_joint_* methods in the Articulation.

## [1.23.0] - 2022-07-26

### Added
- Increase hang detection timeout (OM-55578)

## [1.22.1] - 2022-07-25

### Added
- Added setting gravity from sim config in physics context.

## [1.22.0] - 2022-07-21

### Added
- Added reset_xform_properties parameter to view classes for efficiency when the objects already have the right set of xform properties.
## [1.21.0] - 2022-07-21

### Added
- Added new APIs for ArticulationView and RigidPrimView


## [1.20.0] - 2022-07-17

### Changed
- single prim classes inheritance structure to avoid duplication of code
## [1.19.0] - 2022-07-16
### Added
- added get_first_matching_parent_prim, is_prim_non_root_articulation_link to prim utils

### Changed
- get_all_matching_child_prims to return a list of prims instead of a list of prim_paths
- get_first_matching_child_prim returns a prim instead of a prim path

## [1.18.0] - 2022-06-23
### Changed
- statistics.py moved to omni.isaac.statistics_logging extension

## [1.17.0] - 2022-06-22

### Changed
- Size to be a float for Cuboid instead of 3 dimensional (scale to be used instead for consistency with USD)

## [1.16.0] - 2022-06-16

### Changed
- save_stage allows in place saving without reloading stage. 

## [1.15.2] - 2022-06-13

### Added
- Parse GPU device ID from carb settings /physics/cudaDevice.

### Fixed
- Fixed GPU buffer attribute mismatch in physics context config parsing.

## [1.15.1] - 2022-06-02

### Fixed
- handles_initialized in Articulation class

## [1.15.0] - 2022-05-30

### Changed
- move and rename persistent.isaac.asset_root.cloud from assets_check extension

## [1.14.0] - 2022-05-26

### Changed
- Replaced .check on physics views with an event callback for efficiency. 
- Adds checking for prim/prms in remove-object

## [1.13.2] - 2022-05-26

### Added
- Added APIs to get/set Enable Scene Query Support attribute

## [1.13.1] - 2022-05-25

### Changed
- Renamed copyAssetsURL to cloudAssetsURL.

## [1.13.0] - 2022-05-24

### Fixed
- Setting pd gains in the gpu pipeline.

## [1.12.0] - 2022-05-17

### Fixed
- Object classes to use RigidViews if initialized
- Bug is set_local_poses in RigidPrimView and ArticulationView

### Added
- Physics Handles check to avoid calling tensor api when the view is not valid.
- Added persistent.isaac.asset_root.default
- Added get_full_asset_path()

## [1.11.0] - 2022-05-16

### Fixed
- Object classes to wrap existing prims without changing its properties
- Setting gains to persist across resets

### Changed
- Passing physics materials instead of physics material path along with its properties.

## [1.10.0] - 2022-05-12

### Added
- initialize_physics function in World and SimulationContext

### Fixed
- GPU warmup

## [1.9.0] - 2022-05-12

### Changed
- Use omni.isaac.version.get_version()

### Added
- Added persistent.isaac.asset_root.nvidia and persistent.isaac.asset_root.isaac setting
- Added get_nvidia_asset_root_path() and get_isaac_asset_root_path()
- Added get_url_root() and verify_asset_root_path()

### Removed
- Removed persistent.isaac.nucleus.default setting
- Removed find_nucleus_server() and find_nucleus_server_async()

## [1.8.0] - 2022-05-04

### Changed
- Removing redundant api in ArticulationView and RigidPrimView
- Raise Exceptions when using set_linear_velocities and set_angular_velocities with the gpu pipleine

## [1.7.0] - 2022-05-04

### Changed
- RigidPrim class using RigidPrimView, GeometryPrim uses GeometryPrimView and Articulation uses ArticulationView class

## [1.6.9] - 2022-05-05

### Added
- Disable GPU usage warnings from tensor APIs in Core APIs
- articulation: added an accessor for getting the default state. (previously you could only set it)

## [1.6.8] - 2022-05-05

### Changed
- Added the option to enable flatcache in physics_context
- Disabled updateToUsd in physics_context when flatcache is enabled to allow faster load time 

## [1.6.7] - 2022-05-03

### Changed
- Reorganized the functions in World and SimulationContext to make them clearer to understand

### Added
- Added reset(), reset_async(), clear() methods to SimulationContext

### Fixed
- Reset in World was always resetting the physics sim view

## [1.6.6] - 2022-05-02

### Changed
- Update DOF path parsing in ArticulationView to use tensor API directly
- Use tensor APIs when available for DOF properties

## [1.6.5] - 2022-04-28

### Added
- API to enable/disable omni.physx.flatcache extension in PhysicsContext
- API to track whether GPU pipeline is enabled

### Fixed
- issue with getting next stage free path slash parsing

## [1.6.4] - 2022-04-27

### Added
- density in rigid prim view

### Removed
- Sim start in XFormPrim view and ArticulationView doesn't create a dummy physics view anymore

## [1.6.3] - 2022-04-27

### Fixed
- missing args for `convert()` method

## [1.6.2] - 2022-04-26

### Fixed
- Fixed create_prim method to support sequence data type
- Fixed prim interfaces to use sequence data type for setters and getters for pose and velocities
- Added method `convert()` to backend utils to convert into respective object container

## [1.6.1] - 2022-04-21

### Added
- Added checks for setters/getters of Geometry prim in the case collision is disabled

### Changed
- replaced find_nucleus_server() with get_assets_root_path()
- Adapts the hierarchy of classes in object prims: The inheritance is as follows:
    - Visual<Obj>(GeometryPrim): collision is disabled
    - Fixed<Obj>(Visual<Obj>): collision is enabled
    - Dynamic<Obj>(Fixed<Obj>, RigidPrim): collision is enabled and rigid body API applied (which enables the influence of external forces)

### Fixed
- Fixed issue with specifying a USD path for a view regex

## [1.6.0] - 2022-04-18

### Fixed
- Fixed assets version file check.
- acceleration spelling mistake in articulation_controller

## [1.5.2] - 2022-04-18

### Fixed
- cleaned up imports and comments in the utils

## [1.5.1] - 2022-04-15

### Fixed
- fixing visibility on XFormPrimView
- docstring issues

## [1.5.0] - 2022-04-14

### Added
- An argument to clear scene registery only
- rotation and cross product util functions

### Fixed
- Deleting a reference always when trying to delete a prim under the ref
- Physics start on construction of XFormPrim to be able to use dc interface to query if its under an articulation.
- XFormPrimView: fixed setting translation on init

## [1.4.0] - 2022-04-13

### Changed
- world.py: add step_sim param to step() paralleling the render flag

## [1.3.0] - 2022-04-10

### Changed
- XFormPrim class to use XFormPrimView class internally
- Changed default value of visibility in the XFormPrim class

### Added
- added rotation conversion functions to and from quaternions

## [1.2.0] - 2022-04-08

### Added
- Added implementations of set_gains, set_max_efforts, set_effort_modes, switch_control_modes and the their getters in ArticulationView.
- Forced physics to start on init of ArticulationView to initialize the num_dofs and other variables.
- Added unit tests for ArticulationView.
- Added initial docstrings for the added functions.

## [1.1.0] - 2022-04-05

### Added
- added pose_from_tf_matrix() to omni.isaac.core.utils.transformations

## [1.0.0] - 2022-03-31

### Added
- First version of Tensor API integration

## [0.3.2] - 2022-03-17

### Fixed
- converting gains from dc to usd units when saving to usd

## [0.3.1] - 2022-03-16

### Changed
- replaced find_nucleus_server() with get_assets_root_path()

### Added
- added get_assets_server()

## [0.3.0] - 2022-02-23

### Added
- set gains in usd option is added to the articulation controller

## [0.2.9] - 2022-02-14

### Added
- get/set_rigid_body_enabled to omni.isaac.core.utils.physics
- default predicate to omni.isaac.core.utils.prims.get_all_matching_child_prims
- test_prims to omni.isaac.core.tests

### Changed
- _list and _recursive_walk in omni.isaac.core.utils.nucleus to list_folder and recursive_list_folder

## [0.2.8] - 2022-02-14

### Fixed
- Fix setting of local pose in XFormPrim constructor

## [0.2.7] - 2022-02-13

### Fixed
- Use a SDF Change block when deleting prims
- Do not delete /Render/Vars prim when clearing stage

### Added
- is_prim_hidden_in_stage

## [0.2.6] - 2022-02-10

### Fixed
- GeometryPrim was not setting the collision approximation type correctly

## [0.2.5] - 2022-02-04

### Changed
- isaac.nucleus.default is now a persistent carb setting

## [0.2.4] - 2022-02-02

### Added
- dof_names property to Articulation

## [0.2.3] - 2022-01-26

### Added
- enable/disable rigid_body_physics for RigidPrims
- enable_gravity() for Articulation

### Fixed
- disable_gravity() for Articulation was enabling gravity

## [0.2.2] - 2022-01-21

### Added
- remove_all_semantics util function
- add set_intrinsics_matrix function

### Changed
- get_intrinsics_matrix uses vertical_aperture set on camera prim
- set_camera_view can take a user specified camera path

## [0.2.1] - 2022-01-20

### Changed
- kinematics.py to omni.isaac.motion_generation extension

## [0.2.0] - 2022-01-11

### Changed
- physx and usd tranformations update parameters are read from carb

### Added
- Added set_defaults to SimulationContext, World and PhysicsContext

## [0.1.12] - 2021-12-16

### Changed
- Added feature to detect and update downloaded Isaac Sim assets on Nucleus (OM-41819)

## [0.1.11] - 2021-12-08

### Changed
- recompute_extents now takes an argument to include children in recomputation

## [0.1.10] - 2021-12-01

### Added
- isaac.nucleus.default setting moved from omni.isaac.utils

### Fixed
- XformPrim now checks if orient is in single or double precision before setting

### Changed
- gf_quatf_to_np_array and gf_quatd_to_np_array to gf_quat_to_np_array
- control_index to time_step_index in pre_step in BaseTask

## [0.1.9] - 2021-11-29

### Added
- get_memory_stats to return a dictionary with memory usage statistics

## [0.1.8] - 2021-11-08

### Added
- Propagated scale and visible args to different objects inheriting from XFormPrim object.
- is_simulating to SimulationContext

### Changed
- Split add_ground_plane to add_default_ground_plane and add_ground_plane.
- Changed initialize_handles to initialize.

## [0.1.7] - 2021-11-06

### Added
- Added seperate functions to set different physics scene settings.
- Added error handling in PhysicsContext.
- Added doc strings to PhysicsContext.
- create_bbox_cache, compute_aabb, compute_combined_aabb

### Changed
- moving offset logic to base task to move the task assets accordingly
- changed name of PhysicsScene to PhysicsContext
- renamed mesh.py to bounds.py

## [0.1.6] - 2021-11-04

### Added
- OmniPBR visual material
- Added get_physics_scene in SimulationContext
- clear in world
- get_extension_path_from_name
- is_prim_no_delete

### Changed
- default visual materials and physics materials prim paths
- prim_type is default to Xform for create_prim
- type changed to prim_type for add_reference_to_stage
- get_prim_at_descendent_path -> get_first_matching_child_prim
- get_prims_path_at_descendent_tree -> get_all_matching_child_prims
- check_ancestral -> is_prim_ancestral

### Fixed
- default pose was resetting using local pose
- local pose in Rigid Body was missing an argument
- clear in Scene

### Removed
- set_extension_enabled

## [0.1.5] - 2021-11-01

### Added
- Renamed Cube objects to Cuboids
- Generalized Cubes to Cuboids instead
- Added support for OmniGlass get_applied_visual material
- Changed parent_prim_path to prim_path in OmniGlass
- Unit tests for time stepping
- Switched visual_material_path to visual_material when passed to the different objects

## [0.1.4] - 2021-10-29

### Added
- FixedCuboid class
- Calculate_metrics, is_done and async step functions to World
- More stepping examples to time_stepping.py
- Unit tests for time stepping

### Changed
- Made BaseTask methods non abstract
- Renamed set_physics_dt to set_simulation_dt for simulation context
- physics_dt and rendering_dt separated
- editor_callback change to render_callback
- Moved PhysicsScene to physics_scene.py

## [0.1.3] - 2021-10-21

### Changed
- Renamed view_ports.py to viewports.py
- renamed nucleus_utils.py to nucleus.py

### Added
- disable_extension omni.isaac.core.utils.extensions
- lookat_to_quat to omni.isaac.core.utils.rotations
- get_intrinsics_matrix, backproject_depth, project_depth_to_worldspace to omni.isaac.core.utils.viewports
- set_up_z_axis to omni.isaac.core.utils.stage.set_stage_up_axis

## [0.1.2] - 2021-10-20

### Added
- Added articulation gripper class
- Deleted PD in the namings under articulation controller
- Added base tasks for stacking, pick_place and follow_target
- Added an IK solver under utils

## [0.1.1] - 2021-10-18

### Added
- Added *_callback_exists methods under SimulationContext
- Added object_exists method under Scene

### Changed
- The behavior of .play() method under SimulationContext to always do 1 step with rendering for dc to function properly.

## [0.1.0] - 2021-10-15

### Added
- Added first version of core.
