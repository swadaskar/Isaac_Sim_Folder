# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import carb
import omni
from typing import Optional, Tuple, List
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, UsdShade
from omni.isaac.core.utils.constants import AXES_INDICES
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_path, is_prim_path_valid
from omni.isaac.core.utils.carb import get_carb_setting, set_carb_setting
from omni.isaac.core.utils.stage import get_current_stage, get_stage_units, traverse_stage


class PhysicsContext(object):
    """Provides high level functions to deal with a physics scene and its settings. This will create a 
           a PhysicsScene prim at the specified prim path in case there is no PhysicsScene present in the current
           stage. 
           If there is a PhysicsScene present, it will discard the prim_path specified and sets the
           default settings on the current PhysicsScene found.

        Args:
            physics_dt (float, optional): specifies the physics_dt of the simulation. Defaults to 1.0 / 60.0.
            prim_path (Optional[str], optional): specifies the prim path to create a PhysicsScene at, 
                                                 only in the case where no PhysicsScene already defined. 
                                                 Defaults to "/physicsScene".
            set_defaults (bool, optional): set to True to use the defaults physics parameters
                                            [physics_dt = 1.0/ 60.0,
                                            gravity = -9.81 m / s
                                            ccd_enabled,
                                            stabilization_enabled,
                                            gpu dynamics turned off,
                                            broadcast type is MBP,
                                            solver type is TGS]. Defaults to True.
            backend (str, optional): specifies the backend to be used (numpy or torch). Defaults to numpy.
            device (Optional[str], optional): specifies the device to be used if running on the gpu with torch backend.

        Raises:
            Exception: If prim_path is not absolute.
            Exception: if prim_path already exists and its type is not a PhysicsScene.
        """

    def __init__(
        self,
        physics_dt: Optional[float] = None,
        prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
        backend: str = "numpy",
        device: Optional[str] = None,
    ) -> None:
        # TODO: add backend for physics
        self._prim_path = prim_path
        if not Sdf.Path(self._prim_path).IsAbsolutePath():
            raise Exception(f"Input prim path is not absolute: {self._path}")
        # check if there is a current physics scene defined already in the scene
        current_physics_prim = self.get_current_physics_scene_prim()
        self._physx_scene_api = None
        self._carb_settings = carb.settings.get_settings()
        if current_physics_prim is None:
            # creating a new physics scene
            if is_prim_path_valid(prim_path):
                raise Exception(f"A non physics scene prim already exists at: {self._prim_path}")
            self._physics_scene = self._create_new_physics_scene(prim_path=prim_path)
        else:
            # already exists a physics scene
            self._prim_path = get_prim_path(current_physics_prim)
            carb.log_info(f"Physics Scene at path `{self._prim_path}` is already defined - reusing it")
            self._physics_scene = UsdPhysics.Scene(current_physics_prim)
            self._physx_scene_api = PhysxSchema.PhysxSceneAPI(current_physics_prim)
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._physx_sim_interface = omni.physx.get_physx_simulation_interface()
        self._use_gpu_pipeline = False
        self._use_flatcache = False

        if sim_params is None and set_defaults:
            meters_per_unit = get_stage_units()
            self.set_gravity(value=-9.81 / meters_per_unit)
            self.enable_ccd(flag=True)
            self.enable_stablization(flag=True)
            if device is not None and "cuda" in device:
                # we are combining sim device and pipeline here
                self._carb_settings.set_bool("/physics/suppressReadback", True)
                self.set_broadphase_type("GPU")
                self.enable_gpu_dynamics(flag=True)
                self.enable_flatcache(True)
            else:
                self._carb_settings.set_bool("/physics/suppressReadback", False)
                self.set_broadphase_type(broadcast_type="MBP")
                self.enable_gpu_dynamics(flag=False)
            self.set_solver_type(solver_type="TGS")
            self.set_physics_dt(dt=1.0 / 60.0)

        if sim_params is not None:
            if "gravity" in sim_params.keys():
                up_axis = UsdGeom.GetStageUpAxis(get_current_stage())
                self.set_gravity(sim_params["gravity"][AXES_INDICES[up_axis]])

            if "substeps" in sim_params.keys():
                substeps = sim_params["substeps"]
            else:
                substeps = None
            if "dt" in sim_params.keys():
                self.set_physics_dt(dt=sim_params["dt"], substeps=substeps)
                get_current_stage().SetTimeCodesPerSecond(1 / sim_params["dt"])

            if "use_gpu_pipeline" in sim_params.keys():
                self._carb_settings.set_bool("/physics/suppressReadback", sim_params["use_gpu_pipeline"])
                if sim_params["use_gpu_pipeline"]:
                    self._use_gpu_pipeline = True

            if "use_flatcache" in sim_params.keys() and sim_params["use_flatcache"]:
                self._use_flatcache = True
                self.enable_flatcache(True)

            if "use_gpu" in sim_params.keys():
                self.enable_gpu_dynamics(sim_params["use_gpu"])
                if sim_params["use_gpu"]:
                    self.set_broadphase_type("GPU")
                else:
                    self.set_broadphase_type("MBP")

            if "enable_scene_query_support" in sim_params.keys():
                self.set_enable_scene_query_support(sim_params["enable_scene_query_support"])

            # GPU buffers
            if "gpu_max_rigid_contact_count" in sim_params.keys():
                self.set_gpu_max_rigid_contact_count(sim_params["gpu_max_rigid_contact_count"])
            if "gpu_max_rigid_patch_count" in sim_params.keys():
                self.set_gpu_max_rigid_patch_count(sim_params["gpu_max_rigid_patch_count"])
            if "gpu_found_lost_pairs_capacity" in sim_params.keys():
                self.set_gpu_found_lost_pairs_capacity(sim_params["gpu_found_lost_pairs_capacity"])
            if "gpu_found_lost_aggregate_pairs_capacity" in sim_params.keys():
                self.set_gpu_found_lost_aggregate_pairs_capacity(sim_params["gpu_found_lost_aggregate_pairs_capacity"])
            if "gpu_total_aggregate_pairs_capacity" in sim_params.keys():
                self.set_gpu_total_aggregate_pairs_capacity(sim_params["gpu_total_aggregate_pairs_capacity"])
            if "gpu_max_soft_body_contacts" in sim_params.keys():
                self.set_gpu_max_soft_body_contacts(sim_params["gpu_max_soft_body_contacts"])
            if "gpu_max_particle_contacts" in sim_params.keys():
                self.set_gpu_max_particle_contacts(sim_params["gpu_max_particle_contacts"])
            if "gpu_heap_capacity" in sim_params.keys():
                self.set_gpu_heap_capacity(sim_params["gpu_heap_capacity"])
            if "gpu_temp_buffer_capacity" in sim_params.keys():
                self.set_gpu_temp_buffer_capacity(sim_params["gpu_temp_buffer_capacity"])
            if "gpu_max_num_partitions" in sim_params.keys():
                self.set_gpu_max_num_partitions(sim_params["gpu_max_num_partitions"])
            if "solver_type" in sim_params.keys():
                # TODO: double check this with kelly
                if sim_params["solver_type"] == 0:
                    self.set_solver_type("PGS")
                else:
                    self.set_solver_type("TGS")
            if "enable_stabilization" in sim_params.keys():
                self.enable_stablization(sim_params["enable_stabilization"])
            if "bounce_threshold_velocity" in sim_params.keys():
                self.set_bounce_threshold(sim_params["bounce_threshold_velocity"])
            if "friction_offset_threshold" in sim_params.keys():
                self.set_friction_offset_threshold(sim_params["friction_offset_threshold"])
            if "friction_correlation_distance" in sim_params.keys():
                self.set_friction_correlation_distance(sim_params["friction_correlation_distance"])

            # create default physics material
            # TODO: double check this with kelly
            if "default_physics_material" in sim_params.keys():
                default_material_path = self._prim_path + "/defaultMaterial"
                default_material = UsdShade.Material.Define(get_current_stage(), default_material_path)
                mat = UsdPhysics.MaterialAPI.Apply(default_material.GetPrim())
                mat.CreateStaticFrictionAttr().Set(sim_params["default_physics_material"]["static_friction"])
                mat.CreateDynamicFrictionAttr().Set(sim_params["default_physics_material"]["dynamic_friction"])
                mat.CreateRestitutionAttr().Set(sim_params["default_physics_material"]["restitution"])
                # bind default physics material to scene
                material_api = UsdShade.MaterialBindingAPI.Apply(self._physics_scene.GetPrim())
                material_api.Bind(default_material, UsdShade.Tokens.weakerThanDescendants, "physics")

        if physics_dt is not None:
            self.set_physics_dt(dt=physics_dt)
        return

    @property
    def prim_path(self):
        return self._prim_path

    @property
    def use_gpu_pipeline(self):
        return self._use_gpu_pipeline

    @property
    def use_flatcache(self):
        return self._use_flatcache

    def __del__(self):
        return

    def get_current_physics_scene_prim(self) -> Optional[Usd.Prim]:
        """Used to return the PhysicsScene prim in stage by traversing the stage.

        Returns:
            Optional[Usd.Prim]: returns a PhysicsScene prim if found in current stage. Otherwise, None.
        """
        for prim in traverse_stage():
            if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                return prim
        return None

    def warm_start(self):
        self._physx_interface.start_simulation()
        self._physx_interface.force_load_physics_from_usd()
        self._physx_interface.update_simulation(self.get_physics_dt(), 0.0)
        # self._physx_sim_interface.simulate(self.get_physics_dt(), 0.0) # This causes a hang
        self._physx_sim_interface.fetch_results()

    def _create_new_physics_scene(self, prim_path: str):
        carb.log_info(f"Defining a new Physics Scene at path `{prim_path}`")
        stage = get_current_stage()
        scene = UsdPhysics.Scene.Define(stage, prim_path)
        self._physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(get_prim_at_path(prim_path))
        return scene

    def set_physics_dt(self, dt: float = 1.0 / 60.0, substeps: int = 1) -> None:
        """Sets the physics dt on the PhysicsScene

        Args:
            dt (float, optional): physics dt. Defaults to 1.0/60.0.
            substeps (int, optional): number of physics steps to run for before rendering a frame. Defaults to 1.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
            ValueError: Physics dt must be a >= 0.
            ValueError: Physics dt must be a <= 1.0.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if dt < 0:
            raise ValueError("physics dt cannot be <0")
        # if no stage or no change in physics timestep, exit.
        if get_current_stage() is None:
            return
        # if physics substeps is not valid, make default = 1.
        if substeps is None or substeps <= 1:
            substeps = 1
        if dt == 0:
            self._physx_scene_api.GetTimeStepsPerSecondAttr().Set(0)
            min_steps = 0
        elif dt > 1.0:
            raise ValueError("physics dt must be <= 1.0")
        else:
            steps_per_second = int(1.0 / dt)
            min_steps = int(steps_per_second / substeps)
            self._physx_scene_api.GetTimeStepsPerSecondAttr().Set(steps_per_second)

        set_carb_setting(carb.settings.get_settings(), "persistent/simulation/minFrameRate", min_steps)
        return

    def get_physics_dt(self) -> float:
        """Returns the current physics dt.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            float: physics dt.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        physics_hz = self._physx_scene_api.GetTimeStepsPerSecondAttr().Get()
        if physics_hz == 0:
            return 0.0
        else:
            return 1.0 / physics_hz

    def enable_flatcache(self, enable):
        manager = omni.kit.app.get_app().get_extension_manager()
        flatcache_was_enabled = manager.is_extension_enabled("omni.physx.flatcache")
        if not flatcache_was_enabled and enable:
            manager.set_extension_enabled_immediate("omni.physx.flatcache", True)
            self._carb_settings.set_bool("/physics/updateToUsd", False)
            self._carb_settings.set_bool("/physics/updateParticlesToUsd", False)
            self._carb_settings.set_bool("/physics/updateVelocitiesToUsd", False)
            self._carb_settings.set_bool("/physics/updateForceSensorsToUsd", False)
            self._carb_settings.set_bool("/physics/outputVelocitiesLocalSpace", False)
        elif flatcache_was_enabled and not enable:
            manager.set_extension_enabled_immediate("omni.physx.flatcache", False)
            self._carb_settings.set_bool("/physics/updateToUsd", True)
            self._carb_settings.set_bool("/physics/updateParticlesToUsd", True)
            self._carb_settings.set_bool("/physics/updateVelocitiesToUsd", True)
            self._carb_settings.set_bool("/physics/updateForceSensorsToUsd", True)
            self._carb_settings.set_bool("/physics/outputVelocitiesLocalSpace", True)

    def enable_ccd(self, flag: bool) -> None:
        """Enables a second broad phase after integration that makes it possible to prevent objects from tunneling
           through each other.

        Args:
            flag (bool): enables or diables ccd on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableCCDAttr().Get() is None:
            self._physx_scene_api.CreateEnableCCDAttr(flag)
        else:
            self._physx_scene_api.GetEnableCCDAttr().Set(flag)
        return

    def is_ccd_enabled(self) -> bool:
        """Checks if ccd is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if ccd is enabled, otherwise False.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableCCDAttr().Get()

    def enable_stablization(self, flag: bool) -> None:
        """Enables additional stabilization pass in the solver.

        Args:
            flag (bool): enables or diables stabilization on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableStabilizationAttr().Get() is None:
            self._physx_scene_api.CreateEnableStabilizationAttr(flag)
        else:
            self._physx_scene_api.GetEnableStabilizationAttr().Set(flag)
        return

    def is_stablization_enabled(self) -> bool:
        """Checks if stabilization is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if stabilization is enabled, otherwise False.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableStabilizationAttr().Get()

    def enable_gpu_dynamics(self, flag: bool) -> None:
        """Enables gpu dynamics pipeline, required for deformables for instance.

        Args:
            flag (bool): enables or diables gpu dynamics on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableGPUDynamicsAttr().Get() is None:
            self._physx_scene_api.CreateEnableGPUDynamicsAttr(flag)
        else:
            self._physx_scene_api.GetEnableGPUDynamicsAttr().Set(flag)
        return

    def is_gpu_dynamics_enabled(self) -> bool:
        """Checks if Gpu Dynamics is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if Gpu Dynamics is enabled, otherwise False.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableGPUDynamicsAttr().Get()

    def set_broadphase_type(self, broadcast_type: str) -> None:
        """Broadcast phase algorithm used in simulation.

        Args:
            broadcast_type (str): type of broadcasting to be used, can be "MBP"

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetBroadphaseTypeAttr().Get() is None:
            self._physx_scene_api.CreateBroadphaseTypeAttr(broadcast_type)
        else:
            self._physx_scene_api.GetBroadphaseTypeAttr().Set(broadcast_type)
        return

    def get_broadphase_type(self) -> str:
        """Gets current broadcast phase algorithm type.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            str: Broadcast phase algorithm used.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetBroadphaseTypeAttr().Get()

    def set_solver_type(self, solver_type: str) -> None:
        """solver used for simulation.

        Args:
            solver_type (str): can be "TGS" or "PGS". for references look at..

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetSolverTypeAttr().Get() is None:
            self._physx_scene_api.CreateSolverTypeAttr(solver_type)
        else:
            self._physx_scene_api.GetSolverTypeAttr().Set(solver_type)
        return

    def get_solver_type(self) -> str:
        """Gets current solver type.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            str: solver used for simulation.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetSolverTypeAttr().Get()

    def set_gravity(self, value: float) -> None:
        """sets the gravity direction and magnitude.

        Args:
            value (float): gravity value to be used in simulation.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if value < 0:
            z_dir = -1
            magnitude = value * -1
        else:
            z_dir = 1
            magnitude = value
        up_axis = UsdGeom.GetStageUpAxis(get_current_stage())
        gravity_dir = Gf.Vec3f(0.0)
        gravity_dir[AXES_INDICES[up_axis]] = z_dir
        if self._physics_scene.GetGravityDirectionAttr().Get() is None:
            self._physics_scene.CreateGravityDirectionAttr(gravity_dir)
        else:
            self._physics_scene.GetGravityDirectionAttr().Set(gravity_dir)

        if self._physics_scene.GetGravityMagnitudeAttr().Get() is None:
            self._physics_scene.CreateGravityMagnitudeAttr(magnitude)
        else:
            self._physics_scene.GetGravityMagnitudeAttr().Set(magnitude)
        return

    def get_gravity(self) -> Tuple[List, float]:
        """Gets current gravity.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Tuple[list, float]: returns a tuple, first element corresponds to the gravity direction vector and second element is the magnitude.
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return (
            list(self._physics_scene.GetGravityDirectionAttr().Get()),
            self._physics_scene.GetGravityMagnitudeAttr().Get(),
        )

    def set_physx_update_transformations_settings(
        self,
        update_to_fast_cache: Optional[bool] = None,
        update_to_usd: Optional[bool] = None,
        update_velocities_to_usd: Optional[bool] = None,
        output_velocities_local_space: Optional[bool] = None,
    ) -> None:
        """Sets how physx syncs with the usd when transformations are updated.

        Args:
            update_to_fast_cache (bool, optional): Uses Fast cache if set to True. Defaults to True.
            update_to_usd (bool, optional): Updates to USD the transformations. Defaults to True.
            update_velocities_to_usd (bool, optional): Updates Velocities to USD. Defaults to True.
            output_velocities_local_space (bool, optional): Output the velocities in the local frame and not the world frame. Defaults to False.
        """
        if update_to_fast_cache is not None:
            set_carb_setting(self._carb_settings, "/persistent/physics/useFastCache", update_to_fast_cache)
        if update_to_usd is not None:
            set_carb_setting(self._carb_settings, "/persistent/physics/updateToUsd", update_to_usd)
        if update_velocities_to_usd is not None:
            set_carb_setting(self._carb_settings, "/persistent/physics/updateVelocitiesToUsd", update_velocities_to_usd)
        if output_velocities_local_space is not None:
            set_carb_setting(
                self._carb_settings, "/persistent/physics/outputVelocitiesLocalSpace", output_velocities_local_space
            )
        return

    def get_physx_update_transformations_settings(self) -> Tuple[bool, bool, bool, bool]:
        """Gets how physx syncs with the usd when transformations are updated.

        Returns:
            Tuple[bool, bool, bool, bool]: [update_to_fast_cache, update_to_usd, update_velocities_to_usd, output_velocities_local_space]
        """
        return (
            get_carb_setting(self._carb_settings, "/persistent/physics/useFastCache"),
            get_carb_setting(self._carb_settings, "/persistent/physics/updateToUsd"),
            get_carb_setting(self._carb_settings, "/persistent/physics/updateVelocitiesToUsd"),
            get_carb_setting(self._carb_settings, "/persistent/physics/outputVelocitiesLocalSpace"),
        )

    def _step(self, current_time: float) -> None:
        self._physx_interface.update_simulation(elapsedStep=self.get_physics_dt(), currentTime=current_time)
        self._physx_interface.update_transformations(
            updateToFastCache=get_carb_setting(self._carb_settings, "/persistent/physics/useFastCache"),
            updateToUsd=get_carb_setting(self._carb_settings, "/persistent/physics/updateToUsd"),
            updateVelocitiesToUsd=get_carb_setting(self._carb_settings, "/persistent/physics/updateVelocitiesToUsd"),
            outputVelocitiesLocalSpace=get_carb_setting(
                self._carb_settings, "/persistent/physics/outputVelocitiesLocalSpace"
            ),
        )
        return

    def set_invert_collision_group_filter(self, invert_collision_group_filter: bool) -> None:
        """[summary]

        Args:
            invert_collision_group_filter (bool): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Get() is None:
            self._physx_scene_api.CreateInvertCollisionGroupFilterAttr(invert_collision_group_filter)
        else:
            self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Set(invert_collision_group_filter)
        return

    def get_invert_collision_group_filter(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Get()

    def set_bounce_threshold(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetBounceThresholdAttr().Get() is None:
            self._physx_scene_api.CreateBounceThresholdAttr(value)
        else:
            self._physx_scene_api.GetBounceThresholdAttr().Set(value)
        return

    def get_bounce_threshold(self) -> float:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            float: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetBounceThresholdAttr().Get()

    def set_friction_offset_threshold(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetFrictionOffsetThresholdAttr().Get() is None:
            self._physx_scene_api.CreateFrictionOffsetThresholdAttr(value)
        else:
            self._physx_scene_api.GetFrictionOffsetThresholdAttr().Set(value)
        return

    def get_friction_offset_threshold(self) -> float:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            float: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetFrictionOffsetThresholdAttr().Get()

    def set_friction_correlation_distance(self, value: float) -> None:
        """[summary]

        Args:
            value (float): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Get() is None:
            self._physx_scene_api.CreateFrictionCorrelationDistanceAttr(value)
        else:
            self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Set(value)
        return

    def get_friction_correlation_distance(self) -> float:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            float: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Get()

    def set_enable_scene_query_support(self, enable_scene_query_support: bool) -> None:
        """ Sets the Enable Scene Query Support attribute in Physx Scene

        Args:
            enable_scene_query_support (bool): Whether to enable scene query support

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableSceneQuerySupportAttr().Get() is None:
            self._physx_scene_api.CreateEnableSceneQuerySupportAttr(enable_scene_query_support)
        else:
            self._physx_scene_api.GetEnableSceneQuerySupportAttr().Set(enable_scene_query_support)
        return

    def get_enable_scene_query_support(self) -> bool:
        """ Retrieves the Enable Scene Query Support attribute in Physx Scene

        Raises:
            Exception: [description]

        Returns:
            bool: enable scene query support attribute
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableSceneQuerySupportAttr().Get()

    def set_gpu_max_rigid_contact_count(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxRigidContactCountAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Set(value)
        return

    def get_gpu_max_rigid_contact_count(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Get()

    def set_gpu_max_rigid_patch_count(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxRigidPatchCountAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Set(value)
        return

    def get_gpu_max_rigid_patch_count(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Get()

    def set_gpu_found_lost_pairs_capacity(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuFoundLostPairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Set(value)
        return

    def get_gpu_found_lost_pairs_capacity(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Get()

    def set_gpu_found_lost_aggregate_pairs_capacity(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuFoundLostAggregatePairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Set(value)
        return

    def get_gpu_found_lost_aggregate_pairs_capacity(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Get()

    def set_gpu_total_aggregate_pairs_capacity(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuTotalAggregatePairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Set(value)
        return

    def get_gpu_total_aggregate_pairs_capacity(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Get()

    def set_gpu_max_soft_body_contacts(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxSoftBodyContactsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxSoftBodyContactsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxSoftBodyContactsAttr().Set(value)
        return

    def get_gpu_max_soft_body_contacts(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxSoftBodyContactsAttr().Get()

    def set_gpu_max_particle_contacts(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxParticleContactsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxParticleContactsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxParticleContactsAttr().Set(value)
        return

    def get_gpu_max_particle_contacts(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxParticleContactsAttr().Get()

    def set_gpu_heap_capacity(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuHeapCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuHeapCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuHeapCapacityAttr().Set(value)
        return

    def get_gpu_heap_capacity(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuHeapCapacityAttr().Get()

    def set_gpu_temp_buffer_capacity(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuTempBufferCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuTempBufferCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuTempBufferCapacityAttr().Set(value)
        return

    def get_gpu_temp_buffer_capacity(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuTempBufferCapacityAttr().Get()

    def set_gpu_max_num_partitions(self, value: int) -> None:
        """[summary]

        Args:
            value (int): [description]

        Raises:
            Exception: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxNumPartitionsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxNumPartitionsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxNumPartitionsAttr().Set(value)
        return

    def get_gpu_max_num_partitions(self) -> int:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            int: [description]
        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.gpu_max_num_partitions().Get()
