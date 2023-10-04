# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import carb
from omni.isaac.core.materials.physics_material import PhysicsMaterial
from omni.isaac.core.prims import geometry_prim_view
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.prims.rigid_contact_view import RigidContactView
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.core.prims.soft.cloth_prim import ClothPrim
from omni.isaac.core.prims.soft.cloth_prim_view import ClothPrimView
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.prims.soft.particle_system_view import ParticleSystemView
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.materials.particle_material_view import ParticleMaterialView
from omni.isaac.core.prims.geometry_prim_view import GeometryPrimView
from omni.isaac.core.scenes.scene_registry import SceneRegistry
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.robots.robot_view import RobotView
from omni.isaac.core.prims.base_sensor import BaseSensor
from omni.isaac.core.utils.prims import (
    get_prim_parent,
    get_prim_path,
    is_prim_root_path,
    is_prim_ancestral,
    get_prim_children,
    is_prim_path_valid,
)
from omni.isaac.core.utils.string import find_unique_string_name
import omni.usd.commands
from pxr import Usd, UsdGeom, Sdf
import numpy as np
import builtins
from omni.isaac.core.utils.stage import get_current_stage, update_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from typing import Optional, Tuple
import gc


class Scene(object):
    """This class provides functions to add objects of interest in the stage to retrieve their information and set their 
        reset default state in an easy way. For example: 
        - performing certain commands post_reset
        - getting bounding boxes of the objects 
        - Deleting the objects/ removing them from stage..etc.

        Checkout the required tutorials at 
        https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html
    """

    def __init__(self) -> None:
        self._scene_registry = SceneRegistry()
        self._enable_bounding_box_computations = False
        self._bbox_cache = None
        return

    @property
    def stage(self) -> Usd.Stage:
        """[summary]

        Returns:
            Usd.Stage: [description]
        """
        return get_current_stage()

    def add(self, obj: XFormPrim) -> XFormPrim:
        """[summary]

        Args:
            obj (XFormPrim): [description]

        Raises:
            Exception: [description]
            Exception: [description]

        Returns:
            XFormPrim: [description]
        """
        if self._scene_registry.name_exists(obj.name):
            raise Exception("Cannot add the object {} to the scene since its name is not unique".format(obj.name))
        if isinstance(obj, RigidPrim):
            self._scene_registry.add_rigid_object(name=obj.name, rigid_object=obj)
        elif isinstance(obj, RigidPrimView):
            self._scene_registry.add_rigid_prim_view(name=obj.name, rigid_prim_view=obj)
        elif isinstance(obj, RigidContactView):
            self._scene_registry.add_rigid_contact_view(name=obj.name, rigid_contact_view=obj)
        elif isinstance(obj, GeometryPrim):
            self._scene_registry.add_geometry_object(name=obj.name, geometry_object=obj)
        elif isinstance(obj, GroundPlane):
            self._scene_registry.add_geometry_object(name=obj.name, geometry_object=obj)
        elif isinstance(obj, GeometryPrimView):
            self._scene_registry.add_geometry_prim_view(name=obj.name, geometry_prim_view=obj)
        elif isinstance(obj, Robot):
            self._scene_registry.add_robot(name=obj.name, robot=obj)
        elif isinstance(obj, RobotView):
            self._scene_registry.add_robot_view(name=obj.name, robot_view=obj)
        elif isinstance(obj, Articulation):
            self._scene_registry.add_articulated_system(name=obj.name, articulated_system=obj)
        elif isinstance(obj, ArticulationView):
            self._scene_registry.add_articulated_view(name=obj.name, articulated_view=obj)
        elif isinstance(obj, BaseSensor):
            self._scene_registry.add_sensor(name=obj.name, sensor=obj)
        elif isinstance(obj, XFormPrim):
            self._scene_registry.add_xform(name=obj.name, xform=obj)
        elif isinstance(obj, XFormPrimView):
            self._scene_registry.add_xform_view(name=obj.name, xform_prim_view=obj)
        elif isinstance(obj, ClothPrim):
            self._scene_registry.add_cloth(name=obj.name, cloth=obj)
        elif isinstance(obj, ClothPrimView):
            self._scene_registry.add_cloth_view(name=obj.name, cloth_prim_view=obj)
        elif isinstance(obj, ParticleSystem):
            self._scene_registry.add_particle_system(name=obj.name, particle_system=obj)
        elif isinstance(obj, ParticleSystemView):
            self._scene_registry.add_particle_system_view(name=obj.name, particle_system_view=obj)
        elif isinstance(obj, ParticleMaterial):
            self._scene_registry.add_particle_material(name=obj.name, particle_material=obj)
        elif isinstance(obj, ParticleMaterialView):
            self._scene_registry.add_particle_material_view(name=obj.name, particle_material_view=obj)
        else:
            raise Exception("object type is not supported yet")
        return obj

    def add_ground_plane(
        self,
        size: Optional[float] = None,
        z_position: float = 0,
        name="ground_plane",
        prim_path: str = "/World/groundPlane",
        static_friction: float = 0.5,
        dynamic_friction: float = 0.5,
        restitution: float = 0.8,
        color: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]

        Args:
            size (Optional[float], optional): [description]. Defaults to None.
            z_position (float, optional): [description]. Defaults to 0.
            name (str, optional): [description]. Defaults to "ground_plane".
            prim_path (str, optional): [description]. Defaults to "/World/groundPlane".
            static_friction (float, optional): [description]. Defaults to 0.5.
            dynamic_friction (float, optional): [description]. Defaults to 0.5.
            restitution (float, optional): [description]. Defaults to 0.8.
            color (Optional[np.ndarray], optional): [description]. Defaults to None.

        Returns:
            [type]: [description]
        """
        if Scene.object_exists(self, name=name):
            carb.log_info("ground floor already created with name {}.".format(name))
            return Scene.get_object(self, name=name)
        physics_material_path = find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            restitution=restitution,
        )
        plane = GroundPlane(
            prim_path=prim_path,
            name=name,
            z_position=z_position,
            size=size,
            color=color,
            physics_material=physics_material,
        )
        Scene.add(self, plane)
        return plane

    def add_default_ground_plane(
        self,
        z_position: float = 0,
        name="default_ground_plane",
        prim_path: str = "/World/defaultGroundPlane",
        static_friction: float = 0.5,
        dynamic_friction: float = 0.5,
        restitution: float = 0.8,
    ) -> None:
        """[summary]

        Args:
            z_position (float, optional): [description]. Defaults to 0.
            name (str, optional): [description]. Defaults to "default_ground_plane".
            prim_path (str, optional): [description]. Defaults to "/World/defaultGroundPlane".
            static_friction (float, optional): [description]. Defaults to 0.5.
            dynamic_friction (float, optional): [description]. Defaults to 0.5.
            restitution (float, optional): [description]. Defaults to 0.8.

        Returns:
            [type]: [description]
        """
        if Scene.object_exists(self, name=name):
            carb.log_info("ground floor already created with name {}.".format(name))
            return Scene.get_object(self, name=name)
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        physics_material_path = find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            restitution=restitution,
        )
        plane = GroundPlane(prim_path=prim_path, name=name, z_position=z_position, physics_material=physics_material)
        Scene.add(self, plane)
        return plane

    def post_reset(self) -> None:
        """calls post_reset on all added objects to the Scene Registery.
        """
        prim_registries_available = [
            self._scene_registry._geometry_objects,
            self._scene_registry._rigid_objects,
            self._scene_registry.rigid_prim_views,
            self._scene_registry.rigid_contact_views,
            self._scene_registry.geometry_prim_views,
            self._scene_registry._articulated_systems,
            self._scene_registry._articulated_views,
            self._scene_registry._robots,
            self._scene_registry._sensors,
            self._scene_registry.xforms,
            self._scene_registry._robot_views,
            self._scene_registry._xform_prim_views,
            self._scene_registry._cloth_prims,
            self._scene_registry._particle_systems,
            self._scene_registry._particle_materials,
            self._scene_registry._cloth_prim_views,
            self._scene_registry._particle_system_views,
            self._scene_registry._particle_material_views,
        ]

        for prim_registery in prim_registries_available:
            for prim_name in list(prim_registery):
                if not prim_registery[prim_name].is_valid():
                    prim_object = prim_registery[prim_name]
                    self._scene_registry.remove_object(name=prim_name)
                    del prim_object
                else:
                    prim_registery[prim_name].post_reset()
        if self._enable_bounding_box_computations:
            self._bbox_cache.Clear()
        gc.collect()
        return

    def _finalize(self, physics_sim_view) -> None:
        for xform_name, xform_object in self._scene_registry.xforms.items():
            xform_object.initialize(physics_sim_view)
        for xform_name, xform_view in self._scene_registry.xform_prim_views.items():
            xform_view.initialize(physics_sim_view)
        for cloth_name, cloth_object in self._scene_registry.cloth_prims.items():
            cloth_object.initialize(physics_sim_view)
        for cloth_name, cloth_object in self._scene_registry.cloth_prim_views.items():
            cloth_object.initialize(physics_sim_view)
        for particle_system_name, particle_system_object in self._scene_registry.particle_systems.items():
            particle_system_object.initialize(physics_sim_view)
        for particle_system_name, particle_system_object in self._scene_registry.particle_system_views.items():
            particle_system_object.initialize(physics_sim_view)
        for particle_material_name, particle_material_object in self._scene_registry.particle_materials.items():
            particle_material_object.initialize(physics_sim_view)
        for particle_material_name, particle_material_object in self._scene_registry.particle_material_views.items():
            particle_material_object.initialize(physics_sim_view)
        for sensor_name, sensor_object in self._scene_registry.sensors.items():
            sensor_object.initialize(physics_sim_view)
        for geometry_prim_name, geometry_object in self._scene_registry._geometry_objects.items():
            geometry_object.initialize(physics_sim_view)
        for geometry_prim_name, geometry_view in self._scene_registry.geometry_prim_views.items():
            geometry_view.initialize(physics_sim_view)
        for articulation_name, articulated_system in self._scene_registry.articulated_systems.items():
            articulated_system.initialize(physics_sim_view)
        for articulation_name, articulated_view in self._scene_registry.articulated_views.items():
            articulated_view.initialize(physics_sim_view)
        for robot_name, robot in self._scene_registry.robots.items():
            robot.initialize(physics_sim_view)
        for robots_name, robot_view in self._scene_registry.robot_views.items():
            robot_view.initialize(physics_sim_view)
        for rigid_object_name, rigid_object in self._scene_registry.rigid_objects.items():
            rigid_object.initialize(physics_sim_view)
        for rigid_prim_view_name, rigid_prim_view in self._scene_registry.rigid_prim_views.items():
            rigid_prim_view.initialize(physics_sim_view)
        for rigid_contact_view_name, rigid_contact_view in self._scene_registry.rigid_contact_views.items():
            rigid_contact_view.initialize(physics_sim_view)
        return

    def remove_object(self, name: str, registry_only: bool = False) -> None:
        """[summary]

        Args:
            name (str): Name of the prim to be removed. Defaults to None.
            registry_only (bool, optional): True to remove the object from the scene registery only and not the USD. Defaults to False.
        """
        prim_object = self.get_object(name=name)
        # sometimes the prim path is under a reference
        if not registry_only:
            if hasattr(prim_object, "prim"):
                prims = [prim_object.prim]
            elif hasattr(prim_object, "prims"):
                prims = prim_object.prims
            else:
                carb.log_error(f"No attribute prim present under the key: {name}")
                return
            # iterate over prims under prim-object
            for current_prim in prims:
                prim_path = get_prim_path(current_prim)
                while not is_prim_root_path(prim_path):
                    if not is_prim_ancestral(prim_path):
                        break
                    else:  # its under a reference
                        parent_prim = get_prim_parent(current_prim)
                        children_number = len(get_prim_children(parent_prim))
                        if children_number > 1:
                            break
                        current_prim = parent_prim
                        prim_path = get_prim_path(current_prim)
                if is_prim_path_valid(prim_path) and not is_prim_ancestral(prim_path):
                    omni.usd.commands.DeletePrimsCommand([get_prim_path(current_prim)]).do()
            # update the stage
            if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:
                update_stage()
        self._scene_registry.remove_object(name=name)
        del prim_object
        return

    def get_object(self, name: str) -> XFormPrim:
        """[summary]

        Args:
            name (Optional[str], optional): [description]. Defaults to None.
            prim_path (Optional[str], optional): [description]. Defaults to None.

        Returns:
            XFormPrim: [description]
        """
        return self._scene_registry.get_object(name=name)

    def object_exists(self, name: str) -> bool:
        """[summary]

        Args:
            name (str): [description]

        Returns:
            XFormPrim: [description]
        """
        if self._scene_registry.name_exists(name):
            return True
        else:
            return False

    def clear(self, registry_only: bool = False) -> None:
        """Clears the stage from all added objects to the Scene.

        Args:
            registry_only (bool, optional): True to remove the object from the scene registery only and not the USD. Defaults to False.
        """
        # Group all of the stage delete events together
        with Sdf.ChangeBlock():
            for geometry_object_name in list(self._scene_registry._geometry_objects):
                self.remove_object(geometry_object_name, registry_only=registry_only)
            for geometry_prim_view_name in list(self._scene_registry.geometry_prim_views):
                self.remove_object(geometry_prim_view_name, registry_only=registry_only)
            for rigid_object_name in list(self._scene_registry._rigid_objects):
                self.remove_object(rigid_object_name, registry_only=registry_only)
            for rigid_prim_view_name in list(self._scene_registry.rigid_prim_views):
                self.remove_object(rigid_prim_view_name, registry_only=registry_only)
            for rigid_contact_view_name in list(self._scene_registry._rigid_contact_views):
                self.remove_object(rigid_contact_view_name, registry_only=registry_only)
            for articulated_system_name in list(self._scene_registry._articulated_systems):
                self.remove_object(articulated_system_name, registry_only=registry_only)
            for articulated_view in list(self._scene_registry._articulated_views):
                self.remove_object(articulated_view, registry_only=registry_only)
            for robot_name in list(self._scene_registry._robots):
                self.remove_object(robot_name, registry_only=registry_only)
            for sensor_name in list(self._scene_registry.sensors):
                self.remove_object(sensor_name, registry_only=registry_only)
            for xform_name in list(self._scene_registry.xforms):
                self.remove_object(xform_name, registry_only=registry_only)
            for robot_name in list(self._scene_registry._robot_views):
                self.remove_object(robot_name, registry_only=registry_only)
            for xform_name in list(self._scene_registry._xform_prim_views):
                self.remove_object(xform_name, registry_only=registry_only)

            for cloth_name in list(self._scene_registry._cloth_prims):
                self.remove_object(cloth_name, registry_only=registry_only)
            for particle_system_name in list(self._scene_registry._particle_systems):
                self.remove_object(particle_system_name, registry_only=registry_only)
            for particle_material_name in list(self._scene_registry._particle_materials):
                self.remove_object(particle_material_name, registry_only=registry_only)
            for cloth_name in list(self._scene_registry._cloth_prim_views):
                self.remove_object(cloth_name, registry_only=registry_only)
            for particle_system_name in list(self._scene_registry._particle_system_views):
                self.remove_object(particle_system_name, registry_only=registry_only)
            for particle_material_name in list(self._scene_registry._particle_material_views):
                self.remove_object(particle_material_name, registry_only=registry_only)
        return

    def compute_object_AABB(self, name: str) -> Tuple[np.ndarray, np.ndarray]:
        """[summary]

        Args:
            name (str): [description]

        Raises:
            Exception: [description]

        Returns:
            Tuple[np.ndarray, np.ndarray]: [description]
        """
        if not self._enable_bounding_box_computations:
            raise Exception("bounding box computations should be enabled before quering AABB of an object")
        prim_object = self.get_object(name)
        if not hasattr(prim_object, "prim"):
            carb.log_error(f"Computing AABB bounds supported only for single classes.")
            return
        bounds = self._bbox_cache.ComputeWorldBound(prim_object.prim)
        prim_range = bounds.ComputeAlignedRange()
        return np.array([np.array(prim_range.GetMin()), np.array(prim_range.GetMax())])

    def enable_bounding_boxes_computations(self) -> None:
        """[summary]
        """
        self._bbox_cache = UsdGeom.BBoxCache(
            time=Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=False
        )
        self._enable_bounding_box_computations = True
        return

    def disable_bounding_boxes_computations(self) -> None:
        """[summary]
        """
        self._bbox_cache = None
        self._enable_bounding_box_computations = False
        return
