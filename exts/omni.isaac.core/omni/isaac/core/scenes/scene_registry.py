# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.prims.rigid_contact_view import RigidContactView
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.prims.geometry_prim_view import GeometryPrimView
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.core.prims.soft.cloth_prim import ClothPrim
from omni.isaac.core.prims.soft.cloth_prim_view import ClothPrimView
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.prims.soft.particle_system_view import ParticleSystemView
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.materials.particle_material_view import ParticleMaterialView
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.robots.robot_view import RobotView
from omni.isaac.core.prims.base_sensor import BaseSensor


class SceneRegistry(object):
    def __init__(self) -> None:
        """[summary]
        """
        self._rigid_objects = dict()
        self._geometry_objects = dict()
        self._articulated_systems = dict()
        self._robots = dict()
        self._xforms = dict()
        self._sensors = dict()
        self._xform_prim_views = dict()
        self._cloth_prims = dict()
        self._cloth_prim_views = dict()
        self._particle_systems = dict()
        self._particle_system_views = dict()
        self._particle_materials = dict()
        self._particle_material_views = dict()
        self._geometry_prim_views = dict()
        self._rigid_prim_views = dict()
        self._rigid_contact_views = dict()
        self._articulated_views = dict()
        self._robot_views = dict()
        return

    @property
    def articulated_systems(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._articulated_systems

    @property
    def rigid_objects(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._rigid_objects

    @property
    def rigid_prim_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._rigid_prim_views

    @property
    def rigid_contact_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._rigid_contact_views

    @property
    def geometry_prim_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._geometry_prim_views

    @property
    def articulated_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._articulated_views

    @property
    def robot_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._robot_views

    @property
    def robots(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._robots

    @property
    def xforms(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._xforms

    @property
    def sensors(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._sensors

    @property
    def xform_prim_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._xform_prim_views

    @property
    def cloth_prims(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._cloth_prims

    @property
    def cloth_prim_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._cloth_prim_views

    @property
    def particle_systems(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._particle_systems

    @property
    def particle_system_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._particle_system_views

    @property
    def particle_materials(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._particle_materials

    @property
    def particle_material_views(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        return self._particle_material_views

    # TODO: add if name exists check uniqueness
    def add_rigid_object(self, name, rigid_object: RigidPrim) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            rigid_object (RigidPrim): [description]
        """
        self._rigid_objects[name] = rigid_object
        return

    def add_rigid_prim_view(self, name, rigid_prim_view: RigidPrimView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            rigid_object (RigidPrim): [description]
        """
        self._rigid_prim_views[name] = rigid_prim_view
        return

    def add_rigid_contact_view(self, name, rigid_contact_view: RigidContactView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            rigid_contact_views (RigidContactView): [description]
        """
        self._rigid_contact_views[name] = rigid_contact_view
        return

    def add_articulated_system(self, name, articulated_system: Articulation) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            articulated_system (Articulation): [description]
        """
        self._articulated_systems[name] = articulated_system
        return

    def add_articulated_view(self, name, articulated_view: ArticulationView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            articulated_view (ArticulationView): [description]
        """
        self._articulated_views[name] = articulated_view
        return

    def add_geometry_object(self, name, geometry_object: GeometryPrim) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (GeometryPrim): [description]
        """
        self._geometry_objects[name] = geometry_object
        return

    def add_geometry_prim_view(self, name, geometry_prim_view: GeometryPrimView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (GeometryPrim): [description]
        """
        self._geometry_prim_views[name] = geometry_prim_view
        return

    def add_robot(self, name, robot: Robot) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            robot (Robot): [description]
        """
        self._robots[name] = robot
        return

    def add_robot_view(self, name, robot_view: RobotView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (GeometryPrim): [description]
        """
        self._robot_views[name] = robot_view
        return

    def add_xform_view(self, name, xform_prim_view: XFormPrimView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (GeometryPrim): [description]
        """
        self._xform_prim_views[name] = xform_prim_view
        return

    def add_cloth(self, name, cloth: ClothPrim) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            cloth (ClothPrim): [description]
        """
        self._cloth_prims[name] = cloth
        return

    def add_cloth_view(self, name, cloth_prim_view: ClothPrimView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (ClothPrimView): [description]
        """
        self._cloth_prim_views[name] = cloth_prim_view
        return

    def add_particle_system(self, name, particle_system: ParticleSystem) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (ParticleSystemView): [description]
        """
        self._particle_systems[name] = particle_system
        return

    def add_particle_system_view(self, name, particle_system_view: ParticleSystemView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (ParticleSystemView): [description]
        """
        self._particle_system_views[name] = particle_system_view
        return

    def add_particle_material(self, name, particle_material: ParticleMaterial) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (ParticleMaterial): [description]
        """
        self._particle_materials[name] = particle_material
        return

    def add_particle_material_view(self, name, particle_material_view: ParticleMaterialView) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            geometry_object (ParticleMaterialView): [description]
        """
        self._particle_material_views[name] = particle_material_view
        return

    def add_xform(self, name, xform: XFormPrim) -> None:
        """[summary]

        Args:
            name ([type]): [description]
            robot (Robot): [description]
        """
        self._xforms[name] = xform
        return

    def add_sensor(self, name, sensor: BaseSensor) -> None:
        """[summary]

        Args:
            name ([type]): [description]
                        sensor (BaseSensor): [description]
        """
        self._sensors[name] = sensor

        return

    def name_exists(self, name: str) -> bool:
        """[summary]

        Args:
            name (str): [description]

        Returns:
            bool: [description]
        """
        if (
            name in self._robots
            or name in self._articulated_systems
            or name in self._articulated_views
            or name in self._rigid_objects
            or name in self._geometry_objects
            or name in self._xforms
            or name in self._sensors
            or name in self._rigid_contact_views
            or name in self._rigid_prim_views
            or name in self._geometry_prim_views
            or name in self._robot_views
            or name in self._xform_prim_views
            or name in self._cloth_prims
            or name in self._cloth_prim_views
            or name in self._particle_system_views
            or name in self._particle_material_views
        ):
            return True
        else:
            return False

    def remove_object(self, name: str) -> None:
        """[summary]

        Args:
            name (Optional[str], optional): [description]. Defaults to None.
            prim_path (Optional[str], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
            Exception: [description]
            NotImplementedError: [description]
            Exception: [description]
        """
        if name in self._robots:
            del self._robots[name]
            return
        elif name in self._articulated_systems:
            del self._articulated_systems[name]
            return
        elif name in self._articulated_views:
            del self._articulated_views[name]
            return
        elif name in self._rigid_objects:
            del self._rigid_objects[name]
            return
        elif name in self._geometry_objects:
            del self._geometry_objects[name]
            return
        elif name in self._geometry_prim_views:
            del self._geometry_prim_views[name]
            return
        elif name in self._sensors:
            del self._sensors[name]
            return
        elif name in self._xforms:
            del self._xforms[name]
            return
        elif name in self._xform_prim_views:
            del self._xform_prim_views[name]
            return
        elif name in self._cloth_prims:
            del self._cloth_prims[name]
            return
        elif name in self._cloth_prim_views:
            del self._cloth_prim_views[name]
            return
        elif name in self._particle_systems:
            del self._particle_systems[name]
            return
        elif name in self._particle_system_views:
            del self._particle_system_views[name]
            return
        elif name in self._particle_materials:
            del self._particle_materials[name]
            return
        elif name in self._particle_material_views:
            del self._particle_material_views[name]
            return
        elif name in self._robot_views:
            del self._robot_views[name]
            return
        elif name in self._rigid_prim_views:
            del self._rigid_prim_views[name]
            return
        elif name in self._rigid_contact_views:
            del self._rigid_contact_views[name]
            return
        else:
            raise Exception("Cannot remove object {} from the scene since it doesn't exist".format(name))

    def get_object(self, name: str) -> XFormPrim:
        """[summary]

        Args:
            name (Optional[str], optional): [description]. Defaults to None.
            prim_path (Optional[str], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]

        Returns:
            XFormPrim: [description]
        """
        if name in self._robots:
            return self._robots[name]
        elif name in self._articulated_systems:
            return self._articulated_systems[name]
        elif name in self._articulated_views:
            return self._articulated_views[name]
        elif name in self._rigid_objects:
            return self._rigid_objects[name]
        elif name in self._rigid_prim_views:
            return self._rigid_prim_views[name]
        elif name in self._rigid_contact_views:
            return self._rigid_contact_views[name]
        elif name in self._geometry_prim_views:
            return self._geometry_prim_views[name]
        elif name in self._geometry_objects:
            return self._geometry_objects[name]
        elif name in self._sensors:
            return self._sensors[name]
        elif name in self._xforms:
            return self._xforms[name]
        elif name in self._xform_prim_views:
            return self._xform_prim_views[name]
        elif name in self._cloth_prims:
            return self._cloth_prims[name]
        elif name in self._cloth_prim_views:
            return self._cloth_prim_views[name]
        elif name in self._particle_systems:
            return self._particle_systems[name]
        elif name in self._particle_system_views:
            return self._particle_system_views[name]
        elif name in self._particle_materials:
            return self._particle_materials[name]
        elif name in self._particle_material_views:
            return self._particle_material_views[name]
        elif name in self._robot_views:
            return self._robot_views[name]
