import lula
import carb
import numpy as np
from typing import List, Union, Optional
from omni.isaac.motion_generation.world_interface import WorldInterface
from omni.isaac.core import objects
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
from omni.isaac.core.utils.stage import get_stage_units
from .utils import get_prim_pose_in_meters_rel_robot_base, get_pose3


class LulaWorld(WorldInterface):
    def __init__(self):
        self._world = lula.create_world()
        self._dynamic_obstacles = dict()
        self._static_obstacles = dict()
        self._meters_per_unit = get_stage_units()

        # maintain a map of core.objects.ground_plane to ground-like cuboids that lula made to support the ground plane add function
        self._ground_plane_map = dict()

    def update_world(
        self,
        updated_obstacles: Optional[List] = None,
        robot_pos: Optional[np.array] = np.zeros(3),
        robot_rot: Optional[np.array] = np.eye(3),
        robot_base_moved: bool = False,
    ) -> None:
        """Update the internal world state of Lula.
        This function automatically tracks the positions of obstacles that have been added with add_obstacle()

        Args:
            updated_obstacles (List[core.objects], optional): Obstacles that have been added by add_obstacle() that need to be updated.
                If not specified, all non-static obstacle positions will be updated.
                If specified, only the obstacles that have been listed will have their positions updated
        """
        if updated_obstacles is None or robot_base_moved:
            # assume that all obstacle poses need to be updated
            updated_obstacles = self._dynamic_obstacles.keys()

        for obstacle_prim in updated_obstacles:
            obstacle_handle = self._dynamic_obstacles[obstacle_prim]
            trans, rot = get_prim_pose_in_meters_rel_robot_base(
                obstacle_prim, self._meters_per_unit, robot_pos, robot_rot
            )

            pose = get_pose3(trans, rot)
            self._world.set_pose(obstacle_handle, pose)

        if robot_base_moved:
            # update static obstacles
            for (obstacle_prim, obstacle_handle) in self._static_obstacles.items():
                trans, rot = get_prim_pose_in_meters_rel_robot_base(
                    obstacle_prim, self._meters_per_unit, robot_pos, robot_rot
                )

                pose = get_pose3(trans, rot)
                self._world.set_pose(obstacle_handle, pose)

    def add_cuboid(
        self,
        cuboid: Union[objects.cuboid.DynamicCuboid, objects.cuboid.FixedCuboid, objects.cuboid.VisualCuboid],
        static: Optional[bool] = False,
        robot_pos: Optional[np.array] = np.zeros(3),
        robot_rot: Optional[np.array] = np.eye(3),
    ):
        """Add a block obstacle.

        Args:
            cuboid (core.objects.cuboid): Wrapper object for handling rectangular prism Usd Prims.
            static (bool, optional): If True, indicate that cuboid will never change pose, and may be ignored in internal 
                world updates. Since Lula specifies object positions relative to the robot's frame 
                of reference, static obstacles will have their positions queried any time that 
                set_robot_base_pose() is called.  Defaults to False.


        Returns:
            bool: Always True, indicating that this adder has been implemented
        """

        if cuboid in self._static_obstacles or cuboid in self._dynamic_obstacles:
            carb.log_warn(
                "A cuboid was added twice to a Lula based MotionPolicy.  This has no effect beyond adding the cuboid once."
            )
            return False

        side_lengths = cuboid.get_size() * cuboid.get_local_scale() * self._meters_per_unit

        trans, rot = get_prim_pose_in_meters_rel_robot_base(cuboid, self._meters_per_unit, robot_pos, robot_rot)

        lula_cuboid = lula.create_obstacle(lula.Obstacle.Type.CUBE)
        lula_cuboid.set_attribute(lula.Obstacle.Attribute.SIDE_LENGTHS, side_lengths.astype(np.float64))
        lula_cuboid_pose = get_pose3(trans, rot)
        world_view = self._world.add_world_view()
        lula_cuboid_handle = self._world.add_obstacle(lula_cuboid, lula_cuboid_pose)
        world_view.update()

        if static:
            self._static_obstacles[cuboid] = lula_cuboid_handle
        else:
            self._dynamic_obstacles[cuboid] = lula_cuboid_handle

        return True

    def add_sphere(
        self,
        sphere: Union[objects.sphere.DynamicSphere, objects.sphere.VisualSphere],
        static: bool = False,
        robot_pos: Optional[np.array] = np.zeros(3),
        robot_rot: Optional[np.array] = np.eye(3),
    ) -> bool:
        """Add a sphere obstacle.

        Args:
            sphere (core.objects.sphere): Wrapper object for handling sphere Usd Prims.
            static (bool, optional): If True, indicate that sphere will never change pose, and may be ignored in internal 
                world updates. Since Lula specifies object positions relative to the robot's frame 
                of reference, static obstacles will have their positions queried any time that 
                set_robot_base_pose() is called.  Defaults to False.


        Returns:
            bool: Always True, indicating that this adder has been implemented
        """
        if sphere in self._static_obstacles or sphere in self._dynamic_obstacles:
            carb.log_warn(
                "A sphere was added twice to a Lula based MotionPolicy.  This has no effect beyond adding the sphere once."
            )
            return False

        radius = sphere.get_radius() * self._meters_per_unit
        trans, rot = get_prim_pose_in_meters_rel_robot_base(sphere, self._meters_per_unit, robot_pos, robot_rot)

        lula_sphere = lula.create_obstacle(lula.Obstacle.Type.SPHERE)
        lula_sphere.set_attribute(lula.Obstacle.Attribute.RADIUS, radius)
        lula_sphere_pose = get_pose3(trans, rot)
        lula_sphere_handle = self._world.add_obstacle(lula_sphere, lula_sphere_pose)

        if static:
            self._static_obstacles[sphere] = lula_sphere_handle
        else:
            self._dynamic_obstacles[sphere] = lula_sphere_handle

        return True

    def add_capsule(
        self,
        capsule: Union[objects.capsule.DynamicCapsule, objects.capsule.VisualCapsule],
        static: bool = False,
        robot_pos: Optional[np.array] = np.zeros(3),
        robot_rot: Optional[np.array] = np.eye(3),
    ) -> bool:
        """Add a capsule obstacle.

        Args:
            capsule (core.objects.capsule): Wrapper object for handling capsule Usd Prims.
            static (bool, optional): If True, indicate that capsule will never change pose, and may be ignored in internal 
                world updates. Since Lula specifies object positions relative to the robot's frame 
                of reference, static obstacles will have their positions queried any time that 
                set_robot_base_pose() is called.  Defaults to False.

        Returns:
            bool: Always True, indicating that this function has been implemented
        """

        # As of Lula 0.5.0, what Lula calls a "cylinder" is actually a capsule (i.e., the surface
        # defined by the set of all points a fixed distance from a line segment).  This will be
        # corrected in a future release of Lula.

        if capsule in self._static_obstacles or capsule in self._dynamic_obstacles:
            carb.log_warn(
                "A capsule was added twice to a Lula based MotionPolicy.  This has no effect beyond adding the capsule once."
            )
            return False

        radius = capsule.get_radius() * self._meters_per_unit
        height = capsule.get_height() * self._meters_per_unit

        trans, rot = get_prim_pose_in_meters_rel_robot_base(capsule, self._meters_per_unit, robot_pos, robot_rot)

        lula_capsule = lula.create_obstacle(lula.Obstacle.Type.CYLINDER)
        lula_capsule.set_attribute(lula.Obstacle.Attribute.RADIUS, radius)
        lula_capsule.set_attribute(lula.Obstacle.Attribute.HEIGHT, height)

        lula_capsule_pose = get_pose3(trans, rot)
        lula_capsule_handle = self._world.add_obstacle(lula_capsule, lula_capsule_pose)

        if static:
            self._static_obstacles[capsule] = lula_capsule_handle
        else:
            self._dynamic_obstacles[capsule] = lula_capsule_handle

        return True

    def add_ground_plane(
        self, ground_plane: objects.ground_plane.GroundPlane, plane_width: Optional[float] = 50.0
    ) -> bool:
        """Add a ground_plane.  
        Lula does not support ground planes directly, and instead internally creates a cuboid with an
        expansive face (dimensions 200x200 stage units) coplanar to the ground_plane.

        Args:
            ground_plane (core.objects.ground_plane.GroundPlane): Wrapper object for handling ground_plane Usd Prims.
            plane_width (Optional[float]): The width of the ground plane (in meters) that Lula creates to constrain this robot.  Defaults to 50.0 m

        Returns:
            bool: Always True, indicating that this adder has been implemented
        """
        if ground_plane in self._ground_plane_map:
            carb.log_warn(
                "A ground plane was added twice to a Lula based MotionPolicy.  This has no effect beyond adding the ground plane once."
            )
            return False

        plane_width = plane_width / self._meters_per_unit

        # ignore the ground plane and make a block instead, as lula doesn't support ground planes

        prim_path = find_unique_string_name("/lula/ground_plane", lambda x: not is_prim_path_valid(x))

        ground_width = 0.001  # meters
        lula_ground_plane_cuboid = objects.cuboid.VisualCuboid(
            prim_path, size=1.0, scale=np.array([plane_width, plane_width, ground_width / self._meters_per_unit])
        )
        lula_ground_plane_translation = ground_plane.get_world_pose()[0] - (
            np.array([0, 0, ground_width / 2]) / self._meters_per_unit
        )
        lula_ground_plane_cuboid.set_world_pose(lula_ground_plane_translation)
        lula_ground_plane_cuboid.set_visibility(False)

        self._ground_plane_map[ground_plane] = lula_ground_plane_cuboid
        self.add_cuboid(lula_ground_plane_cuboid, static=True)

        return True

    def disable_obstacle(self, obstacle: objects) -> bool:
        """Disable collision avoidance for obstacle.

        Args:
            obstacle (core.objects): obstacle to be disabled.

        Returns:
            bool: Return True if obstacle was identified and successfully disabled.
        """
        if obstacle in self._dynamic_obstacles:
            obstacle_handle = self._dynamic_obstacles[obstacle]
        elif obstacle in self._static_obstacles:
            obstacle_handle = self._static_obstacles[obstacle]
        elif obstacle in self._ground_plane_map:
            obstacle_handle = self._static_obstacles[self._ground_plane_map[obstacle]]
        else:
            return False
        self._world.disable_obstacle(obstacle_handle)
        return True

    def enable_obstacle(self, obstacle: objects) -> bool:
        """Enable collision avoidance for obstacle.

        Args:
            obstacle (core.objects): obstacle to be enabled.

        Returns:
            bool: Return True if obstacle was identified and successfully enabled.
        """
        if obstacle in self._dynamic_obstacles:
            obstacle_handle = self._dynamic_obstacles[obstacle]
        elif obstacle in self._static_obstacles:
            obstacle_handle = self._static_obstacles[obstacle]
        elif obstacle in self._ground_plane_map:
            obstacle_handle = self._static_obstacles[self._ground_plane_map[obstacle]]
        else:
            return False
        self._world.enable_obstacle(obstacle_handle)
        return True

    def remove_obstacle(self, obstacle: objects) -> bool:
        """Remove obstacle from collision avoidance. Obstacle cannot be re-enabled via enable_obstacle() after 
        removal.
        
        Args:
            obstacle (core.objects): obstacle to be removed.

        Returns:
            bool: Return True if obstacle was identified and successfully removed.
        """
        if obstacle in self._dynamic_obstacles:
            obstacle_handle = self._dynamic_obstacles[obstacle]
            del self._dynamic_obstacles[obstacle]
        elif obstacle in self._static_obstacles:
            obstacle_handle = self._static_obstacles[obstacle]
            del self._static_obstacles[obstacle]
        elif obstacle in self._ground_plane_map:
            lula_ground_plane_cuboid = self._ground_plane_map[obstacle]
            obstacle_handle = self._static_obstacles[lula_ground_plane_cuboid]
            delete_prim(lula_ground_plane_cuboid.prim_path)
            del self._static_obstacles[lula_ground_plane_cuboid]
            del self._ground_plane_map[obstacle]
        else:
            return False
        self._world.remove_obstacle(obstacle_handle)
        return True

    def reset(self) -> None:
        """reset the world to its initial state
        """
        self._world = lula.create_world()
        self._dynamic_obstacles = dict()
        self._static_obstacles = dict()

        for lula_ground_plane_cuboid in self._ground_plane_map.values():
            delete_prim(lula_ground_plane_cuboid.prim_path)
        self._ground_plane_map = dict()
