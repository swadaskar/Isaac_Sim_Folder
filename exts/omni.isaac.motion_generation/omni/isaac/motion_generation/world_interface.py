# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from typing import List, Union, Optional
import omni.isaac.core.objects
from omni.isaac.core.objects import cuboid, sphere, capsule, cylinder, cone, ground_plane


class WorldInterface:
    """Interface for translating USD world to a generic world-aware algorithm such as a MotionPolicy
    """

    def __init__(self) -> None:
        pass

    def update_world(self, updated_obstacles: Optional[List] = None) -> None:
        """Applies all necessary updates to the internal world representation.

        Args:
            updated_obstacles (list, optional): If provided, only the given obstacles will have their poses updated.
                For motion policies that use obstacle poses relative to the robot base (e.g. Lula based policies), 
                this list will be ignored if the robot base has moved because all object poses will have changed 
                relative to the robot. Defaults to None.
        
        Returns:
            None
        """
        pass

    def add_obstacle(self, obstacle: omni.isaac.core.objects, static: Optional[bool] = False) -> bool:
        """Add an obstacle 

        Args:
            obstacle (omni.isaac.core.objects): An obstacle from the package omni.isaac.core.obstacles
                            The type of the obstacle will be checked, and the appropriate add function will be called \n
            static (Optional[bool]): When True, the obstacle will be assumed to remain stationary relative to the USD global frame over time

        Returns:
            success (bool): Returns True if the obstacle type is valid and the appropriate add function has been implemented
        """

        if (
            isinstance(obstacle, cuboid.DynamicCuboid)
            or isinstance(obstacle, cuboid.VisualCuboid)
            or isinstance(obstacle, cuboid.FixedCuboid)
        ):
            return self.add_cuboid(obstacle, static=static)

        elif isinstance(obstacle, cylinder.DynamicCylinder) or isinstance(obstacle, cylinder.VisualCylinder):
            return self.add_cylinder(obstacle, static=static)

        elif isinstance(obstacle, sphere.DynamicSphere) or isinstance(obstacle, sphere.VisualSphere):
            return self.add_sphere(obstacle, static=static)

        elif isinstance(obstacle, capsule.DynamicCapsule) or isinstance(obstacle, capsule.VisualCapsule):
            return self.add_capsule(obstacle, static=static)

        elif isinstance(obstacle, cone.DynamicCone) or isinstance(obstacle, cone.VisualCone):
            return self.add_cone(obstacle, static=static)

        elif isinstance(obstacle, ground_plane.GroundPlane):
            return self.add_ground_plane(obstacle)

        else:
            carb.log_warning(
                "Obstacle added with unsuported type: "
                + str(type(obstacle))
                + "\nObstacle should be from the package omni.isaac.core.objects"
            )
            return False

    def add_cuboid(
        self, cuboid: Union[cuboid.DynamicCuboid, cuboid.FixedCuboid, cuboid.VisualCuboid], static: bool = False
    ) -> bool:
        """Add a block obstacle.

        Args:
            cuboid (core.objects.cuboid): Wrapper object for handling rectangular prism Usd Prims.
            static (bool, optional): If True, indicate that cuboid will never change pose, and may be ignored in internal 
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cuboid()
        """
        carb.log_warning("Function add_cuboid() has not been implemented for this WorldInterface")
        return False

    def add_sphere(self, sphere: Union[sphere.DynamicSphere, sphere.VisualSphere], static: bool = False) -> bool:
        """Add a sphere obstacle.

        Args:
            sphere (core.objects.sphere): Wrapper object for handling sphere Usd Prims.
            static (bool, optional): If True, indicate that sphere will never change pose, and may be ignored in internal 
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_sphere()
        """
        carb.log_warning("Function add_sphere() has not been implemented for this WorldInterface")
        return False

    def add_capsule(self, capsule: Union[capsule.DynamicCapsule, capsule.VisualCapsule], static: bool = False) -> bool:
        """Add a capsule obstacle.

        Args:
            capsule (core.objects.capsule): Wrapper object for handling capsule Usd Prims.
            static (bool, optional): If True, indicate that capsule will never change pose, and may be ignored in internal 
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_capsule()
        """
        carb.log_warning("Function add_capsule() has not been implemented for this WorldInterface")
        return False

    def add_cylinder(
        self, cylinder: Union[cylinder.DynamicCylinder, cylinder.VisualCylinder], static: bool = False
    ) -> bool:
        """Add a cylinder obstacle.

        Args:
            cylinder (core.objects.cylinder): Wrapper object for handling rectangular prism Usd Prims.
            static (bool, optional): If True, indicate that cuboid will never change pose, and may be ignored in internal 
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cylinder()
        """
        carb.log_warning("Function add_cylinder() has not been implemented for this WorldInterface")
        return False

    def add_cone(self, cone: Union[cone.DynamicCone, cone.VisualCone], static: bool = False) -> bool:
        """Add a cone obstacle.

        Args:
            cone (core.objects.cone): Wrapper object for handling cone Usd Prims.
            static (bool, optional): If True, indicate that cone will never change pose, and may be ignored in internal 
                world updates. Defaults to False.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_cone()
        """
        carb.log_warning("Function add_cone() has not been implemented for this WorldInterface")
        return False

    def add_ground_plane(self, ground_plane: ground_plane.GroundPlane) -> bool:
        """Add a ground_plane

        Args:
            ground_plane (core.objects.ground_plane.GroundPlane): Wrapper object for handling ground_plane Usd Prims.

        Returns:
            bool: Return True if underlying WorldInterface has implemented add_ground_plane()
        """
        carb.log_warning("Function add_ground_plane() has not been implemented for this WorldInterface")
        return False

    def disable_obstacle(self, obstacle: omni.isaac.core.objects) -> bool:
        """Disable collision avoidance for obstacle.

        Args:
            obstacle (core.object): obstacle to be disabled.

        Returns:
            bool: Return True if obstacle was identified and successfully disabled.
        """
        carb.log_warning("Function disable_obstacle() has not been implemented for this WorldInterface")
        return False

    def enable_obstacle(self, obstacle: omni.isaac.core.objects) -> bool:
        """Enable collision avoidance for obstacle.

        Args:
            obstacle (core.object): obstacle to be enabled.

        Returns:
            bool: Return True if obstacle was identified and successfully enabled.
        """
        carb.log_warning("Function enable_obstacle() has not been implemented for this WorldInterface")
        return False

    def remove_obstacle(self, obstacle: omni.isaac.core.objects) -> bool:
        """Remove obstacle from collision avoidance. Obstacle cannot be re-enabled via enable_obstacle() after 
        removal.
        
        Args:
            obstacle (core.object): obstacle to be removed.

        Returns:
            bool: Return True if obstacle was identified and successfully removed.
        """
        carb.log_warning("Function remove_obstacle() has not been implemented for this WorldInterface")
        return False

    def reset(self) -> None:
        """Reset all state inside the WorldInterface to its initial values
        """
        pass
