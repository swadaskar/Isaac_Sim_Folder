import lula
import numpy as np

from .world import LulaWorld
from typing import List, Tuple, Union, Optional
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core import objects
from . import utils as lula_utils


class LulaInterfaceHelper(LulaWorld):
    """
    Class containing functions common in Lula based algorithms.  The main utility of this class is handling the tracking of the robot base
    and returning basic robot information
    """

    def __init__(self, robot_description: lula.RobotDescription):
        LulaWorld.__init__(self)

        self._robot_description = robot_description
        self._kinematics = self._robot_description.kinematics()

        self._robot_base_moved = False
        self._robot_pos, self._robot_rot = np.zeros(3), np.eye(3)

        self._meters_per_unit = get_stage_units()

    def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array) -> None:
        """Update position of the robot base. Until this function is called, Lula will assume the base pose
        to be at the origin with identity rotation.

        Args:
            robot_position (np.array): (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation (np.array): (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """
        # all object poses are relative to the position of the robot base
        robot_position = robot_position * self._meters_per_unit
        robot_rot = quats_to_rot_matrices(robot_orientation)

        if np.any(self._robot_pos - robot_position) or np.any(self._robot_rot - robot_rot):
            self._robot_base_moved = True
        else:
            self._robot_base_moved = False

        self._robot_pos = robot_position
        self._robot_rot = robot_rot

    def get_active_joints(self):
        return [
            self._robot_description.c_space_coord_name(i) for i in range(self._robot_description.num_c_space_coords())
        ]

    def get_watched_joints(self) -> List:
        """Lula does not currently support watching joint states that are not controllable

        Returns:
            (List): Always returns an empty list.
        """
        return []

    def get_end_effector_pose(self, active_joint_positions: np.array, frame_name: str) -> Tuple[np.array, np.array]:
        """Return pose of robot end effector given current joint positions.
        The end effector position will be transformed into world coordinates based
        on the believed position of the robot base.  See set_robot_base_pose()

        Args:
            active_joint_positions (np.array): positions of the active joints in the robot

        Returns:
            Tuple[np.array,np.array]:
            end_effector_translation: (3x1) translation vector for the robot end effector
                 relative to the USD stage origin \n
            end_effector_rotation: (3x3) rotation matrix describing the orientation of the
                robot end effector relative to the USD global frame \n
        """
        # returns pose of end effector in world coordinates
        pose = self._kinematics.pose(np.expand_dims(active_joint_positions, 1), frame_name)

        translation = self._robot_rot @ (pose.translation) + self._robot_pos
        rotation = self._robot_rot @ pose.rotation.matrix()
        return translation / self._meters_per_unit, rotation

    def update_world(self, updated_obstacles: Optional[List] = None):
        LulaWorld.update_world(self, updated_obstacles, self._robot_pos, self._robot_rot, self._robot_base_moved)
        self._robot_base_moved = False

    def add_cuboid(
        self,
        cuboid: Union[objects.cuboid.DynamicCuboid, objects.cuboid.FixedCuboid, objects.cuboid.VisualCuboid],
        static: Optional[bool] = False,
    ):
        return LulaWorld.add_cuboid(self, cuboid, static, self._robot_pos, self._robot_rot)

    def add_sphere(
        self, sphere: Union[objects.sphere.DynamicSphere, objects.sphere.VisualSphere], static: bool = False
    ):
        return LulaWorld.add_sphere(self, sphere, static, self._robot_pos, self._robot_rot)

    def add_capsule(
        self, capsule: Union[objects.capsule.DynamicCapsule, objects.capsule.VisualCapsule], static: bool = False
    ):
        return LulaWorld.add_capsule(self, capsule, static, self._robot_pos, self._robot_rot)

    def reset(self):
        LulaWorld.reset(self)

        self._robot_base_moved = False
        self._robot_pos, self._robot_rot = np.zeros(3), np.eye(3)

    def _get_prim_pose_rel_robot_base(self, prim):
        # returns the position of a prim relative to the position of the robot
        return lula_utils.get_prim_pose_in_meters_rel_robot_base(
            prim, self._meters_per_unit, self._robot_pos, self._robot_rot
        )

    def _get_pose_rel_robot_base(self, trans, rot):
        return lula_utils.get_pose_rel_robot_base(trans, rot, self._robot_pos, self._robot_rot)
