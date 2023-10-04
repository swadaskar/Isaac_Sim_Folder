# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence
import omni.kit.app
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.manipulators.grippers.gripper import Gripper
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper
from omni.isaac.manipulators.grippers.surface_gripper import SurfaceGripper
from omni.isaac.core.prims.rigid_prim import RigidPrim


class SingleManipulator(Articulation):
    """Provides high level functions to set/ get properties and actions of a manipulator with a single end effector
        and optionally a gripper.

        Args:
        
            prim_path (str): prim path of the Prim to encapsulate or create.
            end_effector_prim_name (str): end effector prim name to be used to track the rigid body that corresponds
                                            to the end effector.
            name (str, optional): shortname to be used as a key by Scene class. Note: needs to be unique if the
                                    object is added to the Scene. Defaults to "single_manipulator".
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                            Defaults to None, which means left unchanged.
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                                (with respect to its parent prim). shape is (3, ).
                                                                Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                                (depends if translation or position is specified).
                                                                quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                                Defaults to None, which means left unchanged.
            scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                            Defaults to None, which means left unchanged.
            visible (Optional[bool], optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
            gripper (Gripper, optional): Gripper to be used with the manipulator. Defaults to None.
    """

    def __init__(
        self,
        prim_path: str,
        end_effector_prim_name: str,
        name: str = "single_manipulator",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        gripper: Gripper = None,
    ) -> None:
        self._end_effector_prim_name = end_effector_prim_name
        self._gripper = gripper
        self._end_effector = None
        Articulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=None,
        )
        return

    @property
    def end_effector(self) -> RigidPrim:
        """
        Returns:
            RigidPrim: end effector of the manipulator (can be used to get its world pose, angular velocity..etc).
        """
        return self._end_effector

    @property
    def gripper(self) -> Gripper:
        """
        Returns:
            Gripper: gripper of the manipulator (can be used to open or close the gripper, get its world pose or angular velocity..etc).
        """
        return self._gripper

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        Articulation.initialize(self, physics_sim_view=physics_sim_view)
        end_effector_prim_path = self.prim_path + "/" + self._end_effector_prim_name
        self._end_effector = RigidPrim(prim_path=end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        if isinstance(self._gripper, ParallelGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        if isinstance(self._gripper, SurfaceGripper):
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        return

    def post_reset(self) -> None:
        """Resets the manipulator, the end effector and the gripper to its default state.
        """
        Articulation.post_reset(self)
        self._end_effector.post_reset()
        self._gripper.post_reset()
        return
