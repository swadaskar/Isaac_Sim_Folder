import omni.timeline
import numpy as np

import omni.kit.commands
from pxr import UsdPhysics, Usd, Sdf
from pxr import Gf
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid, delete_prim
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from omni.isaac.core.prims import XFormPrim

from typing import List, Tuple
import carb


class ComposedRobot:
    def __init__(
        self,
        base_robot_path: str,
        attach_robot_path: str,
        fixed_joint_path: str,
        attach_robot_root_joints: List[UsdPhysics.Joint],
        collision_groups: List[str],
    ):
        self._base_robot_path = base_robot_path
        self._attach_robot_path = attach_robot_path

        self._fixed_joint_path = fixed_joint_path

        self._attach_robot_root_joints = attach_robot_root_joints
        self._collision_groups = collision_groups

        self._is_composed = True

    def is_composed(self) -> bool:
        """The composed robots are currently composed together.  I.e. the decompose() function has not been called.

        Returns:
            bool: The decompose() function has not been called.
        """
        return self._is_composed

    def decompose(self):
        """Decompose composed robots.  This can only be done one time, and it will result in all non-trivial functions in this class returning immediately.
        """
        if not self.is_composed():
            carb.log_warn("Cannot decompose a robot that has already been decomposed")
            return

        # Reactivate the gripper Articulation Root
        art_b_prim = get_prim_at_path(self._attach_robot_path)
        if art_b_prim.HasProperty("physxArticulation:articulationEnabled"):
            art_b_prim.GetProperty("physxArticulation:articulationEnabled").Set(True)

        # Reactivate the root joints tying attach robot to stage
        for root_joint in self._attach_robot_root_joints:
            root_joint.GetProperty("physics:jointEnabled").Set(True)

        # Delete the Fixed Joint and attach_point_transform
        delete_prim(self._fixed_joint_path)

        for collision_group in self._collision_groups:
            delete_prim(collision_group)

        self._refresh_asset(self._attach_robot_path)
        self._refresh_asset(self._base_robot_path)

        self._is_composed = True

    def get_fixed_joint_transform(self) -> Tuple[np.array, np.array]:
        """Get the transform between mount frames in composed robot.

        Returns:
            Tuple[np.array, np.array]: translation with shape (3,) and orientation with shape (4,)
        """
        if not self.is_composed():
            carb.log_warn("Fixed joint no longer exists in composed robot.  Robots have been decomposed.")
            return None, None
        fixed_joint = UsdPhysics.Joint(get_prim_at_path(self._fixed_joint_path))
        translation = np.array(fixed_joint.GetLocalPos0Attr().Get())
        orientation = np.array(fixed_joint.GetLocalRot0Attr().Get())

        return translation, orientation

    def get_base_robot_prim_path(self) -> str:
        return self._base_robot_path

    def get_attached_robot_prim_path(self) -> str:
        return self._attach_robot_path

    def set_fixed_joint_transform(self, translation: np.array, orientation: np.array):
        """Set the transform between mount frames in the composed robot.

        Args:
            translation (np.array): Local translation relative to mount frame on base robot.
            orientation (np.array): Local quaternion orientation relative to mount frame on base robot.
        """
        if not self.is_composed():
            carb.log_warn("Fixed joint no longer exists in composed robot.  Robots have been decomposed.")
            return
        fixed_joint = UsdPhysics.Joint(get_prim_at_path(self._fixed_joint_path))
        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*translation.astype(float)))
        fixed_joint.GetLocalRot0Attr().Set(Gf.Quatf(*orientation.astype(float)))

        self._refresh_asset(self._attach_robot_path)
        self._refresh_asset(self._base_robot_path)

    def _refresh_asset(self, prim_path):
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = get_current_stage()
        prim = get_prim_at_path(prim_path)

        composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
        if len(composed_payloads) != 0:
            payload = Sdf.Payload(prim_path)
            omni.kit.commands.execute("RemovePayload", stage=stage, prim_path=prim_path, payload=payload)
            omni.kit.commands.execute("AddPayload", stage=stage, prim_path=prim_path, payload=payload)

        composed_refs = omni.usd.get_composed_references_from_prim(prim)
        if len(composed_refs) != 0:
            reference = Sdf.Reference(prim_path)
            omni.kit.commands.execute(
                "RemoveReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference
            )
            omni.kit.commands.execute("AddReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference)


class RobotComposer:
    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()

    def mask_collisions(
        self, group_a_path: str, a_collider_paths: List[str], group_b_path: str, b_collider_paths: List[str]
    ):
        """Mask collisions between two sets of prim paths.

        Args:
            group_a_path (str): Path where a CollisionGroup prim should go.
            a_collider_paths (List[str]): List of prim paths in one group.
            group_b_path (str): Path where the second CollisionGroup prim should go.
            b_collider_paths (List[str]): List of prim paths in the second group that will no longer be able to collide with the first group.
        """
        stage = get_current_stage()
        omni.kit.commands.execute("AddCollisionGroupCommand", stage=stage, path=group_a_path)

        omni.kit.commands.execute("AddCollisionGroupCommand", stage=stage, path=group_b_path)

        omni.kit.commands.execute(
            "AddRelationshipTarget",
            relationship=get_prim_at_path(group_a_path).GetRelationship("physics:filteredGroups"),
            target=group_b_path,
        )

        for collider_path in a_collider_paths:
            omni.kit.commands.execute(
                "AddRelationshipTarget",
                relationship=get_prim_at_path(group_a_path).GetRelationship("collection:colliders:includes"),
                target=collider_path,
            )
        for collider_path in b_collider_paths:
            omni.kit.commands.execute(
                "AddRelationshipTarget",
                relationship=get_prim_at_path(group_b_path).GetRelationship("collection:colliders:includes"),
                target=collider_path,
            )

    def compose(
        self,
        base_robot_path: str,
        attach_robot_path: str,
        base_robot_mount_frame: str,
        attach_robot_mount_frame: str,
        fixed_joint_offset: np.array = np.zeros(3),
        fixed_joint_orient: np.array = np.array([1, 0, 0, 0]),
        single_robot=False,
        mask_all_collisions=True,
    ) -> ComposedRobot:
        """Compose two robots into one physical structure

        Args:
            base_robot_path (str): Path to base robot.
            attach_robot_path (str): Path to attach robot.  The attach robot will be unrooted from the stage and attached only to the base robot
            base_robot_mount_frame (str): Relative path to frame in base robot where there is the desired attach point. 
            attach_robot_mount_frame (str): Relative path to frame in the attach robot where there is the desired attach point.
            fixed_joint_offset (np.array, optional): Fixed offset between attach points. Defaults to np.zeros(3).
            fixed_joint_orient (np.array, optional): Fixed orientation between attach points. Defaults to np.array([1, 0, 0, 0]).
            single_robot (bool, optional): If True: control the resulting composed robots as a single robot Articulation at base_robot_path. Defaults to False.
            mask_all_collisions (bool, optional): Mask all collisions between attach robot and base robot.  This is necessary when setting single_robot=False to prevent Physics constraint
                violations from the new fixed joint.  Advanced users may set this flag to False and use the mask_collisions() function separately for more customizable behavior.  Defaults to True.

        Returns:
            ComposedRobot: An object representing the composed robot.  This object can detach the composed robots and edit the fixed joint transform.
        """
        a_attach_path = base_robot_path + base_robot_mount_frame
        b_attach_path = attach_robot_path + attach_robot_mount_frame
        self._move_art_b_to_local_pos(a_attach_path, attach_robot_path, fixed_joint_offset, fixed_joint_orient)

        # Find and Disable Fixed Joints that Tie Articulation B to the Stage
        root_joints = [
            p
            for p in Usd.PrimRange(get_prim_at_path(attach_robot_path))
            if UsdPhysics.Joint(p)
            and (
                len(p.GetProperty("physics:body0").GetTargets()) == 0
                or len(p.GetProperty("physics:body1").GetTargets()) == 0
            )
        ]

        for root_joint in root_joints:
            root_joint.GetProperty("physics:jointEnabled").Set(False)

        # Create fixed Joint between attach frames
        fixed_joint_path = b_attach_path + "/ComposerFixedJoint"
        fixed_joint_path = find_unique_string_name(fixed_joint_path, lambda x: not is_prim_path_valid(x))

        stage = get_current_stage()
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
        fixed_joint.GetBody0Rel().SetTargets([a_attach_path])
        fixed_joint.GetBody1Rel().SetTargets([b_attach_path])

        # Update FixedJoint to have the desired transform

        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*fixed_joint_offset.astype(float)))
        fixed_joint.GetLocalRot0Attr().Set(Gf.Quatf(*fixed_joint_orient.astype(float)))
        fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(*np.zeros(3).astype(float)))
        fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(*np.array([1, 0, 0, 0]).astype(float)))

        # Disable Articulation Root on Articulation B so that A is always the prim path for the composed robot
        collision_groups = []
        if single_robot:
            art_b_prim = get_prim_at_path(attach_robot_path)
            if art_b_prim.HasProperty("physxArticulation:articulationEnabled"):
                art_b_prim.GetProperty("physxArticulation:articulationEnabled").Set(False)
        else:
            fixed_joint.GetExcludeFromArticulationAttr().Set(True)
            if mask_all_collisions:
                group_a_path = find_unique_string_name(
                    a_attach_path + "/CollisionGroup", lambda x: not is_prim_path_valid(x)
                )
                group_b_path = find_unique_string_name(
                    b_attach_path + "/CollisionGroup", lambda x: not is_prim_path_valid(x)
                )
                collision_groups.append(group_a_path)
                collision_groups.append(group_b_path)
                self.mask_collisions(group_a_path, [base_robot_path], group_b_path, [attach_robot_path])

        self._refresh_asset(base_robot_path)
        self._refresh_asset(attach_robot_path)

        return ComposedRobot(base_robot_path, attach_robot_path, fixed_joint_path, root_joints, collision_groups)

    def _refresh_asset(self, prim_path):
        # Refreshing payloads manually is a way to get the Articulation to update immediately while the timeline is
        # still playing.  Usd Physics should be doing this automatically, but there is currently a bug.  This function
        # will eventually become unnecessary.
        stage = get_current_stage()
        prim = get_prim_at_path(prim_path)

        composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
        if len(composed_payloads) != 0:
            payload = Sdf.Payload(prim_path)
            omni.kit.commands.execute("RemovePayload", stage=stage, prim_path=prim_path, payload=payload)
            omni.kit.commands.execute("AddPayload", stage=stage, prim_path=prim_path, payload=payload)

        composed_refs = omni.usd.get_composed_references_from_prim(prim)
        if len(composed_refs) != 0:
            reference = Sdf.Reference(prim_path)
            omni.kit.commands.execute(
                "RemoveReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference
            )
            omni.kit.commands.execute("AddReference", stage=stage, prim_path=Sdf.Path(prim_path), reference=reference)

    def _move_art_b_to_local_pos(self, a_attach_path, attach_robot_path, rel_offset, rel_orient):
        a_trans, a_orient = XFormPrim(a_attach_path).get_world_pose()

        a_rot = quats_to_rot_matrices(a_orient)
        rel_rot = quats_to_rot_matrices(rel_orient)

        b_translation = a_rot @ rel_offset + a_trans
        b_rot = a_rot @ rel_rot
        b_orient = rot_matrices_to_quats(b_rot)

        XFormPrim(attach_robot_path).set_world_pose(b_translation, b_orient)
