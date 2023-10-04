from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "denso_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change this to the robot usd file.
        assets_root_path = get_assets_root_path()
        asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/robot_arm_nj.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Kuka")
        #define the gripper
        gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/kuka/tool0")
        # gripper = my_world.scene.add(XFormPrim(prim_path=f'/World/kuka/tool0', name="gripper")) # declares in the world

        # ## add part
        # gripper.set_world_pose(position=np.array([2.715, 0, 0.63385]), orientation=np.array([0.70711, 0, 0, 0]))

        gripper = SurfaceGripper(end_effector_prim_path="/World/kuka/tool0", translate=0, direction="x")
        manipulator = SingleManipulator(prim_path="/World/kuka",
                                        name="kuka",
                                        end_effector_prim_name="tool0",
                                        gripper=gripper)
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator