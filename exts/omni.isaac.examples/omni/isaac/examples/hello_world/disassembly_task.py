from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.universal_robots.controllers.pick_place_controller import PickPlaceController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction
# This extension includes several generic controllers that could be used with multiple robots
from omni.isaac.motion_generation import WheelBasePoseController
# Robot specific controller
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.tasks import BaseTask
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
import numpy as np
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid
from omni.isaac.core.utils import prims
from pxr import UsdLux, Sdf, UsdGeom
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control

from omni.isaac.universal_robots import KinematicsSolver
import carb
from collections import deque, defaultdict
import time

class DisassemblyTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self.mp_goal_position = [np.array([-28.07654, 18.06421, 0]),
                        np.array([-25.04914, 18.06421, 0])]
        self._task_event = 0
        self.task_done = [False]*120
        self.delay = 0
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        
        assets_root_path = get_assets_root_path() # http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1
    
        asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/photos/real_microfactory_1_2.usd"

        
        large_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform_unfinished/mobile_platform_flattened.usd"
        # add floor
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Environment")

        robot_arm_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"

        # adding UR10 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/Cover_Gripper/Cover_Gripper.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
        self.ur10 = scene.add(
            SingleManipulator(prim_path="/World/UR10", name="my_ur10", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-17.73638,-17.06779, 0.81965]), orientation=np.array([0.70711, 0, 0, -0.70711]), scale=np.array([1,1,1]))
        )
        self.ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_02 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_02")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_02/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_02/ee_link", translate=0.1611, direction="x")
        self.ur10_02 = scene.add(
            SingleManipulator(prim_path="/World/UR10_02", name="my_ur10_02", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-14.28725, -16.26194, 0.24133]), orientation=np.array([1,0,0,0]), scale=np.array([1,1,1]))
        )
        self.ur10_02.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_01 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_01")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_01/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_01/ee_link", translate=0.1611, direction="x")
        self.ur10_01 = scene.add(
            SingleManipulator(prim_path="/World/UR10_01", name="my_ur10_01", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-14.28725, -18.32192, 0.24133]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10/ee_link", translate=0, direction="x")
        self.screw_ur10 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10", name="my_screw_ur10", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-15.71653, -16.26185, 0.24133]), orientation=np.array([1,0,0,0]), scale=np.array([1,1,1]))
        )
        self.screw_ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_01 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_01")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_01/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_01/ee_link", translate=0, direction="x")
        self.screw_ur10_01 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_01", name="my_screw_ur10_01", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-15.7071, -18.31962, 0.24133]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))


        # add moving platform
        self.moving_platform = scene.add(
            WheeledRobot(
                prim_path="/mobile_platform",
                name="moving_platform",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=large_robot_asset_path,
                position=np.array([-16.80027, -17.26595, 0]),
                orientation=np.array([0.70711, 0, 0, -0.70711])
            )
        )
        return

    def get_observations(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        current_joint_positions_ur10 = self.ur10.get_joint_positions()
        observations= {
            "task_event": self._task_event,
            "delay": self.delay,
            self.moving_platform.name: {
                "position": current_mp_position,
                "orientation": current_mp_orientation,
                "goal_position": self.mp_goal_position
            },
            self.ur10.name: {
                "joint_positions": current_joint_positions_ur10,
            }
        }
        return observations

    def get_params(self):
        params_representation = {}
        params_representation["arm_name"] = {"value": self.ur10.name, "modifiable": False}
        params_representation["mp_name"] = {"value": self.moving_platform.name, "modifiable": False}
        return params_representation
    
    def check_prim_exists(self, prim):
        if prim:
            return True
        return False
    
    def give_location(self, prim_path):
        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body(prim_path)
        object_pose=dc.get_rigid_body_pose(object)
        return object_pose # position: object_pose.p, rotation: object_pose.r
    

    def pre_step(self, control_index, simulation_time):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        ee_pose = self.give_location("/World/UR10/ee_link")

        # iteration 1
        if self._task_event == 0:
            print(self._task_event)
            self._task_event =21
            if self.task_done[self._task_event]:
                self._task_event =21
            self.task_done[self._task_event] = True

        elif self._task_event == 21:
            if np.mean(np.abs(ee_pose.p-np.array([-18.01444, -15.97435, 1.40075])))<0.01:
                self._task_event = 60
        
        elif self._task_event == 60:
            if self.task_done[self._task_event]:
                if self.delay == 320:
                    self._task_event = 11
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True

        elif self._task_event == 11:
            if self.task_done[self._task_event] and current_mp_position[1]<-24.98:
                self._task_event =12
            self.task_done[self._task_event] = True
        elif self._task_event == 12:
            print(np.mean(np.abs(current_mp_orientation-np.array([0,0,0,-1]))))
            if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([0,0,0,-1]))) < 0.503:
                self._task_event = 13
            self.task_done[self._task_event] = True       
        elif self._task_event == 13:
            if self.task_done[self._task_event] and current_mp_position[0]<-36.3:
                self._task_event =14
            self.task_done[self._task_event] = True
        elif self._task_event == 14:
            print(np.mean(np.abs(current_mp_orientation-np.array([0.70711,0,0,0.70711]))))
            if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([0.70711,0,0,0.70711]))) < 0.0042:
                self._task_event = 1
            self.task_done[self._task_event] = True 

        elif self._task_event == 1:
            if self.task_done[self._task_event] and current_mp_position[1]>17.3:
                self._task_event +=1
            self.task_done[self._task_event] = True
        elif self._task_event == 2:
            print(np.mean(np.abs(current_mp_orientation-np.array([1,0,0,0]))))
            if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([1,0,0,0]))) < 0.008:
                self._task_event += 1
            self.task_done[self._task_event] = True       
        elif self._task_event == 3:
            if self.task_done[self._task_event] and current_mp_position[0]>-20.25:
                self._task_event +=1
            self.task_done[self._task_event] = True
        
        # disassembling parts
        elif self._task_event == 4:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 5:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 6:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 7:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 8:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 9:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True

        elif self._task_event == 10:
            if self.task_done[self._task_event]:
                if self.delay == 100:
                    self._task_event +=1
                    self.delay=0
                else:
                    self.delay+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 11:
            pass


        # iteration 9
        elif self._task_event == 22:
            _, current_mp_orientation = self.moving_platform.get_world_pose()
            if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([0, 0, 0.70711, 0.70711]))) < 0.035:
                self._task_event += 1
            self.task_done[self._task_event] = True
        elif self._task_event == 23:
            current_mp_position, _ = self.moving_platform.get_world_pose()
            if self.task_done[self._task_event] and current_mp_position[0]<-36:
                self._task_event += 1
            self.task_done[self._task_event] = True
        elif self._task_event == 24:
            if self.count >100:
                if self.task_done[self._task_event]:
                    self._task_event += 1
                self.count=0
            else:
                self.count+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 25:
            if self.count >100:
                if self.task_done[self._task_event]:
                    self._task_event += 1
                self.count=0
            else:
                self.count+=1
            self.task_done[self._task_event] = True
        return

    def post_reset(self):
        self._task_event = 0
        return