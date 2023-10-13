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

class FuelTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        # self.mp_goal_position = [np.array([-5.40137, 5.54515, 0.03551]), np.array([-5.40137, -4.88, 0.03551]), np.array([-5.40137, -17.69, 0.03551]), np.array([-20.45, -17.69, 0.03551]), np.array([-36.03, -17.69, 0.03551]), np.array([-36.03, -4.71, 0.03551]), np.array([-20.84, -4.71, 0.03551]), np.array([-20.84, 7.36, 0.03551]), np.array([-36.06, 7.36, 0.03551]), np.array([-36.06, 19.64, 0.03551]), np.array([-5.40137, 19.64, 0.03551])]
        # self.mp_goal_position = [np.array([-5.40137, 5.54515, 0.03551]), np.array([-5.40137, -4.88, 0.03551]), np.array([-5.40137, -17.69, 0.03551]), np.array([-20.45, -17.69, 0.03551]), np.array([-36.03, -17.69, 0.03551]), np.array([-36.03, -4.71, 0.03551]), np.array([-20.84, -4.71, 0.03551]), np.array([-20.84, 7.36, 0.03551]), np.array([-36.06, 7.36, 0.03551]), np.array([-36.06, 19.64, 0.03551]), np.array([-5.40137, 19.64, 0.03551])]
        self.mp_goal_position = [np.array([-5.40137, 5.54515, 0.03551]),
                        np.array([-5.40137, -2.628, 0.03551]),
                        np.array([-5.40137, -15.69, 0.03551]),
                        np.array([-20.45, -17.69, 0.03551]),
                        np.array([-36.03, -17.69, 0.03551]),
                        np.array([-36.03, -4.71, 0.03551]),
                        np.array([-20.84, -4.71, 0.03551]),
                        np.array([-20.84, 7.36, 0.03551]),
                        np.array([-36.06, 7.36, 0.03551]),
                        np.array([-36.06, 19.64, 0.03551]),
                        np.array([-5.40137, 19.64, 0.03551])]

        self.eb_goal_position = np.array([-4.39666, 7.64828, 0.035])

        self.ur10_fuel_goal_position = np.array([])

        # self.mp_goal_orientation = np.array([1, 0, 0, 0])
        self._task_event = 0
        self.task_done = [False]*120

        self.motion_event = 0
        self.motion_done = [False]*120

        self._bool_event = 0
        self.count=0
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        
        assets_root_path = get_assets_root_path() # http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1
    
        asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/photos/real_microfactory_1_2.usd"

        robot_arm_path = assets_root_path + "/Isaac/Robots/UR10_fuel/ur10_fuel.usd"

        # adding UR10_fuel for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_fuel")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_fuel/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_fuel/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_fuel/ee_link", translate=0.1611, direction="x")
        self.ur10_fuel = scene.add(
            SingleManipulator(prim_path="/World/UR10_fuel", name="my_ur10_fuel", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-6.09744, -16.5124, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_fuel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_fuel for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_fuel")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_fuel/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_fuel/ee_link", translate=0, direction="x")
        self.screw_ur10_fuel = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_fuel", name="my_screw_ur10_fuel", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-4.02094, -16.52902, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_fuel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        large_robot_asset_path = small_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform/mobile_platform.usd"
        # add floor
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Environment")

        # add moving platform
        self.moving_platform = scene.add(
            WheeledRobot(
                prim_path="/mock_robot",
                name="moving_platform",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=large_robot_asset_path,
                position=np.array([-5.26025, -9.96157, 0.03551]),
                orientation=np.array([0.5, 0.5, -0.5, -0.5]),
            )
        )

        self.engine_bringer = scene.add(
            WheeledRobot(
                prim_path="/engine_bringer",
                name="engine_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-7.47898, -16.15971, 0.035]),
                orientation=np.array([0,0,0.70711, 0.70711]),
            )
        )

        return

    def get_observations(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        current_eb_position, current_eb_orientation = self.engine_bringer.get_world_pose()
        current_joint_positions_ur10_fuel = self.ur10_fuel.get_joint_positions()
        observations= {
            "task_event": self._task_event,
            "task_event": self._task_event,
            self.moving_platform.name: {
                "position": current_mp_position,
                "orientation": current_mp_orientation,
                "goal_position": self.mp_goal_position
            },
            self.engine_bringer.name: {
                "position": current_eb_position,
                "orientation": current_eb_orientation,
                "goal_position": self.eb_goal_position
            },
            self.ur10_fuel.name: {
                "joint_positions": current_joint_positions_ur10_fuel,
            },
            self.screw_ur10_fuel.name: {
                "joint_positions": current_joint_positions_ur10_fuel,
            },
            "bool_counter": self._bool_event
        }
        return observations

    def get_params(self):
        params_representation = {}
        params_representation["arm_name_fuel"] = {"value": self.ur10_fuel.name, "modifiable": False}
        params_representation["screw_arm_fuel"] = {"value": self.screw_ur10_fuel.name, "modifiable": False}
        params_representation["mp_name"] = {"value": self.moving_platform.name, "modifiable": False}
        params_representation["eb_name_fuel"] = {"value": self.engine_bringer.name, "modifiable": False}
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
        ee_pose = self.give_location("/World/UR10_fuel/ee_link")
        screw_ee_pose = self.give_location("/World/Screw_driving_UR10_fuel/ee_link")


        # iteration 1
        if self._task_event == 0:
            if self.task_done[self._task_event]:
                self._task_event += 1
            self.task_done[self._task_event] = True
        elif self._task_event == 1:
            if self.task_done[self._task_event] and current_mp_position[1]<-16.69:
                self._task_event = 51
                self._bool_event+=1
            self.task_done[self._task_event] = True

        elif self._task_event == 51:
            if np.mean(np.abs(ee_pose.p - np.array([-5.005, -16.7606, 0.76714])))<0.02:
                self._task_event = 71
                self._bool_event+=1
        
        elif self._task_event == 71:
            if np.mean(np.abs(ee_pose.p - np.array([-4.18372, 7.03628, 0.44567])))<0.058:
                self._task_event=2
            pass
        
        elif self._task_event == 2:
            pass
        return

    def post_reset(self):
        self._task_event = 0
        return