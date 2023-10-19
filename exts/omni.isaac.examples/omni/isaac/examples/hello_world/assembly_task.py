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

class AssemblyTask(BaseTask):
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

        self.ur10_goal_position = np.array([])

        # self.mp_goal_orientation = np.array([1, 0, 0, 0])
        self._task_event = 0
        self.task_done = [False]*1000

        self.motion_event = 0
        self.motion_done = [False]*1000

        self._bool_event = 0
        self.count=0
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        
        assets_root_path = get_assets_root_path() # http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1
    
        asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/photos/real_microfactory_show_without_robots.usd"

        robot_arm_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"

        # adding UR10 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/version3_j_hook/version3_j_hook.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
        self.ur10 = scene.add(
            SingleManipulator(prim_path="/World/UR10", name="my_ur10", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-4.98573, 6.97238, 0.24168]), orientation=np.array([0.70711, 0, 0, 0.70711]), scale=np.array([1,1,1]))
        )
        self.ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10/ee_link", translate=0, direction="x")
        self.screw_ur10 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10", name="my_screw_ur10", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-4.73889, 4.66511, 0.24168]), orientation=np.array([0.70711, 0, 0, 0.70711]), scale=np.array([1,1,1]))
        )
        self.screw_ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # Floorplan = 0, -4.26389, 0
        # Station_ = 1.50338, -4.95641, 0
        

        # large_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform/mobile_platform1.usd"
        
        # small_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform/mobile_platform1.usd"

        large_robot_asset_path = small_robot_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/Collected_full_warehouse_microfactory/Collected_mobile_platform_improved/Collected_mobile_platform_unfinished/mobile_platform_flattened.usd"
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
                # position=np.array([2.5, 5.65, 0.03551]),  orientation=np.array([0,0,0,1]), # start position
                # position=np.array([-4.78521, -10.1757,0.03551]), orientation=np.array([0.70711, 0, 0, -0.70711]),# initial before fuel cell
                # position=np.array([-9.60803, -17.35671, 0.03551]), orientation=np.array([0, 0, 0, 1]),# initial before battery cell
                position=np.array([-32.5, 3.516, 0.03551]), orientation=np.array([0.70711, 0, 0, 0.70711]),# initial before trunk cell
                # orientation=np.array([0,0,0,1]), # start orientation
            )
        )

        # # Second view from contingency 
        # self.moving_platform = scene.add(
        #     WheeledRobot(
        #         prim_path="/mock_robot",
        #         name="moving_platform",
        #         wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
        #         create_robot=True,
        #         usd_path=large_robot_asset_path,
        #         position=np.array([-3.28741, 10.79225, 0.03551]),
        #         orientation=np.array([0.5, 0.5,-0.5, -0.5]),
        #     )
        # )

        self.engine_bringer = scene.add(
            WheeledRobot(
                prim_path="/engine_bringer",
                name="engine_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-6.919, 7.764, 0.03551]),
                orientation=np.array([0.70711,0, 0,-0.70711]),
            )
        )


        # Suspension task assembly ---------------------------------------------
        # adding UR10_suspension for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_suspension")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_suspension/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_suspension/ee_link", translate=0.1611, direction="x")
        self.ur10_suspension = scene.add(
            SingleManipulator(prim_path="/World/UR10_suspension", name="my_ur10_suspension", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-6.10078, -5.19303, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_suspension.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_suspension for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_suspension")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_suspension/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_suspension/ee_link", translate=0, direction="x")
        self.screw_ur10_suspension = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_suspension", name="my_screw_ur10_suspension", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-3.78767, -5.00871, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_suspension.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.suspension_bringer = scene.add(
            WheeledRobot(
                prim_path="/suspension_bringer",
                name="suspension_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-7.60277, -5.70312, 0.035]),
                orientation=np.array([0.70711,0, 0,-0.70711]),
            )
        )

        # Fuel task assembly --------------------------------------------------
        # adding UR10_fuel for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_fuel")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_fuel/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_fuel/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_fuel/ee_link", translate=0.1611, direction="x")
        self.ur10_fuel = scene.add(
            SingleManipulator(prim_path="/World/UR10_fuel", name="my_ur10_fuel", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-6.09744, -16.16324, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_fuel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_fuel for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_fuel")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_fuel/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_fuel/ee_link", translate=0, direction="x")
        self.screw_ur10_fuel = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_fuel", name="my_screw_ur10_fuel", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-4.24592, -15.9399, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_fuel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.fuel_bringer = scene.add(
            WheeledRobot(
                prim_path="/fuel_bringer",
                name="fuel_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-2.4, -16.15971, 0.035]),
                orientation=np.array([0.70711,0, 0,-0.70711]),
            )
        )


        # battery task assembly --------------------------------------------------
        # adding UR10_battery for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_battery")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_battery/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_battery/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_battery/ee_link", translate=0.1611, direction="x")
        self.ur10_battery = scene.add(
            SingleManipulator(prim_path="/World/UR10_battery", name="my_ur10_battery", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-16.55392, -16.32998, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_battery.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_battery for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_battery")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_battery/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_battery/ee_link", translate=0, direction="x")
        self.screw_ur10_battery = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_battery", name="my_screw_ur10_battery", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-16.35174, -18.16947, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_battery.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.battery_bringer = scene.add(
            WheeledRobot(
                prim_path="/battery_bringer",
                name="battery_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-18.37412, -13.44433, 0.035]),
                orientation=np.array([0.70711,0, 0,-0.70711]),
            )
        )

        # trunk task assembly --------------------------------------------------
        # adding UR10_trunk for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_trunk")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_trunk/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_trunk/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_trunk/ee_link", translate=0.1611, direction="x")
        self.ur10_trunk = scene.add(
            SingleManipulator(prim_path="/World/UR10_trunk", name="my_ur10_trunk", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-27.1031, 4.48605, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_trunk.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_trunk for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_trunk")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_trunk/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_trunk/ee_link", translate=0, direction="x")
        self.screw_ur10_trunk = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_trunk", name="my_screw_ur10_trunk", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-27.29981, 6.5455, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_trunk.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.trunk_bringer = scene.add(
            WheeledRobot(
                prim_path="/trunk_bringer",
                name="trunk_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-30.518, 3.516, 0.035]),
                orientation=np.array([1,0,0,0]),
            )
        )

        # wheel task assembly --------------------------------------------------
        # adding UR10_wheel for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_wheel")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_wheel/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_wheel/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_wheel/ee_link", translate=0.1611, direction="x")
        self.ur10_wheel = scene.add(
            SingleManipulator(prim_path="/World/UR10_wheel", name="my_ur10_wheel", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-27.1031, 4.48605, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_wheel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_wheel for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_wheel")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_wheel/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_wheel/ee_link", translate=0, direction="x")
        self.screw_ur10_wheel = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_wheel", name="my_screw_ur10_wheel", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-27.29981, 6.5455, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_wheel.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_wheel_01 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_wheel_01")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_wheel_01/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_wheel_01/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_wheel_01/ee_link", translate=0.1611, direction="x")
        self.ur10_wheel_01 = scene.add(
            SingleManipulator(prim_path="/World/UR10_wheel_01", name="my_ur10_wheel_01", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-27.1031, 4.48605, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_wheel_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_wheel_01 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_wheel_01")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_wheel_01/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_wheel_01/ee_link", translate=0, direction="x")
        self.screw_ur10_wheel_01 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_wheel_01", name="my_screw_ur10_wheel_01", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-27.29981, 6.5455, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_wheel_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.wheel_bringer = scene.add(
            WheeledRobot(
                prim_path="/wheel_bringer",
                name="wheel_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-20.518, 3.516, 0.035]),
                orientation=np.array([1,0,0,0]),
            )
        )

        # main and lower cover task assembly --------------------------------------------------
         # adding UR10_main_cover for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_main_cover")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/Cover_Gripper/Cover_Gripper.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_main_cover/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_main_cover/ee_link", translate=0.1611, direction="x")
        self.ur10_main_cover = scene.add(
            SingleManipulator(prim_path="/World/UR10_main_cover", name="my_ur10_main_cover", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-17.73638,-17.06779, 0.81965]), orientation=np.array([0.70711, 0, 0, -0.70711]), scale=np.array([1,1,1]))
        )
        self.ur10_main_cover.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_lower_cover for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_lower_cover")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_lower_cover/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_lower_cover/ee_link", translate=0.1611, direction="x")
        self.ur10_lower_cover = scene.add(
            SingleManipulator(prim_path="/World/UR10_lower_cover", name="my_ur10_lower_cover", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-14.28725, -16.26194, 0.24133]), orientation=np.array([1,0,0,0]), scale=np.array([1,1,1]))
        )
        self.ur10_lower_cover.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_lower_cover_01 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_lower_cover_01")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_lower_cover_01/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_lower_cover_01/ee_link", translate=0.1611, direction="x")
        self.ur10_lower_cover_01 = scene.add(
            SingleManipulator(prim_path="/World/UR10_lower_cover_01", name="my_ur10_lower_cover_01", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-14.28725, -18.32192, 0.24133]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_lower_cover_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_lower_cover for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_lower_cover")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_lower_cover/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_lower_cover/ee_link", translate=0, direction="x")
        self.screw_ur10_lower_cover = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_lower_cover", name="my_screw_ur10_lower_cover", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-15.71653, -16.26185, 0.24133]), orientation=np.array([1,0,0,0]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_lower_cover.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_lower_cover_01 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_lower_cover_01")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_lower_cover_01/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_lower_cover_01/ee_link", translate=0, direction="x")
        self.screw_ur10_lower_cover_01 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_lower_cover_01", name="my_screw_ur10_lower_cover_01", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-15.7071, -18.31962, 0.24133]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_lower_cover_01.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # handle task assembly --------------------------------------------------
        # adding UR10_handle for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10_handle")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10_handle/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10_handle/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10_handle/ee_link", translate=0.1611, direction="x")
        self.ur10_handle = scene.add(
            SingleManipulator(prim_path="/World/UR10_handle", name="my_ur10_handle", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-27.1031, 4.48605, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10_handle.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10_handle for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10_handle")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10_handle/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10_handle/ee_link", translate=0, direction="x")
        self.screw_ur10_handle = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10_handle", name="my_screw_ur10_handle", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-27.29981, 6.5455, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10_handle.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self.handle_bringer = scene.add(
            WheeledRobot(
                prim_path="/handle_bringer",
                name="handle_bringer",
                wheel_dof_names=["wheel_tl_joint", "wheel_tr_joint", "wheel_bl_joint", "wheel_br_joint"],
                create_robot=True,
                usd_path=small_robot_asset_path,
                position=np.array([-30.518, 3.516, 0.035]),
                orientation=np.array([1,0,0,0]),
            )
        )

        return

    def get_observations(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        current_eb_position, current_eb_orientation = self.engine_bringer.get_world_pose()
        current_joint_positions_ur10 = self.ur10.get_joint_positions()

        current_eb_position_suspension, current_eb_orientation_suspension = self.suspension_bringer.get_world_pose()
        current_joint_positions_ur10_suspension = self.ur10_suspension.get_joint_positions()

        current_eb_position_fuel, current_eb_orientation_fuel = self.fuel_bringer.get_world_pose()
        current_joint_positions_ur10_fuel = self.ur10_fuel.get_joint_positions()

        current_eb_position_battery, current_eb_orientation_battery = self.battery_bringer.get_world_pose()
        current_joint_positions_ur10_battery = self.ur10_battery.get_joint_positions()

        current_eb_position_trunk, current_eb_orientation_trunk = self.trunk_bringer.get_world_pose()
        current_joint_positions_ur10_trunk = self.ur10_trunk.get_joint_positions()
        
        current_eb_position_wheel, current_eb_orientation_wheel = self.wheel_bringer.get_world_pose()
        current_joint_positions_ur10_wheel = self.ur10_wheel.get_joint_positions()
        current_joint_positions_ur10_wheel_01 = self.ur10_wheel_01.get_joint_positions()

        current_eb_position_lower_cover, current_eb_orientation_lower_cover = self.lower_cover_bringer.get_world_pose()
        current_joint_positions_ur10_lower_cover = self.ur10_lower_cover.get_joint_positions()
        current_joint_positions_ur10_main_cover = self.ur10_main_cover.get_joint_positions()
        current_joint_positions_ur10_lower_cover_01 = self.ur10_lower_cover_01.get_joint_positions()

        current_eb_position_handle, current_eb_orientation_handle = self.handle_bringer.get_world_pose()
        current_joint_positions_ur10_handle = self.ur10_handle.get_joint_positions()

        observations= {
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
            self.ur10.name: {
                "joint_positions": current_joint_positions_ur10,
            },
            self.screw_ur10.name: {
                "joint_positions": current_joint_positions_ur10,
            },
            "bool_counter": self._bool_event,
            self.suspension_bringer.name: {
                "position": current_eb_position_suspension,
                "orientation": current_eb_orientation_suspension,
                "goal_position": self.eb_goal_position
            },
            self.ur10_suspension.name: {
                "joint_positions": current_joint_positions_ur10_suspension,
            },
            self.screw_ur10_suspension.name: {
                "joint_positions": current_joint_positions_ur10_suspension,
            },
            self.ur10_fuel.name: {
                "joint_positions": current_joint_positions_ur10_fuel,
            },
            self.screw_ur10_fuel.name: {
                "joint_positions": current_joint_positions_ur10_fuel,
            },
            self.ur10_battery.name: {
                "joint_positions": current_joint_positions_ur10_battery,
            },
            self.screw_ur10_battery.name: {
                "joint_positions": current_joint_positions_ur10_battery,
            },
            self.ur10_trunk.name: {
                "joint_positions": current_joint_positions_ur10_trunk,
            },
            self.screw_ur10_trunk.name: {
                "joint_positions": current_joint_positions_ur10_trunk,
            },
            self.ur10_wheel.name: {
                "joint_positions": current_joint_positions_ur10_wheel,
            },
            self.screw_ur10_wheel.name: {
                "joint_positions": current_joint_positions_ur10_wheel,
            },
            self.ur10_wheel_01.name: {
                "joint_positions": current_joint_positions_ur10_wheel_01,
            },
            self.screw_ur10_wheel_01.name: {
                "joint_positions": current_joint_positions_ur10_wheel_01,
            },
            self.ur10_lower_cover.name: {
                "joint_positions": current_joint_positions_ur10_lower_cover,
            },
            self.screw_ur10_lower_cover.name: {
                "joint_positions": current_joint_positions_ur10_lower_cover,
            },
            self.ur10_lower_cover_01.name: {
                "joint_positions": current_joint_positions_ur10_lower_cover_01,
            },
            self.screw_ur10_lower_cover_01.name: {
                "joint_positions": current_joint_positions_ur10_lower_cover_01,
            },
            self.ur10_main_cover.name: {
                "joint_positions": current_joint_positions_ur10_main_cover,
            },
            self.ur10_handle.name: {
                "joint_positions": current_joint_positions_ur10_handle,
            },
            self.screw_ur10_handle.name: {
                "joint_positions": current_joint_positions_ur10_handle,
            },
        }
        return observations

    def get_params(self):
        params_representation = {}
        params_representation["arm_name"] = {"value": self.ur10.name, "modifiable": False}
        params_representation["screw_arm"] = {"value": self.screw_ur10.name, "modifiable": False}
        params_representation["mp_name"] = {"value": self.moving_platform.name, "modifiable": False}
        params_representation["eb_name"] = {"value": self.engine_bringer.name, "modifiable": False}

        # suspension task
        params_representation["arm_name_suspension"] = {"value": self.ur10_suspension.name, "modifiable": False}
        params_representation["screw_arm_suspension"] = {"value": self.screw_ur10_suspension.name, "modifiable": False}
        params_representation["eb_name_suspension"] = {"value": self.suspension_bringer.name, "modifiable": False}

        # fuel task
        params_representation["arm_name_fuel"] = {"value": self.ur10_fuel.name, "modifiable": False}
        params_representation["screw_arm_fuel"] = {"value": self.screw_ur10_fuel.name, "modifiable": False}
        params_representation["eb_name_fuel"] = {"value": self.fuel_bringer.name, "modifiable": False}
        
        # battery task
        params_representation["arm_name_battery"] = {"value": self.ur10_battery.name, "modifiable": False}
        params_representation["screw_arm_battery"] = {"value": self.screw_ur10_battery.name, "modifiable": False}
        params_representation["eb_name_battery"] = {"value": self.battery_bringer.name, "modifiable": False}
        
        # trunk task
        params_representation["arm_name_trunk"] = {"value": self.ur10_trunk.name, "modifiable": False}
        params_representation["screw_arm_trunk"] = {"value": self.screw_ur10_trunk.name, "modifiable": False}
        params_representation["eb_name_trunk"] = {"value": self.trunk_bringer.name, "modifiable": False}

        # wheel task
        params_representation["arm_name_wheel"] = {"value": self.ur10_wheel.name, "modifiable": False}
        params_representation["screw_arm_wheel"] = {"value": self.screw_ur10_wheel.name, "modifiable": False}
        params_representation["arm_name_wheel_01"] = {"value": self.ur10_wheel_01.name, "modifiable": False}
        params_representation["screw_arm_wheel_01"] = {"value": self.screw_ur10_wheel_01.name, "modifiable": False}
        params_representation["eb_name_wheel"] = {"value": self.wheel_bringer.name, "modifiable": False}
        
        # lower_cover task
        params_representation["arm_name_main_cover"] = {"value": self.ur10_main_cover.name, "modifiable": False}
        params_representation["arm_name_lower_cover"] = {"value": self.ur10_lower_cover.name, "modifiable": False}
        params_representation["screw_arm_lower_cover"] = {"value": self.screw_ur10_lower_cover.name, "modifiable": False}
        params_representation["arm_name_lower_cover_01"] = {"value": self.ur10_lower_cover_01.name, "modifiable": False}
        params_representation["screw_arm_lower_cover_01"] = {"value": self.screw_ur10_lower_cover_01.name, "modifiable": False}
        params_representation["eb_name_lower_cover"] = {"value": self.lower_cover_bringer.name, "modifiable": False}
        
        # handle task
        params_representation["arm_name_handle"] = {"value": self.ur10_handle.name, "modifiable": False}
        params_representation["screw_arm_handle"] = {"value": self.screw_ur10_handle.name, "modifiable": False}
        params_representation["eb_name_handle"] = {"value": self.handle_bringer.name, "modifiable": False}

        
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
        screw_ee_pose = self.give_location("/World/Screw_driving_UR10/ee_link")

        # suspension
        ee_pose_suspension = self.give_location("/World/UR10_suspension/ee_link")
        screw_ee_pose_suspension = self.give_location("/World/Screw_driving_UR10_suspension/ee_link")



        # motion_tasks
        if self.motion_event == 0:
            if ee_pose.p[1]<=6.09 and self.motion_done[self.motion_event]:
                self.motion_event += 1
            self.task_done[self.motion_event] = True
        elif self.motion_event == 0:
            if self.motion_done[self.motion_event]:
                self.motion_event += 1
            self.task_done[self.motion_event] = True

        


        # iteration 1
        if self._task_event == 0:
            if self.task_done[self._task_event]:
                self._task_event += 1
            self.task_done[self._task_event] = True


        # second view tasks
        # elif self._task_event == 21:
        #     if self.task_done[self._task_event] and current_mp_position[1]<6.05:
        #         self._task_event = 22
        #     self.task_done[self._task_event] = True
        # elif self._task_event == 22:
        #     print(np.mean(np.abs(current_mp_orientation-np.array([0,0,0.70711,0.70711]))))
        #     if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([0,0,0.70711, 0.70711]))) < 0.609:
        #         self._task_event =1
        #     self.task_done[self._task_event] = True
        
        elif self._task_event == 1:
            if self.task_done[self._task_event] and current_mp_position[0]<-4.98951:
                self._task_event = 71
                self._bool_event+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 71:
            if np.mean(np.abs(ee_pose.p - np.array([-4.86552, 5.86784+0.06, 0.37552])))<0.01:
                self._bool_event+=1
                self._task_event = 2
                
        
        elif self._task_event == 2:
            if np.mean(np.abs(screw_ee_pose.p - np.array([-4.20, 4.63272, 0.38666])))<0.01:
                self._task_event=72
                self._bool_event+=1
        
        elif self._task_event == 72:
            if np.mean(np.abs(ee_pose.p - np.array([-4.18372, 7.03628, 0.44567])))<0.058:
                self._task_event=3

        # iteration 2
        elif self._task_event == 3:
            print(np.mean(np.abs(current_mp_orientation-np.array([0.70711, 0.70711, 0, 0]))))
            if self.task_done[self._task_event] and np.mean(np.abs(current_mp_orientation-np.array([0.70711, 0.70711, 0, 0]))) < 0.251:
                self._task_event += 1
            self.task_done[self._task_event] = True
        elif self._task_event == 4:
            # pass
            if np.mean(np.abs(screw_ee_pose.p - np.array([-5.42692, 4.82896, 0.88836])))<0.058:
                self._task_event=6
            
        elif self._task_event == 6:
            if self.count >100:
                if self.task_done[self._task_event]:
                    self._task_event =5
                self.count=0
            else:
                self.count+=1
            self.task_done[self._task_event] = True     
        elif self._task_event == 5:
            self._task_event = 101
            pass


        # --------------- Suspension tasks --------------------------
        elif self._task_event == 101:
            if self.task_done[self._task_event] and current_mp_position[0]>2.1:
                self._task_event = 151
                self._bool_event+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 151:
            if np.mean(np.abs(ee_pose_suspension.p - np.array([-5.00127, -4.80822, 0.53949])))<0.02:
                self._task_event = 171
                self._bool_event+=1
        
        elif self._task_event == 171:
            if np.mean(np.abs(screw_ee_pose_suspension.p - np.array([-3.70349, -4.41856, 0.56125])))<0.058:
                self._task_event=102
        elif self._task_event == 102:
            if self.task_done[self._task_event]:
                if self.count == 100:
                    self._task_event =103
                    self.count=0
                else:
                    self.count+=1
            self.task_done[self._task_event] = True
        elif self._task_event == 103:
            pass
        return

    def post_reset(self):
        self._task_event = 0
        return