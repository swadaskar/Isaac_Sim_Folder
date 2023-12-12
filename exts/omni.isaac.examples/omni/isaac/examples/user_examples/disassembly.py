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
from pxr import UsdLux, Sdf, UsdGeom, Gf
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control

from omni.isaac.universal_robots import KinematicsSolver
import carb
from collections import deque, defaultdict

from omni.physx.scripts import utils

import time

class CustomDifferentialController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.125
        self._wheel_base = 1.152
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[3] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
    
    def turn(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = ((2 * command[0][0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0][1]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0][2]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[3] = ((2 * command[0][3]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)


class RobotsPlaying(BaseTask):
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
    
        asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/photos/real_microfactory_show_without_robots_l.usd"

        
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


class Disassembly(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.done = False
        return

    def setup_scene(self):
        world = self.get_world()
        
        # adding light
        l = UsdLux.SphereLight.Define(world.stage, Sdf.Path("/World/Lights"))
        # l.CreateExposureAttr(...)
        l.CreateColorTemperatureAttr(10000)
        l.CreateIntensityAttr(7500)
        l.CreateRadiusAttr(75)
        l.AddTranslateOp()
        XFormPrim("/World/Lights").set_local_pose(translation=np.array([0,0,100]))

        world.add_task(RobotsPlaying(name="awesome_task"))
        self.isDone = [False]*120
        self.bool_done = [False]*120
        self.motion_task_counter=0

        
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        # bring in moving platforms 
        self.moving_platform = self._world.scene.get_object(task_params["mp_name"]["value"])
        # add main cover on table
        self.add_part_custom("World","main_cover", "ymain_cover", np.array([0.001,0.001,0.001]), np.array([-18.7095, -15.70872, 0.28822]), np.array([0.70711, 0.70711,0,0]))

        # add parts
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","FFrame", "FFrame", np.array([1,1,1]), np.array([-5.82832, 0, 0]), np.array([0.5,0.5,0.5,0.5]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","engine_no_rigid", "engine", np.array([1,1,1]), np.array([311.20868, 598.91546, 181.74458]), np.array([0.7,-0.1298,-0.10842,-0.69374]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","FSuspensionFront", "FSuspensionFront", np.array([1,1,1]), np.array([721.62154, 101.55373, 215.10539]), np.array([0.5564, -0.43637, -0.43637, 0.5564]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","FSuspensionFront", "FSuspensionFront_01", np.array([1,1,1]), np.array([797.10001, 103.5, 422.80001]), np.array([0.44177, -0.55212, -0.55212, 0.44177]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","FSuspensionBack", "FSuspensionBack", np.array([1,1,1]), np.array([443.3, 1348.4, 462.9]), np.array([0.09714, -0.03325, -0.76428, 0.63666]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","battery", "battery", np.array([1,1,1]), np.array([377.63, 788.49883, 270.90454]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","fuel", "fuel", np.array([1,1,1]), np.array([230.13538, 289.31525, 377.64262]), np.array([0.5,0.5,0.5,0.5]))
        # self.add_part_custom("mobile_platform/platform/unfinished_stuff","main_cover", "main_cover", np.array([1,1,1]), np.array([596.47952, 1231.52815, -151.59531]), np.array([0.51452, 0.48504, -0.51452, -0.48504]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","lower_cover", "lower_cover", np.array([1,1,1]), np.array([435.65021, 418.57531,21.83379]), np.array([0.50942, 0.50942,0.4904, 0.4904]))
        self.add_part_custom("mobile_platform/platform/unfinished_stuff","lower_cover", "lower_cover_01", np.array([1,1,1]), np.array([36.473, 415.8277, 25.66846]), np.array([0.5,0.5, 0.5, 0.5]))
        # self.add_part_custom("mobile_platform/platform/unfinished_stuff","Seat", "Seat", np.array([1,1,1]), np.array([179.725, 448.55839, 383.33431]), np.array([0.5,0.5, 0.5, 0.5]))

        self.add_part_custom(f"mobile_platform/platform/unfinished_stuff","FWheel", f"wheel_03", np.array([1,1,1]), np.array([0.15255, -0.1948, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))
        self.add_part_custom(f"mobile_platform/platform/unfinished_stuff","FWheel", f"wheel_01", np.array([1,1,1]), np.array([0.1522, 0.33709, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))
        self.add_part_custom(f"mobile_platform/platform/unfinished_stuff","FWheel", f"wheel_04", np.array([1,1,1]), np.array([-0.80845, -0.22143, 0.43737]), np.array([0.5, -0.5, 0.5, -0.5]))
        self.add_part_custom(f"mobile_platform/platform/unfinished_stuff","FWheel", f"wheel_02", np.array([1,1,1]), np.array([-0.80934, 0.35041, 0.43888]), np.array([0.5, -0.5, 0.5, -0.5]))

        # self.moving_platform = self._world.scene.get_object("moving_platform")
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        self._my_custom_controller = CustomDifferentialController()
        self._my_controller = WheelBasePoseController(name="cool_controller", open_loop_wheel_controller=DifferentialController(name="simple_control", wheel_radius=0.125, wheel_base=0.46), is_holonomic=False)

        self.ur10 = self._world.scene.get_object(task_params["arm_name"]["value"])
        self.my_controller = KinematicsSolver(self.ur10, attach_gripper=True)
        self.articulation_controller = self.ur10.get_articulation_controller()


        # add ee cover on robot
        # self.add_part_custom("World/UR10/ee_link","main_cover", "main_cover", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5,0.5, -0.5, 0.5]))

        # human body movements declaration
        # self.set_new_transform("/World/male_human_01/Character_Source/Root/Root/Pelvis/Spine1", [0, 7.16492, -0.03027], [-0.68407, 0.6840694, 0.1790224, -0.1790201])
        # self.set_new_transform("/World/male_human/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/L_Clavicle/L_UpArm", [18.5044, 0.00001, 0], [0.5378302, -0.2149462, 0.791071, 0.1968338])
        # self.set_new_transform("/World/male_human/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/R_Clavicle/R_UpArm", [-18.50437, -0.00021, -0.00001], [0.5108233, -0.1681133, 0.8128292, 0.223844])

        # for i in range(4):
        #     if i==0:
        #         self.set_new_transform("/World/male_human/Character_Source/Root/Root/Pelvis/Spine1", [0, 7.16492, -0.03027], [-0.68407, 0.6840694, 0.1790224, -0.1790201])
        #         self.set_new_transform("/World/male_human/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/L_Clavicle/L_UpArm", [18.5044, 0.00001, 0], [0.5378302, -0.2149462, 0.791071, 0.1968338])
        #         self.set_new_transform("/World/male_human/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/R_Clavicle/R_UpArm", [-18.50437, -0.00021, -0.00001], [0.5108233, -0.1681133, 0.8128292, 0.223844])
        #     else:
        #         self.set_new_transform("/World/male_human"+f"_0{i}"+"/Character_Source/Root/Root/Pelvis/Spine1", [0, 7.16492, -0.03027], [-0.68407, 0.6840694, 0.1790224, -0.1790201])
        #         self.set_new_transform("/World/male_human"+f"_0{i}"+"/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/L_Clavicle/L_UpArm", [18.5044, 0.00001, 0], [0.5378302, -0.2149462, 0.791071, 0.1968338])
        #         self.set_new_transform("/World/male_human"+f"_0{i}"+"/Character_Source/Root/Root/Pelvis/Spine1/Spine2/Spine3/Chest/R_Clavicle/R_UpArm", [-18.50437, -0.00021, -0.00001], [0.5108233, -0.1681133, 0.8128292, 0.223844])
        return

    async def setup_post_reset(self):
        self._my_controller.reset()
        await self._world.play_async()
        return
    
    def set_new_transform(self, prim_path, translation, orientation):
        dc = _dynamic_control.acquire_dynamic_control_interface()

        # Get the prim you want to move
        prim = dc.get_rigid_body(prim_path)

        # Set the new location
        new_location = Gf.Vec3f(translation[0], translation[1], translation[2])  # Replace with your desired location
        new_rotation = Gf.Rotation(orientation[0], orientation[1], orientation[2], orientation[3])  # Replace with your desired rotation

        # Create a new transform
        new_transform = UsdGeom.TransformAPI(prim)
        new_transform.SetTransform(UsdGeom.XformOp.Transform(Gf.Matrix4d(new_rotation.GetMatrix(), new_location)))

        # Apply the new transform
        dc.set_rigid_body_pose(prim, new_transform.GetLocalTransformation())               

    def give_location(self, prim_path):
        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body(prim_path)
        object_pose=dc.get_rigid_body_pose(object)
        return object_pose # position: object_pose.p, rotation: object_pose.r       
        
    def move_ur10(self, locations):
        
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        actions, success = self.my_controller.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        print(actions)
        if success:
            print("still homing on this location")
            self.articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        # check if reached location
        curr_location = self.give_location("/World/UR10/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.05:
            self.motion_task_counter+=1
            print("Completed one motion plan: ", self.motion_task_counter)
    
    def send_robot_actions(self, step_size):
        current_observations = self._world.get_observations()
        task_params = self._world.get_task("awesome_task").get_params()

        ## Task event numbering:
        # 1 - 30 normal events: forward, stop and add piece, turn
        # 51 - 61 smaller moving platforms events: forward, stop, disappear piece
        # 71 - pick place tasks
        
        if current_observations["task_event"] == 0:
            print("task 0")
            print(current_observations["task_event"])
            if not self.isDone[current_observations["task_event"]]:
                self.isDone[current_observations["task_event"]]=True


        if current_observations["task_event"] == 21:
            # self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]]:
                motion_plan = [{"index":0, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":1, "position": np.array([-1.09352, -0.27789, 0.19772-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444, -15.97435, 1.01392]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":2, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                               {"index":3, "position": np.array([-0.89016, 0.32513, 0.51038-0.16]), "orientation": np.array([0.60698, -0.36274, 0.60698, 0.36274]), "goal_position":np.array([-17.41105, -16.1764, 1.33072]), "goal_orientation":np.array([0.68569, 0.1727, 0.68569, -0.1727])},
                               {"index":4, "position": np.array([-0.74286, 0.42878, 0.51038-0.16]), "orientation": np.array([0.6511, -0.2758, 0.6511, 0.2758]), "goal_position":np.array([-17.30707, -16.32381, 1.33072]), "goal_orientation":np.array([0.65542, 0.26538, 0.65542, -0.26538])},
                               {"index":5, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
                               {"index":6, "position": np.array([-0.28875, 0.74261, 0.51038-0.16]), "orientation": np.array([0.70458, -0.0597, 0.70458, 0.0597]), "goal_position":np.array([-16.99268, -16.77844, 1.33072]), "goal_orientation":np.array([0.54043, 0.456, 0.54043, -0.456])},
                                
                               {"index":7, "position": np.array([0.11095, 0.94627, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":8, "position": np.array([0.11095, 0.94627, 0.2926-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 1.11226]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":9, "position": np.array([0.11095, 0.94627, 0.19682-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 1.01648]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":10, "position": np.array([0.11095, 0.94627, 0.15697-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 0.97663]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":11, "position": np.array([0.11095, 0.94627, 0.11895-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 0.93861]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":12, "position": np.array([0.11095, 0.94627, 0.07882-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 0.89848]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},

                               {"index":13, "position": np.array([0.11095, 0.94627, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175, -17.17995, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":14, "position": np.array([-0.28875, 0.74261, 0.51038-0.16]), "orientation": np.array([0.70458, -0.0597, 0.70458, 0.0597]), "goal_position":np.array([-16.99268, -16.77844, 1.33072]), "goal_orientation":np.array([0.54043, 0.456, 0.54043, -0.456])},
                               {"index":15, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
                               {"index":16, "position": np.array([-0.74286, 0.42878, 0.51038-0.16]), "orientation": np.array([0.6511, -0.2758, 0.6511, 0.2758]), "goal_position":np.array([-17.30707, -16.32381, 1.33072]), "goal_orientation":np.array([0.65542, 0.26538, 0.65542, -0.26538])},
                               {"index":17, "position": np.array([-0.89016, 0.32513, 0.51038-0.16]), "orientation": np.array([0.60698, -0.36274, 0.60698, 0.36274]), "goal_position":np.array([-17.41105, -16.1764, 1.33072]), "goal_orientation":np.array([0.68569, 0.1727, 0.68569, -0.1727])},
                               
                               {"index":18, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])}]
                self.move_ur10(motion_plan)

                # remove world main cover and add ee main cover
                if self.motion_task_counter==2 and not self.bool_done[20]:
                    self.bool_done[20] = True
                    self.remove_part_custom("World", "ymain_cover")
                    self.add_part_custom("World/UR10/ee_link","main_cover", "main_cover", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5,0.5, -0.5, 0.5]))

                # remove ee main cover and add mobile platform main cover
                if self.motion_task_counter==13 and not self.bool_done[21]:
                    self.bool_done[21] = True
                    self.remove_part_custom("World/UR10/ee_link", "main_cover")
                    self.add_part_custom("mobile_platform/platform/unfinished_stuff","main_cover", "xmain_cover", np.array([1,1,1]), np.array([596.47952, 1231.52815, -151.59531]), np.array([0.51452, 0.48504, -0.51452, -0.48504]))

                if self.motion_task_counter==19:
                    print("Done motion plan")
                    self.isDone[current_observations["task_event"]]=True

        # delay
        elif current_observations["task_event"] == 60:
            # self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
            print("task 60 delay:", current_observations["delay"])

        # iteration 0
        # go forward
        elif current_observations["task_event"] == 11:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
            print("task 11")
        # turns
        elif current_observations["task_event"] == 12:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            print("task 12")
        # go forward
        elif current_observations["task_event"] == 13:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
            print("task 13")
        # turn
        elif current_observations["task_event"] == 14:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            print("task 14")

        # iteration 1 
        # go forward
        elif current_observations["task_event"] == 1:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
            print("task 1")
        # turn
        elif current_observations["task_event"] == 2:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            print("task 2")
        # go forward
        elif current_observations["task_event"] == 3:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
            print("task 3")

        # stop, remove and add part
        elif current_observations["task_event"] == 10:

            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("engine")
                # prims.delete_prim("/mobile_platform/platform/engine")
                self.add_part_custom("World/Environment/disassembled_parts","engine_no_rigid", "qengine", np.array([1,1,1]), np.array([-19511.01549, 16368.42662, 451.93828]), np.array([0.99362,0,-0.11281,0]))
                self.isDone[current_observations["task_event"]]=True
        
        # remove and add part
        elif current_observations["task_event"] == 9:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("FSuspensionFront")
                self.remove_part("FSuspensionFront_01")
                self.remove_part("FSuspensionBack")
                # self.add_part_custom("World/Environment/disassembled_parts","FSuspensionFront", "suspension_front", np.array([1,1,1]), np.array([-21252.07916, 16123.85255, -95.12237]), np.array([0.68988,-0.15515,0.17299,0.68562]))
                # self.add_part_custom("World/Environment/disassembled_parts","FSuspensionFront", "suspension_front_1", np.array([1,1,1]), np.array([-21253.69102, 15977.78923, -98.17269]), np.array([0.68988,-0.15515,0.17299,0.68562]))
                self.add_part_custom("World/Environment/disassembled_parts","FSuspensionBack", "suspension_back", np.array([1,1,1]), np.array([-21250.02819, 16296.28288, -79.20706]), np.array([0.69191,-0.16341,0.16465,0.6837]))
                self.isDone[current_observations["task_event"]]=True
        
        # remove and add part
        elif current_observations["task_event"] == 7:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("fuel")
                self.add_part_custom("World/Environment/disassembled_parts","FFuelTank", "FFuelTank", np.array([1,1,1]), np.array([-23473.90962, 16316.56526, 266.94776]), np.array([0.5,0.5,0.5,0.5]))
                self.isDone[current_observations["task_event"]]=True

        # remove and add part
        elif current_observations["task_event"] == 8:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("battery")
                self.add_part_custom("World/Environment/disassembled_parts","battery", "qbattery", np.array([1,1,1]), np.array([-18656.70206, 19011.82244, 283.39021]), np.array([0.5,0.5,0.5,0.5]))
                self.isDone[current_observations["task_event"]]=True

        # remove and add part
        elif current_observations["task_event"] == 6:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("wheel_01")
                self.remove_part("wheel_02")
                self.remove_part("wheel_03")
                self.remove_part("wheel_04")
                self.add_part_custom("World/Environment/disassembled_parts","FWheel", "qwheel_01", np.array([1,1,1]), np.array([-18656.70206, 19011.82244, 283.39021]), np.array([0.5,0.5,0.5,0.5]))
                self.add_part_custom("World/Environment/disassembled_parts","FWheel", "qwheel_02", np.array([1,1,1]), np.array([-18656.70206, 19011.82244, 283.39021]), np.array([0.5,0.5,0.5,0.5]))
                self.add_part_custom("World/Environment/disassembled_parts","FWheel", "qwheel_03", np.array([1,1,1]), np.array([-18656.70206, 19011.82244, 283.39021]), np.array([0.5,0.5,0.5,0.5]))
                self.add_part_custom("World/Environment/disassembled_parts","FWheel", "qwheel_04", np.array([1,1,1]), np.array([-18656.70206, 19011.82244, 283.39021]), np.array([0.5,0.5,0.5,0.5]))
                self.isDone[current_observations["task_event"]]=True
        
        # remove and add part
        elif current_observations["task_event"] == 5:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("lower_cover")
                self.remove_part("lower_cover_01")
                self.add_part_custom("World/Environment/disassembled_parts","lower_cover", "qlower_cover", np.array([1,1,1]), np.array([-20141.19845, 18913.49002, 272.27849]), np.array([0.5,0.5,0.5,0.5]))
                self.add_part_custom("World/Environment/disassembled_parts","lower_cover", "qlower_cover_1", np.array([1,1,1]), np.array([-21228.07681, 18923.38351, 272.27849]), np.array([0.5,0.5,0.5,0.5]))
                self.isDone[current_observations["task_event"]]=True

        # remove and add part
        elif current_observations["task_event"] == 4:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0, 0]))
            if not self.isDone[current_observations["task_event"]] and current_observations["delay"]==100:
                self.remove_part("xmain_cover")
                self.add_part_custom("World/Environment/disassembled_parts","main_cover", "qmain_cover", np.array([1,1,1]), np.array([-23651.85127, 19384.76572, 102.60316]), np.array([0.70428,0.70428,0.06314,-0.06314]))
                self.isDone[current_observations["task_event"]]=True

        
        
        elif current_observations["task_event"] == 11:
            print("delay")
        
        elif current_observations["task_event"] == 12:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5, 0]))
        
        return
    
    def add_part(self, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/mock_robot/platform/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/mock_robot/platform/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)

    def add_part_world(self, parent_prim_name, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/{parent_prim_name}/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/{parent_prim_name}/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_world_pose(position=position, orientation=orientation)
        return part

    def add_part_custom(self, parent_prim_name, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/{parent_prim_name}/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/{parent_prim_name}/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
        return part
    
    def add_part_without_parent(self, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/World/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/World/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
        return part

    def remove_part(self, prim_path):
        prims.delete_prim("/mobile_platform/platform/unfinished_stuff/"+prim_path)

    def remove_part_custom(self, parent_prim_name, child_prim_name):

        prim_path = f"/{parent_prim_name}/{child_prim_name}"
        # world = self.get_world()

        prims.delete_prim(prim_path)
    
    
    def move(self, task):
        mp = self._world.get_observations()[self._world.get_task("awesome_task").get_params()["mp_name"]["value"]]
        print(mp)
        position, orientation, goal_position = mp['position'], mp['orientation'], mp['goal_position'][task-1]
        # In the function where you are sending robot commands
        print(goal_position)
        action = self._my_controller.forward(start_position=position, start_orientation=orientation, goal_position=goal_position)  # Change the goal position to what you want
        full_action = ArticulationAction(joint_efforts=np.concatenate([action.joint_efforts, action.joint_efforts]) if action.joint_efforts else None, joint_velocities=np.concatenate([action.joint_velocities, action.joint_velocities]), joint_positions=np.concatenate([action.joint_positions, action.joint_positions]) if action.joint_positions else None)
        self.moving_platform.apply_action(full_action)
