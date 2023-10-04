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
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
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

        robot_arm_path = assets_root_path + "/Isaac/Robots/UR10/ur10.usd"

        # adding UR10 for pick and place
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/UR10")
        # gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/RG2_v2/RG2_v2.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
        self.ur10 = scene.add(
            SingleManipulator(prim_path="/World/UR10", name="my_ur10", end_effector_prim_name="ee_link", gripper=gripper, translation = np.array([-6.09744, -16.5124, 0.24168]), orientation=np.array([0,0,0,1]), scale=np.array([1,1,1]))
        )
        self.ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        # adding UR10 for screwing in part
        add_reference_to_stage(usd_path=robot_arm_path, prim_path="/World/Screw_driving_UR10")
        gripper_usd = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/robot_tools/screw_driver_link/screw_driver_link.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/Screw_driving_UR10/ee_link")
        screw_gripper = SurfaceGripper(end_effector_prim_path="/World/Screw_driving_UR10/ee_link", translate=0, direction="x")
        self.screw_ur10 = scene.add(
            SingleManipulator(prim_path="/World/Screw_driving_UR10", name="my_screw_ur10", end_effector_prim_name="ee_link", gripper=screw_gripper, translation = np.array([-4.02094, -16.52902, 0.24168]), orientation=np.array([0, 0, 0, 1]), scale=np.array([1,1,1]))
        )
        self.screw_ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

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
        current_joint_positions_ur10 = self.ur10.get_joint_positions()
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
            self.ur10.name: {
                "joint_positions": current_joint_positions_ur10,
            },
            self.screw_ur10.name: {
                "joint_positions": current_joint_positions_ur10,
            },
            "bool_counter": self._bool_event
        }
        return observations

    def get_params(self):
        params_representation = {}
        params_representation["arm_name"] = {"value": self.ur10.name, "modifiable": False}
        params_representation["screw_arm"] = {"value": self.screw_ur10.name, "modifiable": False}
        params_representation["mp_name"] = {"value": self.moving_platform.name, "modifiable": False}
        params_representation["eb_name"] = {"value": self.engine_bringer.name, "modifiable": False}
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


class RoboFactory(BaseSample):
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
        self.delay=0
        print("inside setup_scene", self.motion_task_counter)
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        # bring in moving platforms 
        self.moving_platform = self._world.scene.get_object(task_params["mp_name"]["value"])
        self.engine_bringer = self._world.scene.get_object(task_params["eb_name"]["value"])
        
        self.add_part_custom("engine_bringer/platform","fuel", "fuel", np.array([0.001,0.001,0.001]), np.array([-0.1769, 0.13468, 0.24931]), np.array([0.5,0.5,-0.5,-0.5]))
        self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.90761, 0.03096, 0.69056]), np.array([0.48732, -0.51946, 0.50085, -0.49176]))
        self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
        self.add_part("FFrame", "frame", np.array([0.001, 0.001, 0.001]), np.array([0.45216, -0.32084, 0.28512]), np.array([0, 0, 0.70711, 0.70711]))

        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        self._my_custom_controller = CustomDifferentialController()
        self._my_controller = WheelBasePoseController(name="cool_controller", open_loop_wheel_controller=DifferentialController(name="simple_control", wheel_radius=0.125, wheel_base=0.46), is_holonomic=False)

        self.ur10 = self._world.scene.get_object(task_params["arm_name"]["value"])
        self.screw_ur10 = self._world.scene.get_object(task_params["screw_arm"]["value"])

        self.my_controller = KinematicsSolver(self.ur10, attach_gripper=True)
        self.screw_my_controller = KinematicsSolver(self.screw_ur10, attach_gripper=True)

        self.articulation_controller = self.ur10.get_articulation_controller()
        self.screw_articulation_controller = self.screw_ur10.get_articulation_controller()
   
        return

    async def setup_post_reset(self):
        self._my_controller.reset()
        await self._world.play_async()
        return
    
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
        
        # actions.joint_velocities = [0.0001,0.0001,0.0001,0.0001,0.0001,0.0001]
        # actions.joint_velocities = [1,1,1,1,1,1]
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
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.015:
            self.motion_task_counter+=1
            # time.sleep(0.3)
            print("Completed one motion plan: ", self.motion_task_counter)
    
    def do_screw_driving(self, locations):
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        actions, success = self.screw_my_controller.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            self.screw_articulation_controller.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
        # check if reached location
        curr_location = self.give_location("/World/Screw_driving_UR10/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.015:
            self.motion_task_counter+=1
            print("Completed one motion plan: ", self.motion_task_counter)
            
            
    def transform_for_screw_ur10(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0.00419
        return position    

    
    def send_robot_actions(self, step_size):
        current_observations = self._world.get_observations()
        task_params = self._world.get_task("awesome_task").get_params()

        ## Task event numbering:
        # 1 - 30 normal events: forward, stop and add piece, turn
        # 51 - 61 smaller moving platforms events: forward, stop, disappear piece
        # 71 - pick place tasks


        # Terminology
        # mp - moving platform



        # iteration 1 
        # go forward
        if current_observations["task_event"] == 1:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if not self.isDone[current_observations["task_event"]]:
                self.isDone[current_observations["task_event"]]=True

        # small mp brings in part
        elif current_observations["task_event"] == 51:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))

            if not self.isDone[current_observations["task_event"]]:
                motion_plan = [{"index":0, "position": np.array([0.90691+0.16, 0.01077, 0.19514]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-7.00386, -16.5233, 0.43633]), "goal_orientation":np.array([0,0, 0.70711, 0.70711])},
                               {"index":1, "position": np.array([0.97165+0.16, 0.01077, 0.19514]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-7.00686, -16.5233, 0.43633]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                               {"index":2, "position": np.array([0.97165+0.16, 0.01077, 0.31194]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-7.0686, -16.5233, 0.55313]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                               {"index":3, "position": np.array([-1.09292-0.16, 0.24836, 0.52594]), "orientation": np.array([0,0,0.70711,0.70711]), "goal_position":np.array([-5.005, -16.7606, 0.76714]), "goal_orientation":np.array([0.70711, 0.70711, 0,0])}]
                
                self.move_ur10(motion_plan)

                ee_pose = self.give_location("/World/UR10/ee_link")
                print(ee_pose)
                if self.motion_task_counter==2 and not self.bool_done[current_observations["bool_counter"]]:
                    self.bool_done[current_observations["bool_counter"]] = True
                    print("position:", ee_pose.p)
                    print("rotation:", ee_pose.r)
                    self.remove_part("engine_bringer/platform", "fuel")
                    self.add_part_custom("World/UR10/ee_link","fuel", "qfuel", np.array([0.001,0.001,0.001]), np.array([0.22927, -0.16445, -0.08733]), np.array([0.70711,0,-0.70711,0]))
                
                if self.motion_task_counter==4:
                    print("Done motion plan")
                    self.isDone[current_observations["task_event"]]=True
        
         # arm_robot picks and places part
        elif current_observations["task_event"] == 71:
            
            if not self.isDone[current_observations["task_event"]]:
                if not self.bool_done[current_observations["bool_counter"]]:
                    self.bool_done[current_observations["bool_counter"]] = True
                    print("Part removal done")
                    self.remove_part("World/UR10/ee_link", "qfuel")
                    self.motion_task_counter=0
                    self.add_part_custom("mock_robot/platform","fuel", "xfuel", np.array([0.001,0.001,0.001]), np.array([-0.03273, 0.09227, 0.58731]), np.array([0.70711,0.70711,0,0]))

                def transform(points):
                    points[0]-=0.16
                    points[2]-=0.15
                    return points
                motion_plan = [{"index":0, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":1, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.15931, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":2, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":3, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":4, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.4077, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":5, "position": self.transform_for_screw_ur10(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                               {"index":6, "position": self.transform_for_screw_ur10(transform(np.array([-0.04511, 0.7374, 0.41493]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.97604, -17.26595,0.6576]), "goal_orientation":np.array([0,-0.70711,0,0.70711])}]
                print(self.motion_task_counter)
                self.do_screw_driving(motion_plan)
                ee_pose = self.give_location("/World/Screw_driving_UR10/ee_link")
                print("EE_Pose",ee_pose)
                if np.mean(np.abs(ee_pose.p - motion_plan[6]["goal_position"]))<0.058:
                    self.isDone[current_observations["task_event"]]=True
                    self.motion_task_counter=0
                    motion_plan = [{"index":0, "position": np.array([0.07, -0.81, 0.21]), "orientation": np.array([-0.69, 0, 0, 0.72]), "goal_position":np.array([-4.18372, 7.03628, 0.44567]), "goal_orientation":np.array([0.9999, 0, 0, 0])}]
                    self.move_ur10(motion_plan)
                    self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
                    print("done", self.motion_task_counter)

        # remove engine and add engine
        elif current_observations["task_event"] == 2:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))

        # elif current_observations["task_event"] == 3:
        #     self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[-0.5, 0.5, -0.5, 0.5],0]))
        #     self.motion_task_counter=0
        # elif current_observations["task_event"] == 4:
        #     self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
        #     if not self.isDone[current_observations["task_event"]]:
        #         motion_plan = [{"index":0, "position": self.transform_for_screw_ur10(np.array([0.7558, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.42428, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
        #                        {"index":1, "position": self.transform_for_screw_ur10(np.array([0.92167, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.59014, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
        #                        {"index":2, "position": self.transform_for_screw_ur10(np.array([0.7743, 0.13044, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.44277, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
        #                        {"index":3, "position": self.transform_for_screw_ur10(np.array([0.92789, 0.13045, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.59636, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
        #                        {"index":4, "position": np.array([-0.03152, -0.69498, 0.14425]), "orientation": np.array([0.69771, -0.07322, 0.09792, -0.70587]), "goal_position":np.array([-4.04212, 4.63272, 0.38666]), "goal_orientation":np.array([0.99219, -0.12149, 0.01374, 0.02475])}]
        #         self.do_screw_driving(motion_plan)
        #         ee_pose = self.give_location("/World/Screw_driving_UR10/ee_link")
        #         print("EE_Pose",ee_pose)
        #         if self.motion_task_counter>1 and np.mean(np.abs(ee_pose.p - motion_plan[4]["goal_position"]))<0.058:
        #             self.isDone[current_observations["task_event"]]=True
        #             print("done", self.motion_task_counter)
        return
    
    def add_part(self, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/mock_robot/platform/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/mock_robot/platform/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
    

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

    def remove_part(self, parent_prim_name, child_prim_name):

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
