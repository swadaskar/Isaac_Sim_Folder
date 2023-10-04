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

from omni.isaac.examples.hello_world.engine_task import EngineTask

import time
import asyncio
import rospy

def func():
    try:
        rospy.init_node("hello", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")

    async def my_task():
        from std_msgs.msg import String
        pub = rospy.Publisher("/hello_topic", String, queue_size=10)

        for frame in range(1000):
            pub.publish("hello world " + str(frame))
            await asyncio.sleep(1.0)
        pub.unregister()
        pub = None

    asyncio.ensure_future(my_task())

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


class HelloWorld(BaseSample):
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

        world.add_task(EngineTask(name="engine_task"))
        self.isDone = [False]*120
        self.bool_done = [False]*120
        self.motion_task_counter=0
        self.delay=0
        print("inside setup_scene", self.motion_task_counter)
        func()
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("engine_task").get_params()
        # bring in moving platforms 
        self.moving_platform = self._world.scene.get_object(task_params["mp_name"]["value"])
        self.engine_bringer = self._world.scene.get_object(task_params["eb_name"]["value"])
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

        self.add_part("FFrame", "frame", np.array([0.001, 0.001, 0.001]), np.array([0.45216, -0.32084, 0.28512]), np.array([0, 0, 0.70711, 0.70711]))
        self.add_part_custom("World/Environment","engine_no_rigid", "engine_small", np.array([0.001, 0.001, 0.001]), np.array([-4.86938, 8.14712, 0.59038]), np.array([0.99457, 0, -0.10411, 0]))
        # self.engine = self.add_part_custom("engine_bringer/platform","engine_no_rigid", "engine_small", np.array([0.001,0.001,0.001]), np.array([-0.00072, -0.00652, 0.46753]), np.array([0,0,0,1]))
        # self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
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
        diff = np.mean(np.abs(curr_location.p - target_location["goal_position"]))
        if diff<0.02 or (self.motion_task_counter>=3 and self.motion_task_counter<=12 and diff<0.09):
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
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.02:
            self.motion_task_counter+=1
            print("Completed one motion plan: ", self.motion_task_counter)
            
            
    def transform_for_screw_ur10(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0
        return position

    def transform_for_ur10(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0.00419
        return position    

    
    def send_robot_actions(self, step_size):
        current_observations = self._world.get_observations()
        task_params = self._world.get_task("engine_task").get_params()

        ## Task event numbering:
        # 1 - 30 normal events: forward, stop and add piece, turn
        # 51 - 61 smaller moving platforms events: forward, stop, disappear piece
        # 71 - pick place tasks


        # Terminology
        # mp - moving platform

        # second view tasks
        if current_observations["task_event"] == 21:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        elif current_observations["task_event"] == 22:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[-0.5, 0.5, -0.5, 0.5],0]))

        # iteration 1 
        # go forward
        elif current_observations["task_event"] == 1:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        
         # arm_robot picks and places part
        elif current_observations["task_event"] == 71:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if not self.isDone[current_observations["task_event"]]:
                
                motion_plan = [{"index":0, "position": np.array([0.97858+0.14-0.3, -0.12572, 0.21991]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.95174-0.3, 0.46095]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                               {"index":1, "position": np.array([0.97858+0.14, -0.12572, 0.21991]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.95174, 0.46095]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                               
                               {"index":2, "position": np.array([0.93302+0.14, -0.12572, 0.54475]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.90617, 0.78578]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                            #    {"index":3, "position": np.array([1.00103, -0.12198, 0.24084]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86409, 7.96971, 0.48132]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},

                               {"index":3, "position": np.array([0.80658+0.15, 0.24732, 0.54475]), "orientation": np.array([0.99217, 0, 0, 0.12489]), "goal_position":np.array([-5.23375, 7.77959, 0.78578]), "goal_orientation":np.array([0.61326, 0, 0, 0.78988])},
                               {"index":4, "position": np.array([0.65068+0.15, 0.39893, 0.54475]), "orientation": np.array([0.97001, 0, 0, 0.24305]), "goal_position":np.array([-5.38549, 7.6235, 0.78578]), "goal_orientation":np.array([0.51404, 0, 0, 0.85777])},
                               {"index":5, "position": np.array([0.53837+0.15, 0.63504, 0.54475]), "orientation": np.array([0.92149, 0, 0, 0.38841]), "goal_position":np.array([-5.62169, 7.51092, 0.78578]), "goal_orientation":np.array([0.37695, 0, 0, 0.92624])},
                               {"index":6, "position": np.array([0.33707, 0.82498, 0.54475]), "orientation": np.array([0.77061, 0, 0, 0.6373]), "goal_position":np.array([-5.81157, 7.30908, 0.78578]), "goal_orientation":np.array([0.09427, 0, 0, 0.99555])},
                               {"index":7, "position": np.array([0.04974, 0.90202, 0.54475]), "orientation": np.array([0.65945, 0, 0, 0.75175]), "goal_position":np.array([-5.88845+0.16, 7.0215, 0.78578]), "goal_orientation":np.array([0.06527, 0, 0, -0.99787])},
                               {"index":8, "position": np.array([-0.25724, 0.83912, 0.54475]), "orientation": np.array([0.41054, 0, 0, 0.91184]), "goal_position":np.array([-5.82509, 6.71424, 0.78578]), "goal_orientation":np.array([0.35448, 0, 0, -0.93506])},
                               {"index":9, "position": np.array([-0.54443, 0.27481, 0.37107]), "orientation": np.array([0.14679, 0, 0, 0.98917]), "goal_position":np.array([-5.26026, 6.42705, 0.61211]), "goal_orientation":np.array([0.59565, 0, 0, -0.80324])},
                               {"index":10, "position": np.array([-0.60965, -0.03841, 0.37107]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.36196, 0.61211]), "goal_orientation":np.array([0.70711,0,0,-0.70711])},
                               {"index":11, "position": np.array([-0.67167, -0.03841, 0.16822]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.29994, 0.40925]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])},
                               {"index":12, "position": np.array([-1.05735, -0.06372, 0.1323]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.92148, 5.91425, 0.37333]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])},
                               {"index":13, "position": np.array([-1.10475-0.16+0.06, -0.11984, 0.13512]), "orientation": np.array([0,0,0.08495,-0.99639]), "goal_position":np.array([-4.86552, 5.86784+0.06, 0.37552]), "goal_orientation":np.array([0.70455, -0.06007, 0.6007, -0.70455])}]
                
                self.move_ur10(motion_plan)

                if self.motion_task_counter==2 and not self.bool_done[current_observations["bool_counter"]]:
                    self.bool_done[current_observations["bool_counter"]] = True
                    self.remove_part("World/Environment", "engine_small")
                    self.add_part_custom("World/UR10/ee_link","engine_no_rigid", "qengine_small", np.array([0.001,0.001,0.001]), np.array([0.17441, 0.00314, 0.11018]), np.array([0.70365, -0.06987, -0.06987, -0.70365]))

                if self.motion_task_counter==14:
                    self.isDone[current_observations["task_event"]]=True
                    print("Done motion plan")
                   

        # remove engine and add engine
        elif current_observations["task_event"] == 2:

            if not self.bool_done[current_observations["bool_counter"]]:
                self.bool_done[current_observations["bool_counter"]] = True
                self.remove_part("World/UR10/ee_link", "qengine_small")
                self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
                
                self.motion_task_counter=0
                # self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
            
            if not self.isDone[current_observations["task_event"]]:

                motion_plan = [{"index":0, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":1, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":2, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               
                               {"index":3, "position": self.transform_for_screw_ur10(np.array([0.74286, 0.3942, 0.24203])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.14051,5.40792,0.477701]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
                               {"index":4, "position": self.transform_for_screw_ur10(np.array([0.60205, 0.3942, 0.24203])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.14051,5.40792-0.14,0.477701]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},

                               {"index":5, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":6, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":7, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               
                               {"index":8, "position": self.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                               {"index":9, "position": self.transform_for_screw_ur10(np.array([0.96984-0.2, -0.03195, 0.16514])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70384, 5.63505-0.2, 0.40916]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                               
                               {"index":10, "position": np.array([-0.03152, -0.69498, 0.14425]), "orientation": np.array([0.69771, -0.07322, 0.09792, -0.70587]), "goal_position":np.array([-4.04212, 4.63272, 0.38666]), "goal_orientation":np.array([0.99219, -0.12149, 0.01374, 0.02475])}]

                self.do_screw_driving(motion_plan)
                if self.motion_task_counter==11:
                    self.isDone[current_observations["task_event"]]=True
                    print("done", self.motion_task_counter)
                    self.motion_task_counter=0

        elif current_observations["task_event"] == 72:
            if not self.bool_done[current_observations["bool_counter"]]:
                self.bool_done[current_observations["bool_counter"]] = True
                self.motion_task_counter=0
            print(self.motion_task_counter)
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if not self.isDone[current_observations["task_event"]]:
                
                motion_plan = [{"index":0, "position": np.array([-0.60965-0.16, -0.03841, 0.37107]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.36196, 0.61211]), "goal_orientation":np.array([0.70711,0,0,-0.70711])},
                               {"index":1, "position": np.array([0.07, -0.81, 0.21]), "orientation": np.array([-0.69, 0, 0, 0.72]), "goal_position":np.array([-4.18372, 7.03628, 0.44567]), "goal_orientation":np.array([0.9999, 0, 0, 0])}]
                self.move_ur10(motion_plan)
                if self.motion_task_counter==2:
                    self.isDone[current_observations["task_event"]]=True
                    self.motion_task_counter=0
                    print("Done motion plan")


        elif current_observations["task_event"] == 3:
            self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[-0.5, 0.5, -0.5, 0.5],0]))
            
        elif current_observations["task_event"] == 4:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if not self.isDone[current_observations["task_event"]]:
                motion_plan = [{"index":0, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":1, "position": np.array([-0.68984, 0.06874, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":2, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                               {"index":3, "position": self.transform_for_screw_ur10(np.array([0.7558-0.2, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.42428-0.2, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
                               {"index":4, "position": self.transform_for_screw_ur10(np.array([0.92167-0.2, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.59014-0.2, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},

                               {"index":5, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":6, "position": np.array([-0.68984, 0.06874, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                               {"index":7, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                               {"index":8, "position": self.transform_for_screw_ur10(np.array([0.7743-0.2, 0.13044, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.44277-0.2, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
                               {"index":9, "position": self.transform_for_screw_ur10(np.array([0.92789-0.2, 0.13045, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.59636-0.2, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
                               {"index":10, "position": np.array([-0.03152, -0.69498, 0.14425]), "orientation": np.array([0.69771, -0.07322, 0.09792, -0.70587]), "goal_position":np.array([-4.20, 4.63272, 0.38666]), "goal_orientation":np.array([0.99219, -0.12149, 0.01374, 0.02475])}]
                self.do_screw_driving(motion_plan)
                if self.motion_task_counter==11:
                    self.isDone[current_observations["task_event"]]=True
                    print("done", self.motion_task_counter)
        elif current_observations["task_event"] == 6:
            print("task delay")
        elif current_observations["task_event"] == 5:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
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
        mp = self._world.get_observations()[self._world.get_task("engine_task").get_params()["mp_name"]["value"]]
        print(mp)
        position, orientation, goal_position = mp['position'], mp['orientation'], mp['goal_position'][task-1]
        # In the function where you are sending robot commands
        print(goal_position)
        action = self._my_controller.forward(start_position=position, start_orientation=orientation, goal_position=goal_position)  # Change the goal position to what you want
        full_action = ArticulationAction(joint_efforts=np.concatenate([action.joint_efforts, action.joint_efforts]) if action.joint_efforts else None, joint_velocities=np.concatenate([action.joint_velocities, action.joint_velocities]), joint_positions=np.concatenate([action.joint_positions, action.joint_positions]) if action.joint_positions else None)
        self.moving_platform.apply_action(full_action)
