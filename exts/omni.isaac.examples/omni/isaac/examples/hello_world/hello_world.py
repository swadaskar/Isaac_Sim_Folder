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

from omni.isaac.examples.hello_world.assembly_task import AssemblyTask
import time
import asyncio
import rospy

# def func():
#     try:
#         rospy.init_node("hello", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
#     except rospy.exceptions.ROSException as e:
#         print("Node has already been initialized, do nothing")

#     async def my_task():
#         from std_msgs.msg import String
#         pub = rospy.Publisher("/hello_topic", String, queue_size=10)

#         for frame in range(20):
#             pub.publish("hello world " + str(frame))
#             await asyncio.sleep(1.0)
#         pub.unregister()
#         pub = None

#     asyncio.ensure_future(my_task())

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

        world.add_task(AssemblyTask(name="assembly_task"))
        self.isDone = [False]*1000
        self.bool_done = [False]*1000
        self.motion_task_counter=0
        self.path_plan_counter=0
        self.delay=0
        print("inside setup_scene", self.motion_task_counter)
        # self.schedule = deque(["1","71","2","72","3","4","6","5","151","171","181","102","301","351","371","381","302","201"])
        self.schedule = deque(["401","451","471"])
        # "6":"wait",
        #     "5":"move_to_suspension_cell",
        #     "151":"arm_place_suspension",
        #     "171":"screw_suspension",
        #     "181":"arm_remove_suspension",
        #     "102":"wait",
        #     "201":"move_to_fuel_cell",
        #     "251":"arm_place_fuel",
        #     "271":"screw_fuel", 
        #     "281":"arm_remove_fuel",
        #     "202":"wait"
        # func()
        return

    async def setup_post_load(self):
        self._world = self.get_world()

        # Engine cell set up ----------------------------------------------------------------------------
        task_params = self._world.get_task("assembly_task").get_params()
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

        # Suspension cell set up ------------------------------------------------------------------------
        # bring in moving platforms 
        self.suspension_bringer = self._world.scene.get_object(task_params["eb_name_suspension"]["value"])

        # static suspensions on the table
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.83704, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_01", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.69733, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_02", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.54469, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_03", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.3843, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_04", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.22546, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))

        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_05", np.array([0.001,0.001,0.001]), np.array([-6.10203, -4.74457, 0.41322]), np.array([0.70711, 0, -0.70711, 0]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_06", np.array([0.001,0.001,0.001]), np.array([-5.96018, -4.74457, 0.41322]), np.array([0.70711, 0, -0.70711, 0]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_07", np.array([0.001,0.001,0.001]), np.array([-5.7941, -4.74457, 0.41322]), np.array([0.70711, 0, -0.70711, 0]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_08", np.array([0.001,0.001,0.001]), np.array([-5.63427, -4.74457, 0.41322]), np.array([0.70711, 0, -0.70711, 0]))
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_09", np.array([0.001,0.001,0.001]), np.array([-5.47625, -4.74457, 0.41322]), np.array([0.70711, 0, -0.70711, 0]))

        # self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
        # self.add_part("FFrame", "frame", np.array([0.001, 0.001, 0.001]), np.array([0.45216, -0.32084, 0.28512]), np.array([0, 0, 0.70711, 0.70711]))

        self.ur10_suspension = self._world.scene.get_object(task_params["arm_name_suspension"]["value"])
        self.screw_ur10_suspension = self._world.scene.get_object(task_params["screw_arm_suspension"]["value"])

        self.my_controller_suspension = KinematicsSolver(self.ur10_suspension, attach_gripper=True)
        self.screw_my_controller_suspension = KinematicsSolver(self.screw_ur10_suspension, attach_gripper=True)

        self.articulation_controller_suspension = self.ur10_suspension.get_articulation_controller()
        self.screw_articulation_controller_suspension = self.screw_ur10_suspension.get_articulation_controller()


        # Fuel cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.fuel_bringer = self._world.scene.get_object(task_params["eb_name_fuel"]["value"])
        
        # self.add_part_custom("fuel_bringer/platform","fuel", "fuel", np.array([0.001,0.001,0.001]), np.array([-0.1769, 0.13468, 0.24931]), np.array([0.5,0.5,-0.5,-0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_01", np.array([0.001,0.001,0.001]), np.array([-7.01712, -15.89918, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_02", np.array([0.001,0.001,0.001]), np.array([-7.01712, -15.49449, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_03", np.array([0.001,0.001,0.001]), np.array([-6.54859, -15.46717, 0.41958]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","fuel", "fuel_04", np.array([0.001,0.001,0.001]), np.array([-6.14395, -15.47402, 0.41958]), np.array([0, 0, -0.70711, -0.70711]))

        self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.90761, 0.03096, 0.69056]), np.array([0.48732, -0.51946, 0.50085, -0.49176]))
        self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
        # self.add_part_custom("mock_robot/platform","fuel", "xfuel", np.array([0.001,0.001,0.001]), np.array([0.11281, -0.08612, 0.59517]), np.array([0, 0, -0.70711, -0.70711]))
        # Initialize our controller after load and the first reset

        self.ur10_fuel = self._world.scene.get_object(task_params["arm_name_fuel"]["value"])
        self.screw_ur10_fuel = self._world.scene.get_object(task_params["screw_arm_fuel"]["value"])

        self.my_controller_fuel = KinematicsSolver(self.ur10_fuel, attach_gripper=True)
        self.screw_my_controller_fuel = KinematicsSolver(self.screw_ur10_fuel, attach_gripper=True)

        self.articulation_controller_fuel = self.ur10_fuel.get_articulation_controller()
        self.screw_articulation_controller_fuel = self.screw_ur10_fuel.get_articulation_controller()

        # battery cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.battery_bringer = self._world.scene.get_object(task_params["eb_name_battery"]["value"])
        
        self.add_part_custom("World/Environment","battery", "battery_01", np.array([0.001,0.001,0.001]), np.array([-16.47861, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_02", np.array([0.001,0.001,0.001]), np.array([-16.0548, -15.67949, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_03", np.array([0.001,0.001,0.001]), np.array([-16.02688, -16.10727, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_04", np.array([0.001,0.001,0.001]), np.array([-16.01519, -16.40193, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))

        # Initialize our controller after load and the first reset

        self.ur10_battery = self._world.scene.get_object(task_params["arm_name_battery"]["value"])
        self.screw_ur10_battery = self._world.scene.get_object(task_params["screw_arm_battery"]["value"])

        self.my_controller_battery = KinematicsSolver(self.ur10_battery, attach_gripper=True)
        self.screw_my_controller_battery = KinematicsSolver(self.screw_ur10_battery, attach_gripper=True)

        self.articulation_controller_battery = self.ur10_battery.get_articulation_controller()
        self.screw_articulation_controller_battery = self.screw_ur10_battery.get_articulation_controller()

        # trunk cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.trunk_bringer = self._world.scene.get_object(task_params["eb_name_trunk"]["value"])
        
        self.add_part_custom("World/Environment","trunk", "trunk_01", np.array([0.001,0.001,0.001]), np.array([-27.84904, 3.75405, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","trunk", "trunk_02", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))

        # Initialize our controller after load and the first reset

        self.ur10_trunk = self._world.scene.get_object(task_params["arm_name_trunk"]["value"])
        self.screw_ur10_trunk = self._world.scene.get_object(task_params["screw_arm_trunk"]["value"])

        self.my_controller_trunk = KinematicsSolver(self.ur10_trunk, attach_gripper=True)
        self.screw_my_controller_trunk = KinematicsSolver(self.screw_ur10_trunk, attach_gripper=True)

        self.articulation_controller_trunk = self.ur10_trunk.get_articulation_controller()
        self.screw_articulation_controller_trunk = self.screw_ur10_trunk.get_articulation_controller()

        # wheel cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.wheel_bringer = self._world.scene.get_object(task_params["eb_name_wheel"]["value"])
        
        self.add_part_custom("World/Environment","wheel", "wheel_01", np.array([0.001,0.001,0.001]), np.array([-27.84904, 3.75405, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","wheel", "wheel_02", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","wheel", "wheel_03", np.array([0.001,0.001,0.001]), np.array([-27.84904, 3.75405, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","wheel", "wheel_04", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))


        # Initialize our controller after load and the first reset

        self.ur10_wheel = self._world.scene.get_object(task_params["arm_name_wheel"]["value"])
        self.screw_ur10_wheel = self._world.scene.get_object(task_params["screw_arm_wheel"]["value"])

        self.my_controller_wheel = KinematicsSolver(self.ur10_wheel, attach_gripper=True)
        self.screw_my_controller_wheel = KinematicsSolver(self.screw_ur10_wheel, attach_gripper=True)

        self.articulation_controller_wheel = self.ur10_wheel.get_articulation_controller()
        self.screw_articulation_controller_wheel = self.screw_ur10_wheel.get_articulation_controller()

        self.ur10_wheel_01 = self._world.scene.get_object(task_params["arm_name_wheel_01"]["value"])
        self.screw_ur10_wheel_01 = self._world.scene.get_object(task_params["screw_arm_wheel_01"]["value"])

        self.my_controller_wheel_01 = KinematicsSolver(self.ur10_wheel_01, attach_gripper=True)
        self.screw_my_controller_wheel_01 = KinematicsSolver(self.screw_ur10_wheel_01, attach_gripper=True)

        self.articulation_controller_wheel_01 = self.ur10_wheel_01.get_articulation_controller()
        self.screw_articulation_controller_wheel_01 = self.screw_ur10_wheel_01.get_articulation_controller()
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
    
    def move_ur10(self, locations, task_name=""):
        print("Motion task counter", self.motion_task_counter)
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"articulation_controller"+task_name)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        # check if reached location
        curr_location = self.give_location(f"/World/UR10{task_name}/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        diff = np.mean(np.abs(curr_location.p - target_location["goal_position"]))
        if diff<0.02:
            self.motion_task_counter+=1
            # time.sleep(0.3)
            print("Completed one motion plan: ", self.motion_task_counter)
    
    def do_screw_driving(self, locations, task_name=""):
        print(self.motion_task_counter)
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"screw_my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"screw_articulation_controller"+task_name)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
        # check if reached location
        curr_location = self.give_location(f"/World/Screw_driving_UR10{task_name}/ee_link")
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

    def transform_for_screw_ur10_suspension(self, position):
        position[0]-=0
        position[1]+=0
        position[2]+=-0
        return position
    
    def transform_for_screw_ur10_fuel(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0.00419
        return position   
    
    def move_to_engine_cell(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        if current_mp_position[0]<-4.98951:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            return True
        return False

    def arm_place_engine(self):
        print("doing motion plan")
        motion_plan = [{"index":0, "position": np.array([0.97858+0.14-0.3, -0.12572, 0.21991]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.95174-0.3, 0.46095]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                        {"index":1, "position": np.array([0.97858+0.14, -0.12572, 0.21991]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.95174, 0.46095]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                        
                        {"index":2, "position": np.array([0.93302+0.14, -0.12572, 0.54475]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86054, 7.90617, 0.78578]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                    #    {"index":3, "position": np.array([1.00103, -0.12198, 0.24084]), "orientation": np.array([1, 0, 0, 0]), "goal_position":np.array([-4.86409, 7.96971, 0.48132]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},

                        {"index":3, "position": np.array([0.80658+0.15, 0.24732, 0.54475]), "orientation": np.array([0.99217, 0, 0, 0.12489]), "goal_position":np.array([-5.23375, 7.77959, 0.78578]), "goal_orientation":np.array([0.61326, 0, 0, 0.78988])},
                        {"index":4, "position": np.array([0.65068+0.15, 0.39893, 0.54475]), "orientation": np.array([0.97001, 0, 0, 0.24305]), "goal_position":np.array([-5.38549+0.08, 7.6235, 0.78578]), "goal_orientation":np.array([0.51404, 0, 0, 0.85777])},
                        {"index":5, "position": np.array([0.53837+0.15, 0.63504, 0.54475]), "orientation": np.array([0.92149, 0, 0, 0.38841]), "goal_position":np.array([-5.62169+0.12, 7.51092, 0.78578]), "goal_orientation":np.array([0.37695, 0, 0, 0.92624])},
                        {"index":6, "position": np.array([0.33707, 0.82498, 0.54475]), "orientation": np.array([0.77061, 0, 0, 0.6373]), "goal_position":np.array([-5.81157+0.16, 7.30908, 0.78578]), "goal_orientation":np.array([0.09427, 0, 0, 0.99555])},
                        {"index":7, "position": np.array([0.04974, 0.90202, 0.54475]), "orientation": np.array([0.65945, 0, 0, 0.75175]), "goal_position":np.array([-5.88845+0.16, 7.0215, 0.78578]), "goal_orientation":np.array([0.06527, 0, 0, -0.99787])},
                        {"index":8, "position": np.array([-0.25724, 0.83912, 0.54475]), "orientation": np.array([0.41054, 0, 0, 0.91184]), "goal_position":np.array([-5.82509+0.12, 6.71424+0.11, 0.78578]), "goal_orientation":np.array([0.35448, 0, 0, -0.93506])},
                        {"index":9, "position": np.array([-0.54443, 0.27481, 0.37107]), "orientation": np.array([0.14679, 0, 0, 0.98917]), "goal_position":np.array([-5.26026+0.05, 6.42705+0.16, 0.61211]), "goal_orientation":np.array([0.59565, 0, 0, -0.80324])},
                        {"index":10, "position": np.array([-0.60965, -0.03841, 0.37107]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.36196+0.16, 0.61211]), "goal_orientation":np.array([0.70711,0,0,-0.70711])},
                        {"index":11, "position": np.array([-0.67167, -0.03841, 0.16822]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.29994+0.16, 0.40925]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])},
                        {"index":12, "position": np.array([-1.05735, -0.06372, 0.1323]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.92148, 5.91425+0.16, 0.37333]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])},
                        {"index":13, "position": np.array([-1.10475-0.16+0.06, -0.11984, 0.13512]), "orientation": np.array([0,0,0.08495,-0.99639]), "goal_position":np.array([-4.86552, 5.86784+0.06, 0.37552]), "goal_orientation":np.array([0.70455, -0.06007, 0.6007, -0.70455])}]
        self.move_ur10(motion_plan)

        if self.motion_task_counter==2 and not self.bool_done[1]:
            self.bool_done[1] = True
            self.remove_part("World/Environment", "engine_small")
            self.add_part_custom("World/UR10/ee_link","engine_no_rigid", "qengine_small", np.array([0.001,0.001,0.001]), np.array([0.17441, 0.00314, 0.11018]), np.array([0.70365, -0.06987, -0.06987, -0.70365]))

        if self.motion_task_counter==14:
            self.motion_task_counter=0
            print("Done placing engine")
            self.remove_part("World/UR10/ee_link", "qengine_small")
            self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
            return True
        return False
    
    def screw_engine(self):
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
                        
                        {"index":10, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]

        self.do_screw_driving(motion_plan)
        if self.motion_task_counter==11:
            print("Done screwing engine")
            self.motion_task_counter=0
            return True
        return False

    def arm_remove_engine(self):
        motion_plan = [{"index":0, "position": np.array([-0.60965-0.16, -0.03841, 0.37107]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.36196, 0.61211]), "goal_orientation":np.array([0.70711,0,0,-0.70711])},
                        {"index":1, "position": np.array([0.07, -0.81, 0.21]), "orientation": np.array([-0.69, 0, 0, 0.72]), "goal_position":np.array([-4.33372, 7.03628, 0.44567]), "goal_orientation":np.array([0.9999, 0, 0, 0])}]
        self.move_ur10(motion_plan)
        if self.motion_task_counter==2:
            print("Done arm removal")
            self.motion_task_counter=0
            return True
        return False
    
    def turn_mobile_platform(self):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[-0.5, 0.5, -0.5, 0.5],0]))
        if np.mean(np.abs(current_mp_orientation-np.array([0.70711, 0.70711, 0, 0]))) < 0.251:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            return True
        return False
    
    def screw_engine_two(self):
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
                        {"index":10, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 1.048836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.do_screw_driving(motion_plan)
        if self.motion_task_counter==11:
            self.motion_task_counter=0
            print("Done screwing engine 2nd time")
            return True
        return False

    def wait(self):
        print("Waiting ...")
        if self.delay>100:
            print("Done waiting")
            self.delay=0
            return True
        self.delay+=1
        return False

    def move_mp(self, path_plan):
        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        move_type, goal = path_plan[self.path_plan_counter]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[-0.5,0]))
            else:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            else:
                self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
            


    def move_to_suspension_cell(self):
        print(self.path_plan_counter)
        path_plan = [["translate", [-1, 0, False]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, True]],
                     ["translate", [2.29, 1, False]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.503, True]],
                     ["translate", [-4.22, 0, False]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["translate", [-5.3, 1, False]]]
        self.move_mp(path_plan)
        if len(path_plan) == self.path_plan_counter:
            self.path_plan_counter=0
            return True
        return False

    def arm_place_suspension(self):
        motion_plan = [{"index":0, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, -0.05477, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        
                        {"index":3, "position": np.array([-0.96615-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.13459, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                        {"index":4, "position": np.array([-1.10845-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-4.99229, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                        {"index":5, "position": np.array([-1.10842-0.16, -0.39583, 0.29724]), "orientation": np.array([-0.00055, 0.0008, -0.82242, -0.56888]), "goal_position":np.array([-5.00127, -4.80822, 0.53949]), "goal_orientation":np.array([0.56437, 0.82479, 0.02914, 0.01902])}]
        
        self.move_ur10(motion_plan, "_suspension")
        if self.motion_task_counter==2 and not self.bool_done[2]:
            self.bool_done[2] = True
            self.remove_part("World/Environment", "FSuspensionBack_01")
            self.add_part_custom("World/UR10_suspension/ee_link","FSuspensionBack", "qFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([0.16839, 0.158, -0.44332]), np.array([0,0,0,1]))
        
        if self.motion_task_counter==6:
            print("Done placing fuel")
            self.motion_task_counter=0
            self.remove_part("World/UR10_suspension/ee_link", "qFSuspensionBack")
            self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.87892, 0.0239, 0.82432]), np.array([0.40364, -0.58922, 0.57252, -0.40262]))
            return True
        return False
    
    def screw_suspension(self):
        motion_plan = [{"index":0, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":1, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":2, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        
                        {"index":3, "position": self.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        {"index":4, "position": self.transform_for_screw_ur10_suspension(np.array([0.87215+0.16, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.66069, -4.84629,0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        {"index":5, "position": self.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        
                        {"index":6, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":7, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":8, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                        {"index":9, "position": self.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        {"index":10, "position": self.transform_for_screw_ur10_suspension(np.array([0.91167+0.16, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.69933, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        {"index":11, "position": self.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        
                        {"index":12, "position": self.transform_for_screw_ur10_suspension(np.array([-0.08295-0.16, -0.58914, 0.32041-0.15])), "orientation": np.array([0,0.70711, 0, -0.70711]), "goal_position":np.array([-3.544, -4.41856, 0.56125]), "goal_orientation":np.array([0.70711,0,0.70711,0])}]
        self.do_screw_driving(motion_plan, "_suspension")
        if self.motion_task_counter==13:
            print("Done screwing suspension")
            self.motion_task_counter=0
            return True
        return False

    def arm_remove_suspension(self):
        motion_plan = [{"index":0, "position": np.array([-0.95325-0.16, -0.38757, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.14749, -4.80509, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                       {"index":1, "position": np.array([0.03492, 0.9236, 0.80354]), "orientation": np.array([0.70711, 0, 0, 0.70711]), "goal_position":np.array([-6.13579,-5.95519, 1.04451]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])}]
        self.move_ur10(motion_plan, "_suspension")
        if self.motion_task_counter==2:
            print("Done arm removal")
            self.motion_task_counter=0
            return True
        return False

    def move_to_fuel_cell(self):
        print(self.path_plan_counter)
        path_plan = [["translate", [-5.95, 0, True]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
                     ["translate", [-15.945, 1, True]]]
        self.move_mp(path_plan)
        if len(path_plan) == self.path_plan_counter:
            self.path_plan_counter=0
            return True
        return False

    def arm_place_fuel(self):
        motion_plan = [{"index":0, "position": np.array([0.71705+0.16, -0.17496, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -15.98881, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                        {"index":1, "position": np.array([0.87135+0.16, -0.17496, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.98881, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                        {"index":2, "position": np.array([0.87135+0.16, -0.17496, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.98881, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                       {"index":3, "position": np.array([-0.70299-0.16, -0.19609, 0.65442]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448, -15.9671, 0.89604]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":4, "position": np.array([-0.70299-0.16, -0.19609, 0.53588]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448, -15.9671, 0.77749]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])}]

        self.move_ur10(motion_plan, "_fuel")

        if self.motion_task_counter==2 and not self.bool_done[3]:
            self.bool_done[3] = True
            self.remove_part("World/Environment", "fuel_01")
            self.add_part_custom("World/UR10_fuel/ee_link","fuel", "qfuel", np.array([0.001,0.001,0.001]), np.array([0.05467, -0.16886, 0.08908]), np.array([0.70711,0,0.70711,0]))
        
        if self.motion_task_counter==5:
            print("Done placing fuel")
            self.motion_task_counter=0
            self.remove_part("World/UR10_fuel/ee_link", "qfuel")
            self.add_part_custom("mock_robot/platform","fuel", "xfuel", np.array([0.001,0.001,0.001]), np.array([0.11281, -0.08612, 0.59517]), np.array([0, 0, -0.70711, -0.70711]))
            return True
        return False

    def screw_fuel(self):
        def transform(points):
            points[0]-=0
            points[2]-=0.16
            return points
        motion_plan = [{"index":0, "position": transform(np.array([-0.6864, 0.07591, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.55952, -16.016, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":1, "position": transform(np.array([-0.6864, 0.07591, 0.42514])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.55952, -16.016, 0.666]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":2, "position": transform(np.array([-0.6864, 0.07591, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.55952, -16.016, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                        {"index":3, "position": transform(np.array([0.915, -0.1488, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161, -15.791, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":4, "position": transform(np.array([0.915, -0.1488, 0.572])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161, -15.791, 0.814]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":5, "position": transform(np.array([0.915, -0.1488, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161, -15.791, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                       {"index":6, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":7, "position": transform(np.array([-0.68202, -0.09908, 0.42514])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":8, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                        {"index":9, "position": transform(np.array([0.908, 0.104, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154, -16.044, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":10, "position": transform(np.array([0.908, 0.104, 0.572])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154, -16.044, 0.814]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":11, "position": transform(np.array([0.908, 0.104, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154, -16.044, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        
                        {"index":12, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.3])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.3]), "goal_orientation":np.array([0,0.70711,0,-0.70711])}]
        # motion_plan = [{"index":0, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":1, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":2, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":3, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":4, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":5, "position": self.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":6, "position": self.transform_for_screw_ur10_fuel(transform(np.array([-0.04511, 0.7374, 0.41493]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.97604, -17.26595,0.6576]), "goal_orientation":np.array([0,-0.70711,0,0.70711])}]
        self.do_screw_driving(motion_plan,"_fuel")
        if self.motion_task_counter==13:
            print("Done screwing fuel")
            self.motion_task_counter=0
            return True
        return False

    def arm_remove_fuel(self):
        motion_plan = [{"index":0, "position": np.array([-0.70299-0.16, -0.19609, 0.65442]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448, -15.9671, 0.89604]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":1, "position": np.array([0.14684, 0.7364+0.16, 0.53588]), "orientation": np.array([0.5,0.5,0.5,0.5]), "goal_position":np.array([-6.24423, -16.89961, 0.77749]), "goal_orientation":np.array([0.5,0.5,-0.5,-0.5])}]
        self.move_ur10(motion_plan,"_fuel")
        if self.motion_task_counter==2:
            print("Done arm removal")
            self.motion_task_counter=0
            return True
        return False
    
    def move_to_battery_cell(self):
        print(self.path_plan_counter)
        path_plan = [["translate", [-12.5, 1, False]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.503, True]],
                     ["translate", [-8.63, 0, False]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["translate", [-16.85, 1, False]],
                     ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
                     ["translate", [-16.7, 0, False]]]
        self.move_mp(path_plan)
        if len(path_plan) == self.path_plan_counter:
            self.path_plan_counter=0
            return True
        return False

    def arm_place_battery(self):
        motion_plan = [{"index":0, "position": np.array([-0.12728, -0.61362, 0.4+0.1-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.1]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.12728, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.12728, -0.61362, 0.4+0.1-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.1]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                    #    {"index":3, "position": np.array([0.87593, -0.08943, 0.60328-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.42989, -16.24038, 0.8463]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":3, "position": np.array([-0.15683+0.05, 0.85859, 0.57477+0.1-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657+0.1]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])},
                       {"index":4, "position": np.array([-0.15683+0.05, 0.85859, 0.57477-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])}]
        self.move_ur10(motion_plan, "_battery")

        if self.motion_task_counter==2 and not self.bool_done[3]:
            self.bool_done[3] = True
            self.remove_part("World/Environment", "battery_01")
            self.add_part_custom("World/UR10_battery/ee_link","battery", "qbattery", np.array([0.001,0.001,0.001]), np.array([0.2361, 0.05277, 0.03064]), np.array([0.00253, -0.7071, 0.7071, 0.00253]))
        
        if self.motion_task_counter==5:
            print("Done placing battery")
            self.motion_task_counter=0
            self.remove_part("World/UR10_battery/ee_link", "qbattery")
            self.add_part_custom("mock_robot/platform","battery", "xbattery", np.array([0.001,0.001,0.001]), np.array([-0.20126, 0.06146, 0.58443]), np.array([0.4099, 0.55722, -0.58171, -0.42791]))
            return True
        return False

    def screw_battery(self):
        def transform(points):
            points[0]-=0
            points[2]-=0.16
            return points
        motion_plan = [{"index":0, "position": transform(np.array([-0.03593, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":2, "position": transform(np.array([-0.03593, 0.62489, 0.44932])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":1, "position": transform(np.array([-0.03593, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                       {"index":3, "position": transform(np.array([0.28749, -1.04157, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":4, "position": transform(np.array([0.255, -1.04157, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.12807, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":5, "position": transform(np.array([0.28749, -1.04157, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
                       {"index":6, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":7, "position": transform(np.array([-0.21277, 0.62489, 0.44932])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":8, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                       {"index":9, "position": transform(np.array([0.28749, -0.92175, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.24789, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":10, "position": transform(np.array([0.255, -0.92175, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.24789, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":11, "position": transform(np.array([0.28749, -0.92175, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.24789, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
                       {"index":12, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])}]
        self.do_screw_driving(motion_plan,"_battery")
        if self.motion_task_counter==13:
            print("Done screwing battery")
            self.motion_task_counter=0
            return True
        return False

    def arm_remove_battery(self):
        motion_plan = [{"index":0, "position": np.array([-0.15683+0.05, 0.85859, 0.57477+0.1-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657+0.1]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])},
                       {"index":1, "position": np.array([0.03497, -0.67758, 0.82296]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-16.589,-15.6521,1.22562]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        self.move_ur10(motion_plan, "_battery")
        if self.motion_task_counter==2:
            print("Done arm removal")
            self.motion_task_counter=0
            return True
        return False

    def move_to_trunk_cell(self):
        print(self.path_plan_counter)
        path_plan = [
            # ["translate", [-23.49, 1, False]],
            #          ["rotate", [np.array([0, 0, 0, -1]), 0.503, True]],
            #          ["translate", [-35.47, 0, False]],
            #          ["rotate", [np.array([0.70711, 0, 0, 0.70711]), 0.0042, True]],
                     ["translate", [5, 1, False]],
                     ["rotate", [np.array([1, 0, 0, 0]), 0.0042, True]],
                     ["translate", [-26.75, 0, False]]]
        self.move_mp(path_plan)
        if len(path_plan) == self.path_plan_counter:
            self.path_plan_counter=0
            return True
        return False

    def arm_place_trunk(self):
        motion_plan = [{"index":0, "position": np.array([0.9596, 0.21244, -0.16+0.4547+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.9596, 0.21244, -0.16+0.4547]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.9596, 0.21244, -0.16+0.4547+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":3, "position": np.array([0.74919, -0.50484, -0.16+0.64833]), "orientation": np.array([-0.20088, -0.67797, -0.20088, 0.67797]), "goal_position":np.array([-27.85221, 4.99054, 0.89005]), "goal_orientation":np.array([0.67797, -0.20088, 0.67797, 0.20088])},

                        # {"index":4, "position": np.array([0.41663, -0.77637, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.519, 5.262, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":4, "position": np.array([0.55084, -0.81339, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.65404, 5.26, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":5, "position": np.array([0.42543, -0.81339, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.52862, 5.26, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])}]
        self.move_ur10(motion_plan, "_trunk")

        if self.motion_task_counter==2 and not self.bool_done[5]:
            self.bool_done[5] = True
            self.remove_part("World/Environment", "trunk_02")
            self.add_part_custom("World/UR10_trunk/ee_link","trunk", "qtrunk", np.array([0.001,0.001,0.001]), np.array([0.28167, -0.21084, -0.00861]), np.array([0.70711, 0, 0, 0.70711]))
        
        if self.motion_task_counter==6:
            print("Done placing trunk")
            self.motion_task_counter=0
            self.remove_part("World/UR10_trunk/ee_link", "qtrunk")
            self.add_part_custom("mock_robot/platform","trunk", "xtrunk", np.array([0.001,0.001,0.001]), np.array([-0.79319, -0.21112, 0.70114]), np.array([0.5, 0.5, 0.5, 0.5]))
            return True
        return False

    def screw_trunk(self):
        def transform(points):
            points[0]-=0
            points[2]-=0
            return points
        motion_plan = [{"index":0, "position": transform(np.array([-0.15245, -0.65087-0.24329, -0.16+0.43677+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.14748, 7.43945, 0.67876+0.2]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                       {"index":1, "position": transform(np.array([-0.15245, -0.65087-0.24329, -0.16+0.43677])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.14748, 7.43945, 0.67876]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                       {"index":2, "position": transform(np.array([-0.15245, -0.65087-0.24329, -0.16+0.43677+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.14748, 7.43945, 0.67876+0.2]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
                       
                       {"index":3, "position": transform(np.array([-0.12592, 1.126+0.16-0.24329, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.66311, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       {"index":4, "position": transform(np.array([-0.12592, 1.33575+0.16-0.24329, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.45335, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       {"index":5, "position": transform(np.array([-0.12592, 1.126+0.16-0.24329, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.66311, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       {"index":6, "position": transform(np.array([-0.15245, -0.65087-0.24329, -0.16+0.43677+0.3])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.14748, 7.43945, 0.67876+0.3]), "goal_orientation":np.array([0,-0.70711,0,0.70711])}]
        self.do_screw_driving(motion_plan,"_trunk")
        if self.motion_task_counter==7:
            print("Done screwing trunk")
            self.motion_task_counter=0
            return True
        return False

    def arm_remove_trunk(self):
        motion_plan = [{"index":0, "position": np.array([0.55084, -1.08888, 0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.65404, 5.5746, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":1, "position": np.array([0.03497, -0.67758, 0.82296]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-16.589,-15.6521,1.22562]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        self.move_ur10(motion_plan, "_trunk")
        if self.motion_task_counter==2:
            print("Done arm removal")
            self.motion_task_counter=0
            return True
        return False
        

    def send_robot_actions(self, step_size):
        current_observations = self._world.get_observations()
        # print("\n\n\n\nCurrent observations:",current_observations)

        # naming convention
        # {product number}_{station name}_{operation}_{part}_{task number}
        # p1_stationB_install_chain_engine_31
    
        task_to_func_map = {
            "1": "move_to_engine_cell",
            "71":"arm_place_engine",
            "2":"screw_engine",
            "72":"arm_remove_engine",
            "3":"turn_mobile_platform",
            "4":"screw_engine_two",
            "6":"wait",
            "5":"move_to_suspension_cell",
            "151":"arm_place_suspension",
            "171":"screw_suspension",
            "181":"arm_remove_suspension",
            "102":"wait",
            "301":"move_to_battery_cell",
            "351":"arm_place_battery",
            "371":"screw_battery", 
            "381":"arm_remove_battery",
            "302":"wait",
            "201":"move_to_fuel_cell",
            "251":"arm_place_fuel",
            "271":"screw_fuel", 
            "281":"arm_remove_fuel",
            "202":"wait",
            "401":"move_to_trunk_cell",
            "451":"arm_place_trunk",
            "471":"screw_trunk", 
            "481":"arm_remove_trunk",
            "402":"wait",
            "501":"move_to_wheel_cell",
            "551":"arm_place_wheel",
            "571":"screw_wheel", 
            "581":"arm_remove_wheel",
            "502":"wait",
            "601":"move_to_wheel_cell",
            "651":"arm_place_wheel",
            "671":"screw_wheel", 
            "681":"arm_remove_wheel",
            "602":"wait",
            "701":"move_to_lower_cover_cell",
            "751":"arm_place_lower_cover",
            "771":"screw_lower_cover", 
            "781":"arm_remove_lower_cover",
            "702":"wait",
            "721":"move_to_main_cover_cell",
            "731":"arm_place_main_cover",
            "703":"wait",
            "801":"move_to_handle_cell",
            "851":"arm_place_handle",
            "871":"screw_handle", 
            "881":"arm_remove_handle",
            "802":"wait",
            "901":"move_to_light_cell",
            "951":"arm_place_light",
            "971":"screw_light", 
            "981":"arm_remove_light",
            "902":"wait"
        }

        # schedule = deque(["1","71","2","72","3","4","6","5","151","171","181","102","301","351","371","381","302","201"])
        # schedule = deque(["1","71"])

        if self.schedule:
            curr_schedule = self.schedule[0]

            curr_schedule_function = getattr(self, task_to_func_map[curr_schedule])

            function_done = curr_schedule_function()
            print(self.schedule)
            if function_done:
                print("Done with", task_to_func_map[curr_schedule])
                self.schedule.popleft()

        return


        # # engine task --------------------------------------------------------------------------
        ## Task event numbering:
        # 1 - 30 normal events: forward, stop and add piece, turn
        # 51 - 61 smaller moving platforms events: forward, stop, disappear piece
        # 71 - pick place tasks


        # Terminology
        # mp - moving platform

        # second view tasks
        # if current_observations["task_event"] == 21:
        #     self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        # elif current_observations["task_event"] == 22:
        #     self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
        #     self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[-0.5, 0.5, -0.5, 0.5],0]))

        # iteration 1 
        # go forward
        if current_observations["task_event"] == 1:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        
         # arm_robot picks and places part
        elif current_observations["task_event"] == 71:
            print("bool counter: task 71:", current_observations["bool_counter"])
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
                    self.motion_task_counter=0
                    print("Done motion plan")
                   

        # remove engine and add engine
        elif current_observations["task_event"] == 2:
            print("bool counter: task 2:", current_observations["bool_counter"])
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
                               
                               {"index":10, "position": np.array([-0.03152, -0.69498, 0.14425]), "orientation": np.array([0.69771, -0.07322, 0.09792, -0.70587]), "goal_position":np.array([-4.20, 4.63272, 0.38666]), "goal_orientation":np.array([0.99219, -0.12149, 0.01374, 0.02475])}]

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
                               {"index":10, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 0.88836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
                self.do_screw_driving(motion_plan)
                if self.motion_task_counter==11:
                    self.isDone[current_observations["task_event"]]=True
                    print("done", self.motion_task_counter)
        elif current_observations["task_event"] == 6:
            print("task delay")
            self.motion_task_counter=0
        elif current_observations["task_event"] == 5:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
        
        # #  Suspension task ----------------------------------------------------------

        # iteration 1 
        # go forward
        elif current_observations["task_event"] == 101:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if not self.isDone[current_observations["task_event"]]:
                self.isDone[current_observations["task_event"]]=True
        # small mp brings in part
        elif current_observations["task_event"] == 151:
            print(self.motion_task_counter)
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if not self.isDone[current_observations["task_event"]]:
                motion_plan = [{"index":0, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":1, "position": np.array([0.72034, -0.05477, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               {"index":2, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                               
                               {"index":3, "position": np.array([-0.96615-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.13459, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                               {"index":4, "position": np.array([-1.10845-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-4.99229, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                               {"index":5, "position": np.array([-1.10842-0.16, -0.39583, 0.29724]), "orientation": np.array([-0.00055, 0.0008, -0.82242, -0.56888]), "goal_position":np.array([-5.00127, -4.80822, 0.53949]), "goal_orientation":np.array([0.56437, 0.82479, 0.02914, 0.01902])}]
                
                #                {"index":0, "position": np.array([1.11096, -0.01839, 0.31929-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-7.2154, -5.17695, 0.56252]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                #                {"index":1, "position": np.array([1.11096, -0.01839, 0.19845-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-7.2154, -5.17695, 0.44167]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                #                {"index":2, "position": np.array([1.11096, -0.01839, 0.31929]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-7.2154, -5.17695, 0.56252]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                self.move_ur10(motion_plan, "_suspension")

                if self.motion_task_counter==2 and not self.bool_done[current_observations["bool_counter"]]:
                    self.bool_done[current_observations["bool_counter"]] = True
                    self.remove_part("World/Environment", "FSuspensionBack_01")
                    self.add_part_custom("World/UR10_suspension/ee_link","FSuspensionBack", "qFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([0.16839, 0.158, -0.44332]), np.array([0,0,0,1]))
                
                if self.motion_task_counter==6:
                    print("Done motion plan")
                    self.isDone[current_observations["task_event"]]=True
        
         # arm_robot picks and places part
        elif current_observations["task_event"] == 171:
            
            if not self.isDone[current_observations["task_event"]]:
                if not self.bool_done[current_observations["bool_counter"]]:
                    self.bool_done[current_observations["bool_counter"]] = True
                    print("Part removal done")
                    self.remove_part("World/UR10_suspension/ee_link", "qFSuspensionBack")
                    self.motion_task_counter=0
                    self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.87892, 0.0239, 0.82432]), np.array([0.40364, -0.58922, 0.57252, -0.40262]))
            

                motion_plan = [{"index":0, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":1, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":2, "position": self.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               
                               {"index":3, "position": self.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                               {"index":4, "position": self.transform_for_screw_ur10_suspension(np.array([0.87215+0.16, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.66069, -4.84629,0.58477]), "goal_orientation":np.array([0,0,0,1])},
                               {"index":5, "position": self.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                               
                               {"index":6, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":7, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                               {"index":8, "position": self.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                               {"index":9, "position": self.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                               {"index":10, "position": self.transform_for_screw_ur10_suspension(np.array([0.91167+0.16, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.69933, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                               {"index":11, "position": self.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                               
                               {"index":12, "position": self.transform_for_screw_ur10_suspension(np.array([-0.08295-0.16, -0.58914, 0.32041-0.15])), "orientation": np.array([0,0.70711, 0, -0.70711]), "goal_position":np.array([-3.70349, -4.41856, 0.56125]), "goal_orientation":np.array([0.70711,0,0.70711,0])}]
                print(self.motion_task_counter)
                self.do_screw_driving(motion_plan, "_suspension")
                if self.motion_task_counter==13:
                    self.isDone[current_observations["task_event"]]=True
                    self.done = True
                    self.motion_task_counter=0
                    motion_plan = [{"index":0, "position": np.array([-0.95325-0.16, -0.38757, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.14749, -4.80509, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                                   {"index":1, "position": np.array([0.07, -0.81, 0.21]), "orientation": np.array([-0.69, 0, 0, 0.72]), "goal_position":np.array([-4.18372, 7.03628, 0.44567]), "goal_orientation":np.array([0.9999, 0, 0, 0])}]
                    self.move_ur10(motion_plan, "_suspension")
                    # self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
                    print("done", self.motion_task_counter)

        # remove engine and add engine
        elif current_observations["task_event"] == 102:
            if not self.isDone[current_observations["task_event"]]:
                self.isDone[current_observations["task_event"]]=True
                self.done = True
                self.motion_task_counter=0
                motion_plan = [{"index":0, "position": np.array([-0.95325-0.16, -0.38757, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.14749, -4.80509, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                                {"index":1, "position": np.array([0.07, -0.81, 0.21]), "orientation": np.array([-0.69, 0, 0, 0.72]), "goal_position":np.array([-4.18372, 7.03628, 0.44567]), "goal_orientation":np.array([0.9999, 0, 0, 0])}]
                self.move_ur10(motion_plan,  "_suspension")
            print("task 102 delay")
        elif current_observations["task_event"] == 103:
            print("task 103")
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
