import carb

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

from collections import deque, defaultdict
from geometry_msgs.msg import PoseStamped
import rosgraph
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi 

from omni.isaac.examples.hello_world.assembly_task import AssemblyTask
from omni.isaac.examples.hello_world.ATV_task import ATVTask
import time
import asyncio
import rospy

from omni.isaac.examples.hello_world.executor_functions import ExecutorFunctions
from omni.isaac.examples.hello_world.pf_functions import PartFeederFunctions
from omni.isaac.core import SimulationContext

import omni.graph.core as og

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

class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.done = False
        self.isThere = [False]*1000

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
        self.motion_task_counterl=0
        self.motion_task_counterr=0
        self.path_plan_counter=0
        self.delay=0

        # ATV declarations -------------------------------------------------------------------------
        self.num_of_ATVs = 8
        for i in range(self.num_of_ATVs):
            world.add_task(ATVTask(name=f"ATV_{i}",offset=np.array([0, i*2, 0])))

        # part feeder declarations -------------------------------------------------------------------------
        self.num_of_PFs = 5
        self.name_of_PFs = [{"name":"engine","prim_name":"engine_small","position":np.array([0.07038, 0.03535, 0.42908]),"orientation":np.array([0, 0.12268, 0, 0.99245]),"mp_pos":np.array([8.61707, 17.63327, 0.03551]),"mp_ori":np.array([0,0,0,1])},
                            {"name":"trunk","prim_name":"trunk_02","position":np.array([-0.1389, -0.2191, 0.28512]),"orientation":np.array([0.5, 0.5, 0.5, 0.5]),"mp_pos":np.array([-42.76706, 5.21645, 0.03551]),"mp_ori":np.array([1,0,0,0])},
                            {"name":"wheels","prim_name":"wheel_02","position":np.array([0.45216, -0.32084, 0.28512]),"orientation":np.array([0, 0, 0.70711, 0.70711]),"mp_pos":np.array([-42.71662, 17.56147, 0.03551]),"mp_ori":np.array([1,0,0,0])},
                            {"name":"main_cover","prim_name":"main_cover","position":np.array([0.74446, -0.26918, -0.03119]),"orientation":np.array([0, 0, -0.70711, -0.70711]),"mp_pos":np.array([-28.65, -31.19876, 0.03551]),"mp_ori":np.array([0.70711, 0, 0, 0.70711])},
                            {"name":"handle","prim_name":"handle","position":np.array([-0.4248, 0.46934, 0.94076]),"orientation":np.array([0, 1, 0, 0]),"mp_pos":np.array([-42.77298, -5.63293, 0.03551]),"mp_ori":np.array([0,0,0,1])}]
        for i in range(self.num_of_PFs):
            world.add_task(ATVTask(name=f"PF_{i}", offset=np.array([0, i*2, 0]), mp_name=f"pf_{self.name_of_PFs[i]['name']}_{i}",mp_pos=self.name_of_PFs[i]['mp_pos'],mp_ori=self.name_of_PFs[i]['mp_ori']))

        print("inside setup_scene", self.motion_task_counter)

        self.schedules = [deque(["251","281"]) for _ in range(self.num_of_ATVs)]



        # self.schedules = [deque(["1","71","2","72","3","4","6","101","151","171","181","102","301","351","371","381","302","201","251","271","281","202","401","451","471","481","402","501","590","591","505","592","593","502","701","790","791","702","721","731","703","801","851","871","802","901","951","971","902"]) for _ in range(self.num_of_ATVs)]
        
        # for i in range(3, len(self.schedules)):
        #     self.schedules[i]=deque(["1","71","2","72","3","4","6","101","151","171","181","102","301","351","371","381","302","201","251","271","281","202","401","402","501","590","591","505","592","593","502","701","790","791","702","721","731","703","801","851","871","802","901","951","971","902"])

        self.pf_schedules = [deque([]) for _ in range(self.num_of_PFs)]
        self.right_side = self.left_side = False
        

        # navigation declarations -----------------------------------------------
        
        if not rosgraph.is_master_online():
            print("Please run roscore before executing this script")
            return
        
        try:
            rospy.init_node("set_goal_py",anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")
        
        # FIXME
        # self._initial_goal_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        # self.__send_initial_pose()
        # await asyncio.sleep(1.0)
        # self._action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        return

    async def setup_post_load(self):
        self._world = self.get_world()

        # part feeders set up ----------------------------------------------------------------------------
        self.PF_tasks = []
        self.part_feeders = []
        self.PF_executions = []
        for i in range(self.num_of_PFs):
            self.PF_tasks.append(self._world.get_task(f"PF_{i}"))
            task_params = self.PF_tasks[i].get_params()
            self.part_feeders.append(self._world.scene.get_object(task_params["mp_name"]["value"]))
            if self.name_of_PFs[i]['name'] == "engine":
                self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","engine_no_rigid", "pf_"+self.name_of_PFs[i]["name"]+f"_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
            elif self.name_of_PFs[i]['name'] != "wheels":
                if self.name_of_PFs[i]['name'] == "main_cover":
                    self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","main_cover_orange", "pf_"+self.name_of_PFs[i]["name"]+f"_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
                else:
                    self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform",self.name_of_PFs[i]["name"], "pf_"+self.name_of_PFs[i]["name"]+f"_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
            else:
                self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","FWheel", "pf_"+self.name_of_PFs[i]["name"]+f"_1_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
                self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","FWheel", "pf_"+self.name_of_PFs[i]["name"]+f"_2_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
                self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","FWheel", "pf_"+self.name_of_PFs[i]["name"]+f"_3_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
                self.add_part_custom("pf_"+self.name_of_PFs[i]["name"]+"/platform","FWheel", "pf_"+self.name_of_PFs[i]["name"]+f"_4_{0}", np.array([0.001, 0.001, 0.001]), self.name_of_PFs[i]["position"], self.name_of_PFs[i]["orientation"])
            
            pf = PartFeederFunctions()
            self.PF_executions.append(pf)


        # mobile platform set up -------------------------------------------------------------------------
        self.ATV_tasks = []
        self.moving_platforms = []
        self.ATV_executions = []
        for i in range(self.num_of_ATVs):
            self.ATV_tasks.append(self._world.get_task(f"ATV_{i}"))
            task_params = self.ATV_tasks[i].get_params()
            self.moving_platforms.append(self._world.scene.get_object(task_params["mp_name"]["value"]))
            self.add_part_custom(f"mock_robot_{i}/platform","FFrame", f"frame_{i}", np.array([0.001, 0.001, 0.001]), np.array([0.45216, -0.32084, 0.28512]), np.array([0, 0, 0.70711, 0.70711]))

            atv = ExecutorFunctions()
            self.ATV_executions.append(atv)

            # og.Controller.set(og.Controller.attribute(f"/mock_robot_{i}/TwistSub" + "/node_namespace.inputs:value"), f"mp{i+1}")
            # og.Controller.set(og.Controller.attribute(f"/mock_robot_{i}/LidarPub" + "/node_namespace.inputs:value"), f"mp{i+1}")
            # og.Controller.set(og.Controller.attribute(f"/mock_robot_{i}/TfAndOdomPub" + "/node_namespace.inputs:value"), f"mp{i+1}")

        # Engine cell set up ----------------------------------------------------------------------------
        task_params = self._world.get_task("assembly_task").get_params()
        # bring in moving platforms 
        # self.moving_platform = self._world.scene.get_object(task_params["mp_name"]["value"])
        self.engine_bringer = self._world.scene.get_object(task_params["eb_name"]["value"])
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        # Initialize our controller after load and the first reset
        # self._my_custom_controller = CustomDifferentialController()
        # self._my_controller = WheelBasePoseController(name="cool_controller", open_loop_wheel_controller=DifferentialController(name="simple_control", wheel_radius=0.125, wheel_base=0.46), is_holonomic=False)

        self.ur10 = self._world.scene.get_object(task_params["arm_name"]["value"])
        self.screw_ur10 = self._world.scene.get_object(task_params["screw_arm"]["value"])

        self.my_controller = KinematicsSolver(self.ur10, attach_gripper=True)
        self.screw_my_controller = KinematicsSolver(self.screw_ur10, attach_gripper=True)

        self.articulation_controller = self.ur10.get_articulation_controller()
        self.screw_articulation_controller = self.screw_ur10.get_articulation_controller()

        # self.add_part("FFrame", "frame", np.array([0.001, 0.001, 0.001]), np.array([0.45216, -0.32084, 0.28512]), np.array([0, 0, 0.70711, 0.70711]))
        self.add_part_custom("World/Environment","engine_no_rigid", "engine_small_0", np.array([0.001, 0.001, 0.001]), np.array([-4.86938, 8.14712, 0.59038]), np.array([0.99457, 0, -0.10411, 0]))

        # Suspension cell set up ------------------------------------------------------------------------
        # bring in moving platforms 
        self.suspension_bringer = self._world.scene.get_object(task_params["eb_name_suspension"]["value"])

        # static suspensions on the table
        self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_00", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.83704, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
        # self.add_part_custom("World/Environment","FSuspensionBack", "FSuspensionBack_01_0", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.69733, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
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
        # self.add_part_custom("World/Environment","fuel", "fuel_01_0", np.array([0.001,0.001,0.001]), np.array([-7.01712, -15.89918, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.offset = -0.25894
        self.add_part_custom("World/Environment","fuel", "fuel_00", np.array([0.001,0.001,0.001]), np.array([-6.96448, -16.13794+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel_yellow", "fuel_01", np.array([0.001,0.001,0.001]), np.array([-6.96448, -15.83793+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_02", np.array([0.001,0.001,0.001]), np.array([-6.96448, -15.53714+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel_yellow", "fuel_03", np.array([0.001,0.001,0.001]), np.array([-6.96448, -15.23127+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_04", np.array([0.001,0.001,0.001]), np.array([-7.22495, -16.13794+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel_yellow", "fuel_05", np.array([0.001,0.001,0.001]), np.array([-7.22495, -15.83793+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel", "fuel_06", np.array([0.001,0.001,0.001]), np.array([-7.22495, -15.53714+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","fuel_yellow", "fuel_07", np.array([0.001,0.001,0.001]), np.array([-7.22495, -15.23127+self.offset, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))

        # self.add_part_custom("World/Environment","fuel", "fuel_3", np.array([0.001,0.001,0.001]), np.array([-6.54859, -15.46717, 0.41958]), np.array([0, 0, -0.70711, -0.70711]))
        # self.add_part_custom("World/Environment","fuel", "fuel_4", np.array([0.001,0.001,0.001]), np.array([-6.14395, -15.47402, 0.41958]), np.array([0, 0, -0.70711, -0.70711]))

        # self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.90761, 0.03096, 0.69056]), np.array([0.48732, -0.51946, 0.50085, -0.49176]))
        # self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
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
        
        # self.add_part_custom("World/Environment","battery", "battery_01_0", np.array([0.001,0.001,0.001]), np.array([-16.47861, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_00", np.array([0.001,0.001,0.001]), np.array([-16.66421, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_01", np.array([0.001,0.001,0.001]), np.array([-16.47861, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_02", np.array([0.001,0.001,0.001]), np.array([-16.29557, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_03", np.array([0.001,0.001,0.001]), np.array([-16.11273, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_04", np.array([0.001,0.001,0.001]), np.array([-16.66421, -15.55639, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_05", np.array([0.001,0.001,0.001]), np.array([-16.47861, -15.55639, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_06", np.array([0.001,0.001,0.001]), np.array([-16.29557, -15.55639, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))
        self.add_part_custom("World/Environment","battery", "battery_07", np.array([0.001,0.001,0.001]), np.array([-16.11273, -15.55639, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))

        # part feeding battery --------------------------------------
        # self.add_part_custom("battery_bringer/platform","battery", "battery_01", np.array([0.001,0.001,0.001]), np.array([-0.23374, 0.08958, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))
        # self.add_part_custom("battery_bringer/platform","battery", "battery_02", np.array([0.001,0.001,0.001]), np.array([-0.23374, -0.13743, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))
        # self.add_part_custom("battery_bringer/platform","battery", "battery_03", np.array([0.001,0.001,0.001]), np.array([0.04161, -0.13743, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))
        # self.add_part_custom("battery_bringer/platform","battery", "battery_04", np.array([0.001,0.001,0.001]), np.array([0.30769, -0.13743, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))
        # self.add_part_custom("battery_bringer/platform","battery", "battery_05", np.array([0.001,0.001,0.001]), np.array([0.03519, 0.08958, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))
        # self.add_part_custom("battery_bringer/platform","battery", "battery_06", np.array([0.001,0.001,0.001]), np.array([0.30894, 0.08958, 0.2623]), np.array([0, 0, 0.70711, 0.70711]))


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
        self.add_part_custom("World/Environment","trunk", "trunk_02_0", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))

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
        
        self.add_part_custom("World/Environment","FWheel", "wheel_01_0", np.array([0.001,0.001,0.001]), np.array([-15.17319, 4.72577, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","FWheel", "wheel_02_0", np.array([0.001,0.001,0.001]), np.array([-15.17319, 5.24566, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","FWheel", "wheel_03_0", np.array([0.001,0.001,0.001]), np.array([-18.97836, 4.72577, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
        self.add_part_custom("World/Environment","FWheel", "wheel_04_0", np.array([0.001,0.001,0.001]), np.array([-18.97836, 5.24566, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))

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

        # lower_cover cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        # self.lower_cover_bringer = self._world.scene.get_object(task_params["eb_name_lower_cover"]["value"])
        
        self.add_part_custom("World/Environment","lower_cover", "lower_cover_01_0", np.array([0.001,0.001,0.001]), np.array([-26.2541, -15.57458, 0.40595]), np.array([0, 0, 0.70711, 0.70711]))
        self.add_part_custom("World/Environment","lower_cover", "lower_cover_02", np.array([0.001,0.001,0.001]), np.array([-26.2541, -15.30883, 0.40595]), np.array([0, 0, 0.70711, 0.70711]))
        self.add_part_custom("World/Environment","lower_cover", "lower_cover_03", np.array([0.001,0.001,0.001]), np.array([-25.86789, -15.30883, 0.40595]), np.array([0, 0, 0.70711, 0.70711]))

        self.add_part_custom("World/Environment","lower_cover", "lower_cover_04_0", np.array([0.001,0.001,0.001]), np.array([-26.26153, -19.13631, 0.40595]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","lower_cover", "lower_cover_05", np.array([0.001,0.001,0.001]), np.array([-26.26153, -19.3805, 0.40595]), np.array([0, 0, -0.70711, -0.70711]))
        self.add_part_custom("World/Environment","lower_cover", "lower_cover_06", np.array([0.001,0.001,0.001]), np.array([-25.88587, -19.3805, 0.40595]), np.array([0, 0, -0.70711, -0.70711]))
        
        self.add_part_custom("World/Environment","main_cover", "main_cover_0", np.array([0.001,0.001,0.001]), np.array([-18.7095-11.83808, -15.70872, 0.28822]), np.array([0.70711, 0.70711,0,0]))
        # Initialize our controller after load and the first reset

        self.ur10_lower_cover = self._world.scene.get_object(task_params["arm_name_lower_cover"]["value"])
        self.screw_ur10_lower_cover = self._world.scene.get_object(task_params["screw_arm_lower_cover"]["value"])

        self.my_controller_lower_cover = KinematicsSolver(self.ur10_lower_cover, attach_gripper=True)
        self.screw_my_controller_lower_cover = KinematicsSolver(self.screw_ur10_lower_cover, attach_gripper=True)

        self.articulation_controller_lower_cover = self.ur10_lower_cover.get_articulation_controller()
        self.screw_articulation_controller_lower_cover = self.screw_ur10_lower_cover.get_articulation_controller()

        self.ur10_lower_cover_01 = self._world.scene.get_object(task_params["arm_name_lower_cover_01"]["value"])
        self.screw_ur10_lower_cover_01 = self._world.scene.get_object(task_params["screw_arm_lower_cover_01"]["value"])

        self.my_controller_lower_cover_01 = KinematicsSolver(self.ur10_lower_cover_01, attach_gripper=True)
        self.screw_my_controller_lower_cover_01 = KinematicsSolver(self.screw_ur10_lower_cover_01, attach_gripper=True)

        self.articulation_controller_lower_cover_01 = self.ur10_lower_cover_01.get_articulation_controller()
        self.screw_articulation_controller_lower_cover_01 = self.screw_ur10_lower_cover_01.get_articulation_controller()

        self.ur10_main_cover = self._world.scene.get_object(task_params["arm_name_main_cover"]["value"])

        self.my_controller_main_cover = KinematicsSolver(self.ur10_main_cover, attach_gripper=True)

        self.articulation_controller_main_cover = self.ur10_main_cover.get_articulation_controller()
        
        # handle cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.handle_bringer = self._world.scene.get_object(task_params["eb_name_handle"]["value"])
        
        self.add_part_custom("World/Environment","handle", "handle_0", np.array([0.001,0.001,0.001]), np.array([-29.70213, -7.25934, 1.08875]), np.array([0, 0.70711, 0.70711, 0]))

        # Initialize our controller after load and the first reset

        self.ur10_handle = self._world.scene.get_object(task_params["arm_name_handle"]["value"])
        self.screw_ur10_handle = self._world.scene.get_object(task_params["screw_arm_handle"]["value"])

        self.my_controller_handle = KinematicsSolver(self.ur10_handle, attach_gripper=True)
        self.screw_my_controller_handle = KinematicsSolver(self.screw_ur10_handle, attach_gripper=True)

        self.articulation_controller_handle = self.ur10_handle.get_articulation_controller()
        self.screw_articulation_controller_handle = self.screw_ur10_handle.get_articulation_controller()
        # self.add_part_custom("World/UR10_main_cover/ee_link","main_cover", f"qmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5, 0.5, -0.5, 0.5]))
        # light cell set up ---------------------------------------------------------------------------------
        # bring in moving platforms 
        self.light_bringer = self._world.scene.get_object(task_params["eb_name_light"]["value"])
        
        self.add_part_custom("World/Environment","FFrontLightAssembly", "light_01", np.array([0.001,0.001,0.001]), np.array([-18.07685, -6.94868, -0.71703]), np.array([0.28511, -0.28511, -0.64708, -0.64708]))
        self.add_part_custom("World/Environment","FFrontLightAssembly", "light_02", np.array([0.001,0.001,0.001]), np.array([-18.07685, -7.14276, -0.71703]), np.array([0.28511, -0.28511, -0.64708, -0.64708]))
        self.add_part_custom("World/Environment","FFrontLightAssembly", "light_03_0", np.array([0.001,0.001,0.001]), np.array([-18.07685, -7.35866, -0.71703]), np.array([0.28511, -0.28511, -0.64708, -0.64708]))

        # self.add_part_custom("mock_robot/platform","handle", "xhandle", np.array([0.001,0.001,0.001]), np.array([0.82439, 0.44736, 1.16068]), np.array([0.20721, 0.68156, -0.67309, -0.19874]))
        # self.add_part_custom("mock_robot/platform","main_cover", "xmain_cover", np.array([0.001,0.001,0.001]), np.array([-0.81508, 0.27909, 0.19789]), np.array([0.70711, 0.70711, 0, 0]))
        # Initialize our controller after load and the first reset

        self.ur10_light = self._world.scene.get_object(task_params["arm_name_light"]["value"])
        self.screw_ur10_light = self._world.scene.get_object(task_params["screw_arm_light"]["value"])

        self.my_controller_light = KinematicsSolver(self.ur10_light, attach_gripper=True)
        self.screw_my_controller_light = KinematicsSolver(self.screw_ur10_light, attach_gripper=True)

        self.articulation_controller_light = self.ur10_light.get_articulation_controller()
        self.screw_articulation_controller_light = self.screw_ur10_light.get_articulation_controller()



        # temporary additions -------------------------------------------------

        # self.add_part_custom("mock_robot/platform","FWheel", "xwheel_02", np.array([0.001,0.001,0.001]), np.array([-0.80934, 0.35041, 0.43888]), np.array([0.5, -0.5, 0.5, -0.5]))

        # self.add_part_custom("mock_robot/platform","FWheel", "xwheel_04", np.array([0.001,0.001,0.001]), np.array([-0.80845, -0.22143, 0.43737]), np.array([0.5, -0.5, 0.5, -0.5]))

        # self.add_part_custom("mock_robot/platform","FWheel", "xwheel_01", np.array([0.001,0.001,0.001]), np.array([0.1522, 0.33709, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))

        # self.add_part_custom("mock_robot/platform","FWheel", "xwheel_03", np.array([0.001,0.001,0.001]), np.array([0.15255, -0.1948, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))

        # self.add_part_custom("mock_robot/platform","trunk", "xtrunk", np.array([0.001,0.001,0.001]), np.array([-0.79319, -0.21112, 0.70114]), np.array([0.5, 0.5, 0.5, 0.5]))

        # self.add_part_custom("mock_robot/platform","battery", "xbattery", np.array([0.001,0.001,0.001]), np.array([-0.20126, 0.06146, 0.58443]), np.array([0.4099, 0.55722, -0.58171, -0.42791]))
        # self.add_part_custom("mock_robot/platform","fuel", "xfuel", np.array([0.001,0.001,0.001]), np.array([0.11281, -0.08612, 0.59517]), np.array([0, 0, -0.70711, -0.70711]))
        # self.add_part_custom("mock_robot/platform","FSuspensionBack", "xFSuspensionBack", np.array([0.001,0.001,0.001]), np.array([-0.87892, 0.0239, 0.82432]), np.array([0.40364, -0.58922, 0.57252, -0.40262]))
        # self.add_part_custom("mock_robot/platform","engine_no_rigid", "engine", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))

        # # part feeder stuff ----------------------------
        # self.add_part_custom("fuel_bringer/platform","fuel", "fuel_05", np.array([0.001,0.001,0.001]), np.array([-0.17458, 0.136, 0.26034]), np.array([0.5 ,0.5, -0.5, -0.5]))
        # self.add_part_custom("fuel_bringer/platform","fuel", "fuel_06", np.array([0.001,0.001,0.001]), np.array([0.10727, 0.136, 0.26034]), np.array([0.5 ,0.5, -0.5, -0.5]))
        # self.add_part_custom("fuel_bringer/platform","fuel", "fuel_07", np.array([0.001,0.001,0.001]), np.array([0.375, 0.136, 0.26034]), np.array([0.5 ,0.5, -0.5, -0.5]))
        
        # self.add_part_custom("suspension_bringer/platform","FSuspensionBack", "FSuspensionBack_10", np.array([0.001,0.001,0.001]), np.array([0.16356, -0.19391, 0.25373]), np.array([0.70711, 0, -0.70711, 0]))
        # self.add_part_custom("suspension_bringer/platform","FSuspensionBack", "FSuspensionBack_11", np.array([0.001,0.001,0.001]), np.array([0.31896, -0.19391, 0.25373]), np.array([0.70711, 0, -0.70711, 0]))
        # self.add_part_custom("suspension_bringer/platform","FSuspensionBack", "FSuspensionBack_12", np.array([0.001,0.001,0.001]), np.array([0.47319, -0.19391, 0.25373]), np.array([0.70711, 0, -0.70711, 0]))
        # self.add_part_custom("suspension_bringer/platform","FSuspensionBack", "FSuspensionBack_13", np.array([0.001,0.001,0.001]), np.array([0.6372, -0.19391, 0.25373]), np.array([0.70711, 0, -0.70711, 0]))
        # self.add_part_custom("suspension_bringer/platform","FSuspensionBack", "FSuspensionBack_14", np.array([0.001,0.001,0.001]), np.array([0.80216, -0.19391, 0.25373]), np.array([0.70711, 0, -0.70711, 0]))

        # self.add_part_custom("engine_bringer/platform","engine_no_rigid", "engine_11", np.array([0.001,0.001,0.001]), np.array([0, 0, 0.43148]), np.array([0.99457, 0, -0.10407, 0]))

        
        # ATV executor class declaration -------------------------------------------------

        self.suspension = [[{"index":0, "position": np.array([0.72034, 0.08607, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -5.27931, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, 0.08607, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -5.27931, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, 0.08607, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -5.27931, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])}],
                        
                        [{"index":0, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, -0.05477, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, -0.05477, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.822, -5.13962, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])}],

                        [{"index":0, "position": np.array([0.72034, -0.20689, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.98634, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, -0.20689, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.98634, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, -0.20689, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.98634, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])}],

                        [{"index":0, "position": np.array([0.72034, -0.36659, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.82664, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, -0.36659, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.82664, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, -0.36659, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.82664, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])}],

                        [{"index":0, "position": np.array([0.72034, -0.52521, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.66802, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":1, "position": np.array([0.72034, -0.52521, 0.33852-0.16]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.66802, 0.58122]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":2, "position": np.array([0.72034, -0.52521, 0.33852-0.16+0.2]), "orientation": np.array([0.5,-0.5,0.5,0.5]), "goal_position":np.array([-6.8209, -4.66802, 0.58122+0.2]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])}],
                        
                        [{"index":0, "position": np.array([0.44319, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.54418, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":1, "position": np.array([0.44319, -0.60758, 0.33852-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.54418, -4.58567, 0.58122]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":2, "position": np.array([0.44319, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.54418, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}],

                        [{"index":0, "position": np.array([0.30166, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.40265, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":1, "position": np.array([0.30166, -0.60758, 0.33852-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.40265, -4.58567, 0.58122]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":2, "position": np.array([0.30166, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.40265, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}],

                        [{"index":0, "position": np.array([0.1356, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.23659, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":1, "position": np.array([0.1356, -0.60758, 0.33852-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.23659, -4.58567, 0.58122]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                        {"index":2, "position": np.array([0.1356, -0.60758, 0.33852-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.23659, -4.58567, 0.58122+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}],
                        ]

        self.battery = [[{"index":0, "position": np.array([0.05713, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.05713, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.05713, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],
                        
                        [{"index":0, "position": np.array([-0.12728, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.12728, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.12728, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],

                       [{"index":0, "position": np.array([-0.31243, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.31243, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.31243, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],
                        
                        [{"index":0, "position": np.array([-0.49671, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.49671, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.49671, -0.61362, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.71631, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],

                        [{"index":0, "position": np.array([0.05713, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.05713, -0.74178, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.58815, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.05713, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.61088, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],
                        
                        [{"index":0, "position": np.array([-0.12728, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.12728, -0.74178, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.58815, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.12728, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],

                       [{"index":0, "position": np.array([-0.31243, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.31243, -0.74178, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.58815, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.31243, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.24132, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],
                        
                        [{"index":0, "position": np.array([-0.49671, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.49671, -0.74178, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.58815, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.49671, -0.74178, 0.4+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.05704, -15.58815, 0.64303+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}],
                        ]

        self.fuel = [[{"index":0, "position": np.array([0.71705+0.16, 0.06232-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -16.22609+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16, 0.06232-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -16.22609+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16, 0.06232-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -16.22609+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16, -0.2367-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -15.92707+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16, -0.2367-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.92707+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16, -0.2367-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.92707+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16, -0.53766-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -15.62611+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16, -0.53766-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.62611+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16, -0.53766-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.62611+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16, -0.8428-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -15.32097+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16, -0.8428-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.32097+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16, -0.8428-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.32097+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],
                    
                      [{"index":0, "position": np.array([0.71705+0.16+0.26557, 0.06232-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443-0.26557, -16.22609+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16+0.26557, 0.06232-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -16.22609+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16+0.26557, 0.06232-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -16.22609+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16+0.26557, -0.2367-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443-0.26557, -15.92707+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16+0.26557, -0.2367-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.92707+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16+0.26557, -0.2367-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.92707,+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16+0.26557, -0.53766-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443-0.26557, -15.62611+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16+0.26557, -0.53766-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.62611+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16+0.26557, -0.53766-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.62611+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],

                      [{"index":0, "position": np.array([0.71705+0.16+0.26557, -0.8428-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443-0.26557, -15.32097+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":1, "position": np.array([0.87135+0.16+0.26557, -0.8428-self.offset, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.32097+self.offset, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                      {"index":2, "position": np.array([0.87135+0.16+0.26557, -0.8428-self.offset, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873-0.26557, -15.32097+self.offset, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])}],
                    ]

        for i,ATV in enumerate(self.ATV_executions):
            # id ----------------
            ATV.id = i

            # no part feeding parts
            ATV.suspension = self.suspension[i]
            ATV.battery = self.battery[i]
            ATV.fuel = self.fuel[-1]

            # ATV declarations ----------------------------------------------------------
            ATV.moving_platform = self.moving_platforms[i]

            # Engine cell setup -------------------------
            ATV.my_controller = self.my_controller
            ATV.screw_my_controller = self.screw_my_controller
            ATV.articulation_controller = self.articulation_controller
            ATV.screw_articulation_controller = self.screw_articulation_controller

            # Suspension cell set up ------------------------------------------------------------------------

            ATV.my_controller_suspension = self.my_controller_suspension
            ATV.screw_my_controller_suspension = self.screw_my_controller_suspension

            ATV.articulation_controller_suspension = self.articulation_controller_suspension
            ATV.screw_articulation_controller_suspension = self.screw_articulation_controller_suspension


            # Fuel cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_fuel = self.my_controller_fuel
            ATV.screw_my_controller_fuel = self.screw_my_controller_fuel

            ATV.articulation_controller_fuel = self.articulation_controller_fuel
            ATV.screw_articulation_controller_fuel = self.screw_articulation_controller_fuel

            # battery cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_battery = self.my_controller_battery
            ATV.screw_my_controller_battery = self.screw_my_controller_battery

            ATV.articulation_controller_battery = self.articulation_controller_battery
            ATV.screw_articulation_controller_battery = self.screw_articulation_controller_battery

            # trunk cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_trunk = self.my_controller_trunk
            ATV.screw_my_controller_trunk = self.screw_my_controller_trunk

            ATV.articulation_controller_trunk = self.articulation_controller_trunk
            ATV.screw_articulation_controller_trunk = self.screw_articulation_controller_trunk

            # wheel cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_wheel = self.my_controller_wheel
            ATV.screw_my_controller_wheel = self.screw_my_controller_wheel

            ATV.articulation_controller_wheel = self.articulation_controller_wheel
            ATV.screw_articulation_controller_wheel = self.screw_articulation_controller_wheel

            ATV.my_controller_wheel_01 = self.my_controller_wheel_01
            ATV.screw_my_controller_wheel_01 = self.screw_my_controller_wheel_01

            ATV.articulation_controller_wheel_01 = self.articulation_controller_wheel_01
            ATV.screw_articulation_controller_wheel_01 = self.screw_articulation_controller_wheel_01

            # lower_cover cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_lower_cover = self.my_controller_lower_cover
            ATV.screw_my_controller_lower_cover = self.screw_my_controller_lower_cover

            ATV.articulation_controller_lower_cover = self.articulation_controller_lower_cover
            ATV.screw_articulation_controller_lower_cover = self.screw_articulation_controller_lower_cover

            ATV.my_controller_lower_cover_01 = self.my_controller_lower_cover_01
            ATV.screw_my_controller_lower_cover_01 = self.screw_my_controller_lower_cover_01

            ATV.articulation_controller_lower_cover_01 = self.articulation_controller_lower_cover_01
            ATV.screw_articulation_controller_lower_cover_01 = self.screw_articulation_controller_lower_cover_01

            ATV.my_controller_main_cover = self.my_controller_main_cover

            ATV.articulation_controller_main_cover = self.articulation_controller_main_cover
            
            # handle cell set up ---------------------------------------------------------------------------------

            ATV.my_controller_handle = self.my_controller_handle
            ATV.screw_my_controller_handle = self.screw_my_controller_handle

            ATV.articulation_controller_handle = self.articulation_controller_handle
            ATV.screw_articulation_controller_handle = self.screw_articulation_controller_handle

            # light cell set up --------------------------------------------------------------------------------
            ATV.my_controller_light = self.my_controller_light
            ATV.screw_my_controller_light = self.screw_my_controller_light

            ATV.articulation_controller_light = self.articulation_controller_light
            ATV.screw_articulation_controller_light = self.screw_articulation_controller_light
            
            ATV.world = self._world

            ATV.declare_utils()

        # PF executor class declaration -------------------------------------------------

        for i,PF in enumerate(self.PF_executions):
            # id ----------------
            PF.id = 0

            # PF declarations ----------------------------------------------------------
            PF.moving_platform = self.part_feeders[i]

            # Engine cell setup -------------------------
            PF.my_controller = self.my_controller
            PF.screw_my_controller = self.screw_my_controller
            PF.articulation_controller = self.articulation_controller
            PF.screw_articulation_controller = self.screw_articulation_controller

            # Suspension cell set up ------------------------------------------------------------------------

            PF.my_controller_suspension = self.my_controller_suspension
            PF.screw_my_controller_suspension = self.screw_my_controller_suspension

            PF.articulation_controller_suspension = self.articulation_controller_suspension
            PF.screw_articulation_controller_suspension = self.screw_articulation_controller_suspension


            # Fuel cell set up ---------------------------------------------------------------------------------

            PF.my_controller_fuel = self.my_controller_fuel
            PF.screw_my_controller_fuel = self.screw_my_controller_fuel

            PF.articulation_controller_fuel = self.articulation_controller_fuel
            PF.screw_articulation_controller_fuel = self.screw_articulation_controller_fuel

            # battery cell set up ---------------------------------------------------------------------------------

            PF.my_controller_battery = self.my_controller_battery
            PF.screw_my_controller_battery = self.screw_my_controller_battery

            PF.articulation_controller_battery = self.articulation_controller_battery
            PF.screw_articulation_controller_battery = self.screw_articulation_controller_battery

            # trunk cell set up ---------------------------------------------------------------------------------

            PF.my_controller_trunk = self.my_controller_trunk
            PF.screw_my_controller_trunk = self.screw_my_controller_trunk

            PF.articulation_controller_trunk = self.articulation_controller_trunk
            PF.screw_articulation_controller_trunk = self.screw_articulation_controller_trunk

            # wheel cell set up ---------------------------------------------------------------------------------

            PF.my_controller_wheel = self.my_controller_wheel
            PF.screw_my_controller_wheel = self.screw_my_controller_wheel

            PF.articulation_controller_wheel = self.articulation_controller_wheel
            PF.screw_articulation_controller_wheel = self.screw_articulation_controller_wheel

            PF.my_controller_wheel_01 = self.my_controller_wheel_01
            PF.screw_my_controller_wheel_01 = self.screw_my_controller_wheel_01

            PF.articulation_controller_wheel_01 = self.articulation_controller_wheel_01
            PF.screw_articulation_controller_wheel_01 = self.screw_articulation_controller_wheel_01

            # lower_cover cell set up ---------------------------------------------------------------------------------

            PF.my_controller_lower_cover = self.my_controller_lower_cover
            PF.screw_my_controller_lower_cover = self.screw_my_controller_lower_cover

            PF.articulation_controller_lower_cover = self.articulation_controller_lower_cover
            PF.screw_articulation_controller_lower_cover = self.screw_articulation_controller_lower_cover

            PF.my_controller_lower_cover_01 = self.my_controller_lower_cover_01
            PF.screw_my_controller_lower_cover_01 = self.screw_my_controller_lower_cover_01

            PF.articulation_controller_lower_cover_01 = self.articulation_controller_lower_cover_01
            PF.screw_articulation_controller_lower_cover_01 = self.screw_articulation_controller_lower_cover_01

            PF.my_controller_main_cover = self.my_controller_main_cover

            PF.articulation_controller_main_cover = self.articulation_controller_main_cover
            
            # handle cell set up ---------------------------------------------------------------------------------

            PF.my_controller_handle = self.my_controller_handle
            PF.screw_my_controller_handle = self.screw_my_controller_handle

            PF.articulation_controller_handle = self.articulation_controller_handle
            PF.screw_articulation_controller_handle = self.screw_articulation_controller_handle

            # light cell set up --------------------------------------------------------------------------------
            PF.my_controller_light = self.my_controller_light
            PF.screw_my_controller_light = self.screw_my_controller_light

            PF.articulation_controller_light = self.articulation_controller_light
            PF.screw_articulation_controller_light = self.screw_articulation_controller_light
            
            PF.world = self._world

            PF.declare_utils()

        
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
    
    def _check_goal_reached(self, goal_pose):
        # Cannot get result from ROS because /move_base/result also uses move_base_msgs module
        mp_position, mp_orientation = self.moving_platforms[0].get_world_pose()
        _, _, mp_yaw = euler_from_quaternion(mp_orientation)
        _, _, goal_yaw = euler_from_quaternion(goal_pose[3:])
        
        # FIXME: pi needed for yaw tolerance here because map rotated 180 degrees
        if np.allclose(mp_position[:2], goal_pose[:2], atol=self._xy_goal_tolerance) \
            and abs(mp_yaw-goal_yaw) <= pi + self._yaw_goal_tolerance:
            print("Goal "+str(goal_pose)+" reached!")
            # This seems to crash Isaac sim...
            # self.get_world().remove_physics_callback("mp_nav_check")
    
    # Goal hardcoded for now
    def _send_navigation_goal(self, x=None, y=None, a=None):
        # x, y, a = -18, 14, 3.14
        # x,y,a = -4.65, 5.65,3.14
        orient_x, orient_y, orient_z, orient_w = quaternion_from_euler(0, 0, a)
        pose = [x, y, 0, orient_x, orient_y, orient_z, orient_w]

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.get_rostime()
        print("goal pose: "+str(pose))
        goal_msg.pose.position.x = pose[0]
        goal_msg.pose.position.y = pose[1]
        goal_msg.pose.position.z = pose[2]
        goal_msg.pose.orientation.x = pose[3]
        goal_msg.pose.orientation.y = pose[4]
        goal_msg.pose.orientation.z = pose[5]
        goal_msg.pose.orientation.w = pose[6]

        world = self.get_world()

        self._goal_pub.publish(goal_msg)

        # self._check_goal_reached(pose)

        world = self.get_world()
        if not world.physics_callback_exists("mp_nav_check"):
            world.add_physics_callback("mp_nav_check", lambda step_size: self._check_goal_reached(pose))
        # Overwrite check with new goal
        else:
            world.remove_physics_callback("mp_nav_check")
            world.add_physics_callback("mp_nav_check", lambda step_size: self._check_goal_reached(pose))
    
    def move_to_engine_cell(self):
        # # print("sending nav goal")
        if not self.bool_done[123]:
            self._send_navigation_goal(-4.65, 5.65, 3.14)
            self.bool_done[123] = True
        return False

    def send_robot_actions(self, step_size):
        current_observations = self._world.get_observations()

        # print("\n\n\n\nCurrent observations:",current_observations)

        # naming convention
        # {product number}_{station name}_{operation}_{part}_{task number}
        # p1_stationB_install_chain_engine_31
    
        task_to_func_map = {
            "0":"move_to_engine_cell_nav",
            "1": "move_to_engine_cell",
            "71":"arm_place_engine",
            "2":"screw_engine",
            "72":"arm_remove_engine",
            "3":"turn_mobile_platform",
            "4":"screw_engine_two",
            "6":"wait",

            "101":"move_to_suspension_cell",
            "151":"arm_place_suspension",
            "171":"screw_suspension",
            "181":"arm_remove_suspension",
            "102":"wait",

            "301":"move_to_battery_cell",
            "351":"arm_place_battery",
            "371":"screw_battery", 
            "381":"arm_remove_battery",
            "302":"wait",

            "305":"move_mp_to_battery_cell",
            "306":"battery_part_feeding",
            "307":"moving_part_feeders",

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
            "590":"arm_place_fwheel_together",
            "591":"screw_fwheel_together",
            "505":"move_ahead_in_wheel_cell",
            "592":"arm_place_bwheel_together",
            "593":"screw_bwheel_together",
            "502":"wait",

            "701":"move_to_lower_cover_cell",
            "790":"arm_place_lower_cover_together",
            "791":"screw_lower_cover_together",
            "702":"wait",
            "721":"move_to_main_cover_cell",
            "731":"arm_place_main_cover",
            "703":"wait",

            "801":"move_to_handle_cell",
            "851":"arm_place_handle",
            "871":"screw_handle", 
            "802":"wait",

            "901":"move_to_light_cell",
            "951":"arm_place_light",
            "971":"screw_light",
            "902":"wait"
        }

        # schedule = deque(["1","71","2","72","3","4","6","101","151","171","181","102","301","351","371","381","302","201","251","271","281","202","401","451","471","481","402","501","590","591","505","592","593","502","701","790","791","702","721","731","703","801","851","871","881","802","901","951","971","902"])
        # schedule = deque(["1","71","2","72","3","4","6","101","102","301","302","201","202","401","402","501","505","502","701","721","703","801","851","871","881","802","901","902"])
        # schedule = deque(["1","71"])

        sc = SimulationContext()
        print("Time:", sc.current_time)

        for i in range(len(self.schedules)):
            # wait for part feeder check
            if i>0:
                isWait = self.wait_for_parts(i)

                if self.schedules[i-1] and self.schedules[i] and self.schedules[i]==self.schedules[i-1]:
                    isWait=True
                    if isWait and sc.current_time>i*60:
                        print("ATV "+str(i)+": ", self.schedules[i])
                        print("Waiting for next mp to move...")
                else:
                    if isWait and sc.current_time>i*60:
                        print("ATV "+str(i)+": ", self.schedules[i])
                        print("Waiting for part...")
            else:
                isWait = False

            # if self.schedules[i] and sc.current_time>i*60 and not isWait:
            if self.schedules[i] and sc.current_time>i*6:
                print("ATV "+str(i)+": ", self.schedules[i])
                curr_schedule = self.schedules[i][0]

                curr_schedule_function = getattr(self.ATV_executions[i], task_to_func_map[curr_schedule])

                function_done = curr_schedule_function()
                
                if function_done:
                    print("Done with", task_to_func_map[curr_schedule])
                    self.schedules[i].popleft()

                # updating visited status for each ATV
                if self.schedules[i]:
                    new_schedule = self.schedules[i][0]
                    if not self.ATV_executions[i].visited["engine"] and any(int(new_schedule) >= x for x in [0,1,2,3,4,6,71,72]):
                        self.ATV_executions[i].visited["engine"]=True
                    if not self.ATV_executions[i].visited["trunk"] and any(int(new_schedule) >= x for x in [401,451,471,481,402]):
                        self.ATV_executions[i].visited["trunk"]=True
                    if not self.ATV_executions[i].visited["wheels"] and any(int(new_schedule) >= x for x in [501,590,591,505,592,593,502]):
                        self.ATV_executions[i].visited["wheels"]=True
                    if not self.ATV_executions[i].visited["cover"] and any(int(new_schedule) >= x for x in [701,790,791,702,721,731,703]):
                        self.ATV_executions[i].visited["cover"]=True
                    if not self.ATV_executions[i].visited["handle"] and any(int(new_schedule) >= x for x in [801,851,871,802]):
                        self.ATV_executions[i].visited["handle"]=True
                



        pf_to_function = {"6":"wait",
                          "81":"move_pf_engine",
                        "82":"place_engine",
                        "83":"move_pf_engine_back",
                        "181":"move_pf_trunk",
                        "182":"place_trunk",
                        "183":"move_pf_trunk_back",
                        "381":"move_pf_main_cover",
                        "382":"place_main_cover",
                        "383":"move_pf_main_cover_back",
                        "481":"move_pf_handle",
                        "482":"place_handle",
                        "483":"move_pf_handle_back"}             

        # for i in range(len(self.pf_schedules)):
        for i in range(5):
            
            if not self.check_prim_exists_extra("World/Environment/"+self.name_of_PFs[i]["prim_name"]) and not self.pf_schedules[i]:
                if self.name_of_PFs[i]["name"]=="engine":
                    self.pf_schedules[i] = deque(["81","82","83"])
                elif self.name_of_PFs[i]["name"]=="trunk":
                    self.pf_schedules[i] = deque(["181","182","183"])
                elif self.name_of_PFs[i]["name"]=="main_cover":
                    self.pf_schedules[i] = deque(["381","382","383"])
                elif self.name_of_PFs[i]["name"]=="handle":
                    self.pf_schedules[i] = deque(["481","482","483"])

            print("PF "+self.name_of_PFs[i]["name"]+": ", self.pf_schedules[i])
            if self.pf_schedules[i]:
                # print("PF "+str(i)+": ", self.pf_schedules[i])
                curr_schedule = self.pf_schedules[i][0]

                curr_schedule_function = getattr(self.PF_executions[i], pf_to_function[curr_schedule])

                function_done = curr_schedule_function()
                
                if function_done:
                    print("Done with", pf_to_function[curr_schedule])
                    self.pf_schedules[i].popleft()

        # for i in range(1,self.num_of_ATVs):
        #     if self.schedules[i-1] and self.schedules[i]:
        #         # print("Task "+str(i), int(self.schedules[i-1][0])//100, int(self.schedules[i][0])//100)
        #         if int(self.schedules[i-1][0])//100 > int(self.schedules[i][0])//100:
        #             self.ATV_executions[i].spawn_new_parts()
        #     else:
        #         # print("Task "+str(i), "F", int(self.schedules[i][0])//100)
        #         self.ATV_executions[i].spawn_new_parts()

        return
    
    def add_part_custom(self, parent_prim_name, part_name, prim_name, scale, position, orientation):
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/{parent_prim_name}/{prim_name}") # gives asset ref path
        part= self._world.scene.add(XFormPrim(prim_path=f'/{parent_prim_name}/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
        return part

    def wait(self):
        print("Waiting ...")
        if self.delay>100:
            print("Done waiting")
            self.delay=0
            return True
        self.delay+=1
        return False
    
    def check_prim_exists_extra(self, prim_path):
        for i in range(self.num_of_ATVs):
            curr_prim = self._world.stage.GetPrimAtPath("/"+prim_path+f"_{i}")
            if curr_prim.IsValid():
                return True
        return False
    
    def check_prim_exists(self, prim_path):
        curr_prim = self._world.stage.GetPrimAtPath("/"+prim_path)
        if curr_prim.IsValid():
            return True
        return False
    
    def wait_for_parts(self, id):
        isWait = False
        isWait |= not self.check_prim_exists(f"World/Environment/engine_small_{id}") and not self.ATV_executions[id].visited["engine"] and self.ATV_executions[id-1].visited["engine"]
        isWait |= not self.check_prim_exists(f"World/Environment/trunk_02_{id}") and not self.ATV_executions[id].visited["trunk"] and self.ATV_executions[id-1].visited["trunk"]
        isWait |= not self.check_prim_exists(f"World/Environment/wheel_02_{id}") and not self.ATV_executions[id].visited["wheels"] and self.ATV_executions[id-1].visited["wheels"]
        isWait |= not self.check_prim_exists(f"World/Environment/main_cover_{id}") and not self.ATV_executions[id].visited["cover"] and self.ATV_executions[id-1].visited["cover"]
        isWait |= not self.check_prim_exists(f"World/Environment/handle_{id}") and not self.ATV_executions[id].visited["handle"] and self.ATV_executions[id-1].visited["handle"]
        return isWait