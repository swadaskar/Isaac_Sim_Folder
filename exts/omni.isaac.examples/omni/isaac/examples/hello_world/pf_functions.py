import numpy as np
from omni.isaac.examples.hello_world.util import Utils
import asyncio
import rospy
from geometry_msgs.msg import PoseStamped
import rosgraph
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi 

class PartFeederFunctions:
    def __init__(self) -> None:
        self.util = Utils()
        self.isDone = [False]*1000
        self.bool_done = [False]*1000
        self.delay=0
        self.right_side = self.left_side = False

        self.world = None

        self.id = None
        
        # visited array
        # meaning of its values:
        # False - not visited
        # True - visiting or visited 
        self.visited = {"engine":False, "trunk":False, "wheels":False, "cover":False, "handle":False}

        # Engine cell set up ----------------------------------------------------------------------------
        # bring in moving platforms 

        self.moving_platform = None

        self.my_controller = None
        self.screw_my_controller = None

        self.articulation_controller = None
        self.screw_articulation_controller = None

        # Suspension cell set up ------------------------------------------------------------------------

        self.my_controller_suspension = None
        self.screw_my_controller_suspension = None

        self.articulation_controller_suspension = None
        self.screw_articulation_controller_suspension = None


        # Fuel cell set up ---------------------------------------------------------------------------------

        self.my_controller_fuel = None
        self.screw_my_controller_fuel = None

        self.articulation_controller_fuel = None
        self.screw_articulation_controller_fuel = None

        # battery cell set up ---------------------------------------------------------------------------------

        self.my_controller_battery = None
        self.screw_my_controller_battery = None

        self.articulation_controller_battery = None
        self.screw_articulation_controller_battery = None

        # trunk cell set up ---------------------------------------------------------------------------------

        self.my_controller_trunk = None
        self.screw_my_controller_trunk = None

        self.articulation_controller_trunk = None
        self.screw_articulation_controller_trunk = None

        # wheel cell set up ---------------------------------------------------------------------------------

        self.my_controller_wheel = None
        self.screw_my_controller_wheel = None

        self.articulation_controller_wheel = None
        self.screw_articulation_controller_wheel = None

        self.my_controller_wheel_01 = None
        self.screw_my_controller_wheel_01 = None

        self.articulation_controller_wheel_01 = None
        self.screw_articulation_controller_wheel_01 = None

        # lower_cover cell set up ---------------------------------------------------------------------------------

        self.my_controller_lower_cover = None
        self.screw_my_controller_lower_cover = None

        self.articulation_controller_lower_cover = None
        self.screw_articulation_controller_lower_cover = None

        self.my_controller_lower_cover_01 = None
        self.screw_my_controller_lower_cover_01 = None

        self.articulation_controller_lower_cover_01 = None
        self.screw_articulation_controller_lower_cover_01 = None

        self.my_controller_main_cover = None

        self.articulation_controller_main_cover = None
        
        # handle cell set up ---------------------------------------------------------------------------------

        self.my_controller_handle = None
        self.screw_my_controller_handle = None

        self.articulation_controller_handle = None
        self.screw_articulation_controller_handle = None

        # light cell set up --------------------------------------------------------------------------------
        self.my_controller_light = None
        self.screw_my_controller_light = None

        self.articulation_controller_light = None
        self.screw_articulation_controller_light = None

        # Util declarations ----------------------------------------------------------------------------------
    def declare_utils(self):

        self.util.world = self.world

        self.util.id = self.id

        # Engine cell set up ----------------------------------------------------------------------------
        # bring in moving platforms 

        self.util.moving_platform = self.moving_platform

        self.util.my_controller = self.my_controller
        self.util.screw_my_controller = self.screw_my_controller

        self.util.articulation_controller = self.articulation_controller
        self.util.screw_articulation_controller = self.screw_articulation_controller

        # Suspension cell set up ------------------------------------------------------------------------

        self.util.my_controller_suspension = self.my_controller_suspension
        self.util.screw_my_controller_suspension = self.screw_my_controller_suspension

        self.util.articulation_controller_suspension = self.articulation_controller_suspension
        self.util.screw_articulation_controller_suspension = self.screw_articulation_controller_suspension


        # Fuel cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_fuel = self.my_controller_fuel
        self.util.screw_my_controller_fuel = self.screw_my_controller_fuel

        self.util.articulation_controller_fuel = self.articulation_controller_fuel
        self.util.screw_articulation_controller_fuel = self.screw_articulation_controller_fuel

        # battery cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_battery = self.my_controller_battery
        self.util.screw_my_controller_battery = self.screw_my_controller_battery

        self.util.articulation_controller_battery = self.articulation_controller_battery
        self.util.screw_articulation_controller_battery = self.screw_articulation_controller_battery

        # trunk cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_trunk = self.my_controller_trunk
        self.util.screw_my_controller_trunk = self.screw_my_controller_trunk

        self.util.articulation_controller_trunk = self.articulation_controller_trunk
        self.util.screw_articulation_controller_trunk = self.screw_articulation_controller_trunk

        # wheel cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_wheel = self.my_controller_wheel
        self.util.screw_my_controller_wheel = self.screw_my_controller_wheel

        self.util.articulation_controller_wheel = self.articulation_controller_wheel
        self.util.screw_articulation_controller_wheel = self.screw_articulation_controller_wheel

        self.util.my_controller_wheel_01 = self.my_controller_wheel_01
        self.util.screw_my_controller_wheel_01 = self.screw_my_controller_wheel_01

        self.util.articulation_controller_wheel_01 = self.articulation_controller_wheel_01
        self.util.screw_articulation_controller_wheel_01 = self.screw_articulation_controller_wheel_01

        # lower_cover cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_lower_cover = self.my_controller_lower_cover
        self.util.screw_my_controller_lower_cover = self.screw_my_controller_lower_cover

        self.util.articulation_controller_lower_cover = self.articulation_controller_lower_cover
        self.util.screw_articulation_controller_lower_cover = self.screw_articulation_controller_lower_cover

        self.util.my_controller_lower_cover_01 = self.my_controller_lower_cover_01
        self.util.screw_my_controller_lower_cover_01 = self.screw_my_controller_lower_cover_01

        self.util.articulation_controller_lower_cover_01 = self.articulation_controller_lower_cover_01
        self.util.screw_articulation_controller_lower_cover_01 = self.screw_articulation_controller_lower_cover_01

        self.util.my_controller_main_cover = self.my_controller_main_cover

        self.util.articulation_controller_main_cover = self.articulation_controller_main_cover
        
        # handle cell set up ---------------------------------------------------------------------------------

        self.util.my_controller_handle = self.my_controller_handle
        self.util.screw_my_controller_handle = self.screw_my_controller_handle

        self.util.articulation_controller_handle = self.articulation_controller_handle
        self.util.screw_articulation_controller_handle = self.screw_articulation_controller_handle

        # light cell set up --------------------------------------------------------------------------------
        self.util.my_controller_light = self.my_controller_light
        self.util.screw_my_controller_light = self.screw_my_controller_light

        self.util.articulation_controller_light = self.articulation_controller_light
        self.util.screw_articulation_controller_light = self.screw_articulation_controller_light

        # if self.id%2!=0:
        #     self.color_path = ["main_cover","main_cover_orange"]
        # else:
        #     self.color_path = ["main_cover_orange","main_cover"]
    
    def update_color(self):
        if self.id%2!=0:
            self.color_path = ["main_cover","main_cover_orange"]
        else:
            self.color_path = ["main_cover_orange","main_cover"]

    def spawn_new_parts(self):

        if self.id ==2:
            # print(not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id}") and not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/engine_{self.id-1}"))
            pass


        if self.id!=0:
            
            # Engine cell set up ----------------------------------------------------------------------------
            # if not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id}") and not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id-1}") and not self.util.check_prim_exists(f"mock_robot_{self.id}/platform/engine_{self.id}"):
            if not self.isDone[0] and not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id}") and not self.util.check_prim_exists(f"World/Environment/engine_small_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/engine_{self.id-1}"):
                self.isDone[0] = True
                self.util.add_part_custom("World/Environment","engine_no_rigid", f"engine_small_{self.id}", np.array([0.001, 0.001, 0.001]), np.array([-4.86938, 8.14712, 0.59038]), np.array([0.99457, 0, -0.10411, 0]))
                # print(" \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n --------------------------------------spawned engine "+str(self.id)+" \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

            # Suspension cell set up ------------------------------------------------------------------------
            # if not self.util.check_prim_exists(f"World/Environment/FSuspensionBack_01_{self.id}") and not self.util.check_prim_exists(f"World/Environment/FSuspensionBack_01_{self.id-1}") and not self.util.check_prim_exists(f"mock_robot_{self.id}/platform/xFSuspensionBack_{self.id}"):
            if not self.isDone[1] and not self.util.check_prim_exists(f"World/Environment/FSuspensionBack_01_{self.id}") and not self.util.check_prim_exists(f"World/Environment/FSuspensionBack_01_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xFSuspensionBack_{self.id-1}"):
                self.isDone[1] = True
                self.util.add_part_custom("World/Environment","FSuspensionBack", f"FSuspensionBack_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-6.66288, -4.69733, 0.41322]), np.array([0.5, 0.5, -0.5, 0.5]))
                # print(" \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n --------------------------------------spawned suspension "+str(self.id)+" \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        
            # Fuel cell set up ---------------------------------------------------------------------------------
            if not self.isDone[2] and not self.util.check_prim_exists(f"World/Environment/fuel_01_{self.id}") and not self.util.check_prim_exists(f"World/Environment/fuel_01_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xfuel_{self.id-1}"):
                self.isDone[2] = True

                if self.id%2==0:
                    self.util.add_part_custom("World/Environment","fuel", f"fuel_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-7.01712, -15.89918, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))
                else:
                    self.util.add_part_custom("World/Environment","fuel_yellow", f"fuel_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-7.01712, -15.89918, 0.41958]), np.array([0.5, 0.5, -0.5, -0.5]))


            # battery cell set up ---------------------------------------------------------------------------------
            if not self.isDone[3] and not self.util.check_prim_exists(f"World/Environment/battery_01_{self.id}") and not self.util.check_prim_exists(f"World/Environment/battery_01_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xbattery_{self.id-1}"):
                self.isDone[3] = True
                self.util.add_part_custom("World/Environment","battery", f"battery_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-16.47861, -15.68368, 0.41467]), np.array([0.70711, 0.70711, 0, 0]))

            # trunk cell set up ---------------------------------------------------------------------------------
            if not self.isDone[4] and not self.util.check_prim_exists(f"World/Environment/trunk_02_{self.id}") and not self.util.check_prim_exists(f"World/Environment/trunk_02_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xtrunk_{self.id-1}"):
                self.isDone[4] = True
                self.util.add_part_custom("World/Environment","trunk", f"trunk_02_{self.id}", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711]))

            # wheel cell set up ---------------------------------------------------------------------------------
            if not self.isDone[5] and not self.util.check_prim_exists(f"World/Environment/wheel_02_{self.id}") and not self.util.check_prim_exists(f"World/Environment/wheel_02_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xwheel_02_{self.id-1}"):
                self.isDone[5] = True
                self.util.add_part_custom("World/Environment","FWheel", f"wheel_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-15.17319, 4.72577, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
                self.util.add_part_custom("World/Environment","FWheel", f"wheel_02_{self.id}", np.array([0.001,0.001,0.001]), np.array([-15.17319, 5.24566, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
                self.util.add_part_custom("World/Environment","FWheel", f"wheel_03_{self.id}", np.array([0.001,0.001,0.001]), np.array([-18.97836, 4.72577, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))
                self.util.add_part_custom("World/Environment","FWheel", f"wheel_04_{self.id}", np.array([0.001,0.001,0.001]), np.array([-18.97836, 5.24566, 0.42127]), np.array([0.5, -0.5, -0.5, -0.5]))

            # lower_cover cell set up ---------------------------------------------------------------------------------
            if not self.isDone[6] and not self.util.check_prim_exists(f"World/Environment/main_cover_{self.id}") and not self.util.check_prim_exists(f"World/Environment/main_cover_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xmain_cover_{self.id-1}"):
                self.isDone[6] = True
                self.util.add_part_custom("World/Environment","lower_cover", f"lower_cover_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([-26.2541, -15.57458, 0.40595]), np.array([0, 0, 0.70711, 0.70711]))

                self.util.add_part_custom("World/Environment","lower_cover", f"lower_cover_04_{self.id}", np.array([0.001,0.001,0.001]), np.array([-26.26153, -19.13631, 0.40595]), np.array([0, 0, -0.70711, -0.70711]))
                
                if self.id%2==0:
                    self.util.add_part_custom("World/Environment","main_cover", f"main_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([-18.7095-11.83808, -15.70872, 0.28822]), np.array([0.70711, 0.70711,0,0]))
                else:
                    self.util.add_part_custom("World/Environment","main_cover_orange", f"main_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([-18.7095-11.83808, -15.70872, 0.28822]), np.array([0.70711, 0.70711,0,0]))
            
            # handle cell set up ---------------------------------------------------------------------------------
            if not self.isDone[7] and not self.util.check_prim_exists(f"World/Environment/handle_{self.id}") and not self.util.check_prim_exists(f"World/Environment/handle_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xhandle_{self.id-1}"):
                self.isDone[7] = True
                self.util.add_part_custom("World/Environment","handle", f"handle_{self.id}", np.array([0.001,0.001,0.001]), np.array([-29.70213, -7.25934, 1.08875]), np.array([0, 0.70711, 0.70711, 0]))

            # light cell set up ---------------------------------------------------------------------------------
            if not self.isDone[8] and not self.util.check_prim_exists(f"World/Environment/light_03_{self.id}") and not self.util.check_prim_exists(f"World/Environment/light_03_{self.id-1}") and self.util.check_prim_exists(f"mock_robot_{self.id-1}/platform/xlight_{self.id-1}"):
                self.isDone[8] = True
                self.util.add_part_custom("World/Environment","FFrontLightAssembly", f"light_03_{self.id}", np.array([0.001,0.001,0.001]), np.array([-18.07685, -7.35866, -0.71703]), np.array([0.28511, -0.28511, -0.64708, -0.64708]))
    

    # ---------------------------------------------------- part feeder functions ------------------------------------------------------------

    # -----------------------------engine---------------------------------

    def move_pf_engine(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-0.8, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [13.65, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-4.947, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711,0,0,-0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [10.042, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-6.28358, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711,0,0,-0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [7.1069, 1, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def place_engine(self):
        print("doing motion plan")
        motion_plan = [{"index":0, "position": np.array([0.16871, 0.73405+0.16, 0.08489]), "orientation": np.array([0.70711, 0, 0, 0.70711]), "goal_position":np.array([-5.71975, 7.14096, 0.32698]), "goal_orientation":np.array([0,0,0,-1])},
                        {"index":1, "position": np.array([0.16871, 1.09027+0.16, 0.08489]), "orientation": np.array([0.70711, 0, 0, 0.70711]), "goal_position":np.array([-6.07597, 7.14096, 0.32698]), "goal_orientation":np.array([0,0,0,-1])},
                        {"index":2, "position": np.array([0.16871, 1.09027+0.16, 0.35359]), "orientation": np.array([0.70711, 0, 0, 0.70711]), "goal_position":np.array([-6.07597, 7.14096, 0.59568]), "goal_orientation":np.array([0,0,0,-1])},
                        
                        {"index":3, "position": np.array([0.71113, 0.57884, 0.65168]), "orientation": np.array([0.94313, 0, 0, 0.33242]), "goal_position":np.array([-5.46465, 7.557, 0.89377]), "goal_orientation":np.array([0.43184, 0, 0, 0.90195])},# -5.56465, 7.68341, 0.89377

                        {"index":4, "position": np.array([0.97986+0.16, -0.22219, 0.54922]), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.76367, 7.95221, 0.79131]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                        {"index":5, "position": np.array([0.97986+0.16, -0.22219, 0.23754]), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.76367, 7.95221, 0.47963]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},
                        {"index":6, "position": np.array([0.60234+0.16, -0.22219, 0.23754]), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.76367, 7.57469, 0.47963]), "goal_orientation":np.array([0.70711, 0, 0, 0.70711])},

                        {"index":7, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.67382, 7.1364, 1.04897]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10(motion_plan)

        if self.util.motion_task_counter==2 and not self.bool_done[self.id*10]:
            self.bool_done[self.id*10] = True
            self.util.remove_part("pf_engine/platform", f"pf_engine_{self.id}")
            self.util.add_part_custom("World/UR10/ee_link","engine_no_rigid", f"pf_qengine_small_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.19536, 0.10279, 0.11708]), np.array([0.70177, -0.08674, -0.08674, -0.70177]))

        if self.util.motion_task_counter==6 and not self.bool_done[self.id*10+1]:
            self.bool_done[self.id*10+1] = True
            self.util.remove_part("World/UR10/ee_link", f"pf_qengine_small_{self.id}")
            self.util.add_part_custom(f"World/Environment","engine_no_rigid", f"engine_small_{self.id+1}", np.array([0.001,0.001,0.001]), np.array([-4.86938, 8.14712, 0.59038]), np.array([0.99457, 0, -0.10411, 0]))
            
        if self.util.motion_task_counter==8:
            self.util.motion_task_counter=0
            print("Done placing engine")
            return True
        return False
    
    def move_pf_engine_back(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [9.65, 1, False]],
             ["wait", []],
             ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
             ["wait", []],
             ["translate", [-4.947, 0, False]],
             ["wait", []],
             ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
             ["wait", []],
             ["translate", [12.8358, 1, False]],
             ["wait", []],
             ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
             ["wait", []],
             ["translate", [-1.22, 0, False]],
             ["wait", []],
             ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, True]],
             ["wait", []],
             ["translate", [17.63327, 1, False]],
             ["wait", []],
             ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
             ["wait", []],
             ["translate", [8.61707, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.util.add_part_custom("pf_engine/platform","engine_no_rigid", "pf_engine"+f"_{self.id+1}", np.array([0.001, 0.001, 0.001]), np.array([0.07038, 0.03535, 0.42908]), np.array([0, 0.12268, 0, 0.99245]))
            self.id+=1
            return True
        return False
    
    # -----------------------------trunk---------------------------------

    def move_pf_trunk(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-27.271, 0, False]],
                     ["wait",[]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def place_trunk(self):
        motion_plan = [{"index":0, "position": np.array([0.95548, 0.21169, 0.45316+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.05854, 4.27434, 0.69518+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.854, -0.22549, 0.56746-0.16]), "orientation": np.array([0.26914, 0.65388, 0.26914, -0.65388]), "goal_position":np.array([-27.95709, 4.7115, 0.80948]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       
                       {"index":2, "position": np.array([0.28794, -0.72111, 0.34571+0.2-0.16]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-27.39105, 5.20712, 0.58774+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":3, "position": np.array([0.28794, -0.72111, 0.34571-0.16]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-27.39105, 5.20712, 0.58774]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":4, "position": np.array([0.28794, -0.72111, 0.34571+0.2-0.16]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-27.39105, 5.20712, 0.58774+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                       {"index":5, "position": np.array([0.854, -0.22549, 0.56746-0.16]), "orientation": np.array([0.26914, 0.65388, 0.26914, -0.65388]), "goal_position":np.array([-27.95709, 4.7115, 0.80948]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                       {"index":6, "position": np.array([0.95548, 0.21169, 0.45316+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.05854, 4.27434, 0.69518+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":7, "position": np.array([0.95548, 0.21169, 0.45316-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.05854, 4.27434, 0.69518]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":8, "position": np.array([0.95548, 0.21169, 0.45316+0.2-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.05854, 4.27434, 0.69518+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       
                       {"index":9, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.2673, 3.79761, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]

        self.util.move_ur10(motion_plan, "_trunk")

        if self.util.motion_task_counter==4 and not self.bool_done[self.id*10]:
            self.bool_done[self.id*10] = True
            self.util.remove_part("pf_trunk/platform", f"pf_trunk_{self.id}")
            self.util.add_part_custom("World/UR10_trunk/ee_link","trunk", f"pf_qtrunk_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.28167, -0.21084, -0.00861]), np.array([0.70711, 0, 0, 0.70711]))
        
        if self.util.motion_task_counter==8 and not self.bool_done[self.id*10+1]:
            self.bool_done[self.id*10+1] = True
            self.util.remove_part("World/UR10_trunk/ee_link", f"pf_qtrunk_{self.id}")
            self.util.add_part_custom(f"World/Environment","trunk", f"trunk_02_{self.id+1}", np.array([0.001,0.001,0.001]), np.array([-27.84904, 4.26505, 0.41467]), np.array([0, 0, -0.70711, -0.70711])) 
            
        if self.util.motion_task_counter==10:
            self.util.motion_task_counter=0
            print("Done placing trunk")
            return True
        return False
    
    def move_pf_trunk_back(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-42.76, 0, False]],
             ["wait", []]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.util.add_part_custom("pf_trunk/platform","trunk", f"pf_trunk_{self.id+1}", np.array([0.001, 0.001, 0.001]), np.array([-0.1389, -0.2191, 0.28512]), np.array([0.5, 0.5, 0.5, 0.5]))
            self.id+=1
            return True
        return False
    
    # -----------------------------wheels---------------------------------

    # -----------------------------cover---------------------------------

    def move_pf_main_cover(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-17.18, 1, False]],
                     ["wait",[]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.update_color()
            return True
        return False
    
    def place_main_cover(self):
        motion_plan = [
            {"index":0, "position": np.array([0.11095-0.11, 0.94, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":1, "position": np.array([0.11095-0.11, 0.94, 0.2926-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.11226]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":2, "position": np.array([0.11095-0.11, 0.94, 0.19682-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.01648]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":3, "position": np.array([0.11095-0.11, 0.94, 0.15697-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.97663]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":4, "position": np.array([0.11095-0.11, 0.94, 0.11895-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.93861]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":5, "position": np.array([0.11095-0.11, 0.94, 0.07882-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.89848]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":6, "position": np.array([0.06726, 0.93507, -0.1155-0.16+0.09]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-28.639, -17.135, 0.705+0.09]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":7, "position": np.array([0.06726, 0.93507, -0.1155-0.16+0.05]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-28.639, -17.135, 0.705+0.05]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":8, "position": np.array([0.06726, 0.93507, -0.1155-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-28.639, -17.135, 0.705]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":9, "position": np.array([0.06726, 0.93507, -0.1155-0.16+0.05]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-28.639, -17.135, 0.705+0.05]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":10, "position": np.array([0.06726, 0.93507, -0.1155-0.16+0.09]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-28.639, -17.135, 0.705+0.09]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":11, "position": np.array([0.11095-0.11, 0.94, 0.07882-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.89848]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            
            {"index":12, "position": np.array([0.11095, 0.94627, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
            {"index":13, "position": np.array([-0.28875, 0.74261, 0.51038-0.16]), "orientation": np.array([0.70458, -0.0597, 0.70458, 0.0597]), "goal_position":np.array([-16.99268-11.83808, -16.77844, 1.33072]), "goal_orientation":np.array([0.54043, 0.456, 0.54043, -0.456])},
            {"index":14, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},

            {'index': 15, 'position': np.array([-0.74286,  0.42878,  0.35038]), 'orientation': np.array([ 0.6511, -0.2758,  0.6511,  0.2758]), 'goal_position': np.array([-29.14515, -16.32381,   1.33072]), 'goal_orientation': np.array([ 0.65542,  0.26538,  0.65542, -0.26538])}, 
            {'index': 16, 'position': np.array([-0.89016,  0.32513,  0.35038]), 'orientation': np.array([ 0.60698, -0.36274,  0.60698,  0.36274]), 'goal_position': np.array([-29.24913, -16.1764 ,   1.33072]), 'goal_orientation': np.array([ 0.68569,  0.1727 ,  0.68569, -0.1727 ])}, 
            {'index': 17, 'position': np.array([-1.09352, -0.27789,  0.42455]), 'orientation': np.array([ 0.5, -0.5,  0.5,  0.5]), 'goal_position': np.array([-29.85252, -15.97435,   1.40075]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])}, 
            {'index': 18, 'position': np.array([-1.09352, -0.27789,  0.03772]), 'orientation': np.array([ 0.5, -0.5,  0.5,  0.5]), 'goal_position': np.array([-29.85252, -15.97435,   1.01392]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])}, 
            {'index': 19, 'position': np.array([-1.09352, -0.27789,  0.42455]), 'orientation': np.array([ 0.5, -0.5,  0.5,  0.5]), 'goal_position': np.array([-29.85252, -15.97435,   1.40075]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])},

            {'index': 20, 'position': np.array([-0.89016,  0.32513,  0.35038]), 'orientation': np.array([ 0.60698, -0.36274,  0.60698,  0.36274]), 'goal_position': np.array([-29.24913, -16.1764 ,   1.33072]), 'goal_orientation': np.array([ 0.68569,  0.1727 ,  0.68569, -0.1727 ])}, 
            {'index': 21, 'position': np.array([-0.74286,  0.42878,  0.35038]), 'orientation': np.array([ 0.6511, -0.2758,  0.6511,  0.2758]), 'goal_position': np.array([-29.14515, -16.32381,   1.33072]), 'goal_orientation': np.array([ 0.65542,  0.26538,  0.65542, -0.26538])}, 
            {"index": 22, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
            {'index': 23, 'position': np.array([0.16394, 0.68799,  0.44663]), 'orientation': np.array([ 0.70711, 0, 0.70711, 0]), 'goal_position': np.array([-28.88652, -17.23535,   1.42725]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])}]
       
        self.util.move_ur10(motion_plan, "_main_cover")

        if self.util.motion_task_counter==9 and not self.bool_done[self.id*10]:
            self.bool_done[self.id*10] = True
            self.util.remove_part("pf_main_cover/platform", f"pf_main_cover_{self.id}")
            self.util.add_part_custom("World/UR10_main_cover/ee_link",self.color_path[0], f"pf_qmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5, 0.5, -0.5, 0.5]))
        
        if self.util.motion_task_counter==19 and not self.bool_done[self.id*10+1]:
            self.bool_done[self.id*10+1] = True
            self.util.remove_part("World/UR10_main_cover/ee_link", f"pf_qmain_cover_{self.id}")
            self.util.add_part_custom("World/Environment",self.color_path[0], f"main_cover_{self.id+1}", np.array([0.001,0.001,0.001]), np.array([-18.7095-11.83808, -15.70872, 0.28822]), np.array([0.70711, 0.70711,0,0]))
            
        if self.util.motion_task_counter==24:
            self.util.motion_task_counter=0
            print("Done placing main_cover")
            return True
        return False
    
    def move_pf_main_cover_back(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-31.2, 1, False]],
             ["wait", []]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.util.add_part_custom("pf_main_cover/platform",self.color_path[1], f"pf_main_cover_{self.id+1}", np.array([0.001, 0.001, 0.001]), np.array([0.74446, -0.26918, -0.03119]), np.array([0, 0, -0.70711, -0.70711]))
            self.id+=1
            return True
        return False

    # -----------------------------handle---------------------------------
    
    def move_pf_handle(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-32.52, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, 0.70711]), 0.0042, True]],
                    ["wait", []],
                     ["translate", [-7.93, 1, False]],
                    ["wait", []],
                    ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
                    ["wait", []],
                    ["translate", [-28.661, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def place_handle(self):
        motion_plan = [{"index":0, "position": np.array([0.15669, 0.84626, 0.19321-0.16+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.6592, -8.04267, 0.4333+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.15669, 0.84626, 0.19321-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.6592, -8.04267, 0.4333]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.15669, 0.84626, 0.19321-0.16+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.6592, -8.04267, 0.4333+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       
                       {"index":3, "position": np.array([0.63197, 0.53467, 0.5295-0.16]), "orientation": np.array([0.67393, -0.21407, 0.67393, 0.21407]), "goal_position":np.array([-29.13451, -7.73107, 0.76958]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       
                       {"index":4, "position": np.array([0.83727, -0.35853, 0.3259-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33982, -6.83787, 0.56599+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":5, "position": np.array([0.83727, -0.35853, 0.3259-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33982, -6.83787, 0.56599]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":6, "position": np.array([0.83727, -0.35853, 0.3259-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33982, -6.83787, 0.56599+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       
                       {"index":7, "position": np.array([-0.00141, 0.74106, -0.16+0.61331]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.5011, -7.93748, 0.85506]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]

        self.util.move_ur10(motion_plan, "_handle")

        if self.util.motion_task_counter==2 and not self.bool_done[self.id*10]:
            self.bool_done[self.id*10] = True
            self.util.remove_part("pf_handle/platform", f"pf_handle_{self.id}")
            self.util.add_part_custom("World/UR10_handle/ee_link","handle", f"pf_qhandle_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.5218, 0.42317, 0.36311]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counter==6 and not self.bool_done[self.id*10+1]:
            self.bool_done[self.id*10+1] = True
            self.util.remove_part("World/UR10_handle/ee_link", f"pf_qhandle_{self.id}")
            self.util.add_part_custom(f"World/Environment","handle", f"handle_02_{self.id+1}", np.array([0.001,0.001,0.001]), np.array([-29.70213, -7.25934, 1.08875]), np.array([0, 0.70711, 0.70711, 0]))
            
        if self.util.motion_task_counter==8:
            self.util.motion_task_counter=0
            print("Done placing handle")
            return True
        return False
    
    def move_pf_handle_back(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-32.52, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, 0.70711]), 0.0042, True]],
                    ["wait", []],
                     ["translate", [-5.63293, 1, False]],
                    ["wait", []],
                    ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
                    ["wait", []],
                    ["translate", [-42.77298, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.util.add_part_custom("pf_handle/platform","handle", f"pf_handle_{self.id+1}", np.array([0.001, 0.001, 0.001]), np.array([-0.4248, 0.46934, 0.94076]), np.array([0, 1, 0, 0]))
            self.id+=1
            return True
        return False