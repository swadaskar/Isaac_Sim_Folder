import numpy as np
from omni.isaac.examples.hello_world.util import Utils
import asyncio
import rospy
from geometry_msgs.msg import PoseStamped
import rosgraph
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi 

class ExecutorFunctions:
    def __init__(self) -> None:
        self.util = Utils()
        self.isDone = [False]*1000
        self.bool_done = [False]*1000
        self.delay=0
        self.right_side = self.left_side = False

        # disassembly counter
        self.disassembly_event=0

        self.world = None

        self.id = None

        # non part feeder parts
        self.suspension = None
        self.battery = None
        self.fuel = None
        self.light = None
        self.lower_cover = None

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

        # self._goal_pub = rospy.Publisher(f"/mp{self.id+1}/move_base_simple/goal", PoseStamped, queue_size=1)
        # self._xy_goal_tolerance = 0.25
        # self._yaw_goal_tolerance = 0.05


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

    def _check_goal_reached(self, goal_pose):
        # Cannot get result from ROS because /move_base/result also uses move_base_msgs module
        mp_position, mp_orientation = self.moving_platform.get_world_pose()
        _, _, mp_yaw = euler_from_quaternion(mp_orientation)
        _, _, goal_yaw = euler_from_quaternion(goal_pose[3:])
        
        # FIXME: pi needed for yaw tolerance here because map rotated 180 degrees
        if np.allclose(mp_position[:2], goal_pose[:2], atol=self._xy_goal_tolerance) \
            and abs(mp_yaw-goal_yaw) <= pi + self._yaw_goal_tolerance:
            print(f"Goal for mp_{self.id} "+str(goal_pose)+" reached!")
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
        if not world.physics_callback_exists(f"mp_nav_check_{self.id}"):
            world.add_physics_callback(f"mp_nav_check_{self.id}", lambda step_size: self._check_goal_reached(pose))
        # Overwrite check with new goal
        else:
            world.remove_physics_callback(f"mp_nav_check_{self.id}")
            world.add_physics_callback(f"mp_nav_check_{self.id}", lambda step_size: self._check_goal_reached(pose))
    
    def move_to_engine_cell_nav(self):
        # # print("sending nav goal")
        if not self.bool_done[123]:
            self._send_navigation_goal(-4.65, 5.65, 3.14)
            self.bool_done[123] = True
        return False
        
    def move_to_engine_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-4.98951, 0, False], {"position": np.array([-4.98951, 5.65, 0.03551]), "orientation": np.array([0, 0, 0, 1])}]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
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
        self.util.move_ur10(motion_plan)

        if self.util.motion_task_counter==2 and not self.bool_done[1]:
            self.bool_done[1] = True
            self.util.remove_part("World/Environment", f"engine_small_{self.id}")
            self.util.add_part_custom("World/UR10/ee_link","engine_no_rigid", f"qengine_small_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.17441, 0.00314, 0.11018]), np.array([0.70365, -0.06987, -0.06987, -0.70365]))

        if self.util.motion_task_counter==14:
            self.util.motion_task_counter=0
            print("Done placing engine")
            self.util.remove_part("World/UR10/ee_link", f"qengine_small_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","engine_no_rigid", f"engine_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.16041, -0.00551, 0.46581]), np.array([0.98404, -0.00148, -0.17792, -0.00274]))
            return True
        return False
    
    def screw_engine(self):
        # motion_plan = [{"index":0, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":1, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":2, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        
        #                 {"index":3, "position": self.util.transform_for_screw_ur10(np.array([0.74286, 0.3942, 0.24203])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.14051,5.40792,0.477701]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
        #                 {"index":4, "position": self.util.transform_for_screw_ur10(np.array([0.60205, 0.3942, 0.24203])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.14051,5.40792-0.14,0.477701]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
        #                 {"index":5, "position": self.util.transform_for_screw_ur10(np.array([0.74286, 0.3942, 0.24203])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.14051,5.40792,0.477701]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},

        #                 {"index":6, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":7, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":8, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        
        #                 {"index":9, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
        #                 {"index":10, "position": self.util.transform_for_screw_ur10(np.array([0.96984-0.2, -0.03195, 0.16514])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70384, 5.63505-0.2, 0.40916]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
        #                 {"index":11, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        
        #                 {"index":12, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]

        motion_plan = [{"index":0, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        {"index":1, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        {"index":2, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        
                        {"index":3, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":4, "position": self.util.transform_for_screw_ur10(np.array([0.96984-0.2, -0.03195, 0.16514])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70384, 5.63505-0.2, 0.40916]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":5, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},

        ]


        self.util.do_screw_driving(motion_plan)
        if self.util.motion_task_counter==6:
            print("Done screwing engine")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_remove_engine(self):
        motion_plan = [{"index":0, "position": np.array([-0.60965-0.16, -0.03841, 0.37107]), "orientation": np.array([0,0,0,-1]), "goal_position":np.array([-4.94679, 6.36196, 0.61211]), "goal_orientation":np.array([0.70711,0,0,-0.70711])},
                        {"index":1, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.67382, 7.1364, 1.04897]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10(motion_plan)
        if self.util.motion_task_counter==2:
            print("Done arm removal")
            self.util.motion_task_counter=0
            return True
        return False
    
    def turn_mobile_platform(self):
        print(self.util.path_plan_counter)
        path_plan = [["rotate", [np.array([1, 0, 0, 0]), 0.0042, True]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def screw_engine_two(self):
        # motion_plan = [{"index":0, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":1, "position": np.array([-0.68984, 0.06874, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":2, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

        #                 {"index":3, "position": self.util.transform_for_screw_ur10(np.array([0.7558-0.2, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.42428-0.2, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},
        #                 {"index":4, "position": self.util.transform_for_screw_ur10(np.array([0.92167-0.2, 0.59565, 0.17559])), "orientation": np.array([0.24137, -0.97029, -0.00397, -0.0163]), "goal_position":np.array([-5.3358, 5.59014-0.2, 0.41358]), "goal_orientation":np.array([0.18255, -0.68481, -0.68739, 0.15875])},

        #                 {"index":5, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":6, "position": np.array([-0.68984, 0.06874, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                 {"index":7, "position": np.array([-0.68984, 0.06874, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.80693, 3.97591, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

        #                 {"index":8, "position": self.util.transform_for_screw_ur10(np.array([0.7743-0.2, 0.13044, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.44277-0.2, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
        #                 {"index":9, "position": self.util.transform_for_screw_ur10(np.array([0.92789-0.2, 0.13045, 0.24968])), "orientation": np.array([0.14946, 0.98863, 0.00992, 0.01353]), "goal_position":np.array([-4.8676, 5.59636-0.2, 0.48787]), "goal_orientation":np.array([0.09521, 0.6933, 0.70482, 0.1162])},
        #                 {"index":10, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 1.048836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        
        motion_plan = [{"index":0 , "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":1, "position": self.util.transform_for_screw_ur10(np.array([0.96984-0.2, -0.03195, 0.16514])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70384, 5.63505-0.2, 0.40916]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":2, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},

                        {"index":3, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        {"index":4, "position": np.array([-0.68114, -0.10741, -0.16+0.43038]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        {"index":5, "position": np.array([-0.68114, -0.10741, -0.16+0.43038+0.2]), "orientation": np.array([0,-0.70711, 0, 0.70711]), "goal_position":np.array([-4.63079, 3.98461, 0.67129+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        
                        {"index":6, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":7, "position": self.util.transform_for_screw_ur10(np.array([0.96984-0.2, -0.03195, 0.16514])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70384, 5.63505-0.2, 0.40916]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        {"index":8, "position": self.util.transform_for_screw_ur10(np.array([0.82391-0.2, -0.02307, 0.15366])), "orientation": np.array([0.34479, 0.93825, -0.02095, 0.019]), "goal_position":np.array([-4.70797, 5.48974-0.2, 0.40163]), "goal_orientation":np.array([0.20664, 0.69092, 0.65241, 0.233])},
                        
                        {"index":9, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.42692, 4.82896, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.do_screw_driving(motion_plan)
        if self.util.motion_task_counter==10:
            self.util.motion_task_counter=0
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
    
    def wait_infinitely(self):
        print("Waiting ...")
        self.moving_platform.apply_action(self.util._my_custom_controller.forward(command=[0,0]))
        return False
        # if self.delay>100:
        #     print("Done waiting")
        #     self.delay=0
        #     return True
        # self.delay+=1
        # return False

    def move_to_suspension_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-1, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [2.29, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]], # 503
                     ["wait",[]],
                    #  ["translate", [-4.22, 0, False]],
                     ["translate", [-4.7, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-5.3, 1, False], {"position": np.array([-5.3, -4.876, 0.03551]), "orientation": np.array([0, 0, 0, 1])}]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_suspension(self):
        motion_plan = [*self.suspension,
                        {"index":3, "position": np.array([-0.96615-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.13459, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                        {"index":4, "position": np.array([-1.10845-0.16, -0.56853+0.12, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-4.99229, -4.62413-0.12, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                        {"index":5, "position": np.array([-1.10842-0.16, -0.39583, 0.29724]), "orientation": np.array([-0.00055, 0.0008, -0.82242, -0.56888]), "goal_position":np.array([-5.00127, -4.80822, 0.53949]), "goal_orientation":np.array([0.56437, 0.82479, 0.02914, 0.01902])}]
        
        self.util.move_ur10(motion_plan, "_suspension")
        if self.util.motion_task_counter==2 and not self.bool_done[2]:
            self.bool_done[2] = True
            self.util.remove_part("World/Environment", f"FSuspensionBack_0{self.id}")
            self.util.add_part_custom("World/UR10_suspension/ee_link","FSuspensionBack", f"qFSuspensionBack_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.16839, 0.158, -0.44332]), np.array([0,0,0,1]))
        
        if self.util.motion_task_counter==6:
            print("Done placing fuel")
            self.util.motion_task_counter=0
            self.util.remove_part("World/UR10_suspension/ee_link", f"qFSuspensionBack_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FSuspensionBack", f"xFSuspensionBack_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.87892, 0.0239, 0.82432]), np.array([0.40364, -0.58922, 0.57252, -0.40262]))
            return True
        return False
    
    def screw_suspension(self):
        motion_plan = [{"index":0, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":1, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":2, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.56003, 0.05522, -0.16+0.43437+0.25])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.2273, -5.06269, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        
                        {"index":3, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        {"index":4, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.87215+0.16, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.66069, -4.84629,0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        {"index":5, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.83141+0.16-0.2, -0.16343, 0.34189])), "orientation": np.array([1,0,0,0]), "goal_position":np.array([-4.61995+0.2, -4.84629, 0.58477]), "goal_orientation":np.array([0,0,0,1])},
                        
                        {"index":6, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":7, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":8, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.55625, -0.1223, -0.16+0.43437+0.2])), "orientation": np.array([0, -0.70711,0,0.70711]), "goal_position":np.array([-3.23108, -4.88517, 0.67593+0.25]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                        {"index":9, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        {"index":10, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.91167+0.16, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.69933, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        {"index":11, "position": self.util.transform_for_screw_ur10_suspension(np.array([0.81036+0.16-0.1, -0.26815, 0.24723])), "orientation": np.array([0,-1, 0, 0]), "goal_position":np.array([-4.59801+0.1, -4.7396, 0.49012]), "goal_orientation":np.array([0,0,1,0])},
                        
                        {"index":12, "position": self.util.transform_for_screw_ur10_suspension(np.array([-0.08295-0.16, -0.58914, 0.32041-0.15])), "orientation": np.array([0,0.70711, 0, -0.70711]), "goal_position":np.array([-3.544, -4.41856, 0.56125]), "goal_orientation":np.array([0.70711,0,0.70711,0])}]
        self.util.do_screw_driving(motion_plan, "_suspension")
        if self.util.motion_task_counter==13:
            print("Done screwing suspension")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_remove_suspension(self):
        motion_plan = [{"index":0, "position": np.array([-0.95325-0.16, -0.38757, 0.31143]), "orientation": np.array([-0.00257, 0.00265, -0.82633, -0.56318]), "goal_position":np.array([-5.14749, -4.80509, 0.55254]), "goal_orientation":np.array([0.56316, 0.82633, -0.00001, -0.00438])},
                       {"index":1, "position": np.array([0.03492, 0.9236, 0.80354]), "orientation": np.array([0.70711, 0, 0, 0.70711]), "goal_position":np.array([-6.13579,-5.95519, 1.04451]), "goal_orientation":np.array([0.70711, 0, 0, -0.70711])}]
        self.util.move_ur10(motion_plan, "_suspension")
        if self.util.motion_task_counter==2:
            print("Done arm removal")
            self.util.motion_task_counter=0
            return True
        return False

    def move_to_fuel_cell(self):
        print(self.util.path_plan_counter)
        # path_plan = [["translate", [-5.15, 0, True]],
        #              ["wait",[]],
        #              ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
        #              ["wait",[]],
        #              ["translate", [-15.945, 1, True]]]
        path_plan = [["translate", [-9.69, 0, True]],
                     ["wait",[]],
                     ["rotate", [np.array([0.06883, 0, 0, 0.99763]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-8.53, 0, True]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-5.15, 0, True]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-15.945, 1, True]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_fuel(self):
        motion_plan = [
                        # {"index":0, "position": np.array([0.71705+0.16, -0.17496, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.81443, -15.98881, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                        # {"index":1, "position": np.array([0.87135+0.16, -0.17496, 0.34496]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.98881, 0.58618]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                        # {"index":2, "position": np.array([0.87135+0.16, -0.17496, 0.48867]), "orientation": np.array([0.70711,0.70711,0,0]), "goal_position":np.array([-6.96873, -15.98881, 0.72989]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                        *self.fuel,
                       {"index":3, "position": np.array([-0.70299-0.16-0.04247, -0.19609, 0.65442]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448+0.04247, -15.9671, 0.89604]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":4, "position": np.array([-0.70299-0.16-0.04247, -0.19609, 0.53588]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448+0.04247, -15.9671, 0.77749]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])}]

        self.util.move_ur10(motion_plan, "_fuel")

        if self.util.motion_task_counter==2 and not self.bool_done[4]:
            self.bool_done[4] = True
            self.util.remove_part("World/Environment", f"fuel_0{self.id}")
            if self.id%2==0:
                self.util.add_part_custom("World/UR10_fuel/ee_link","fuel", f"qfuel_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.05467, -0.16886, 0.08908]), np.array([0.70711,0,0.70711,0]))
            else:
                self.util.add_part_custom("World/UR10_fuel/ee_link","fuel_yellow", f"qfuel_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.05467, -0.16886, 0.08908]), np.array([0.70711,0,0.70711,0]))
        
        if self.util.motion_task_counter==5:
            print("Done placing fuel")
            self.util.motion_task_counter=0
            self.util.remove_part("World/UR10_fuel/ee_link", f"qfuel_{self.id}")
            if self.id%2==0:
                self.util.add_part_custom(f"mock_robot_{self.id}/platform","fuel", f"xfuel_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.11281, -0.08612, 0.59517]), np.array([0, 0, -0.70711, -0.70711]))
            else:
                self.util.add_part_custom(f"mock_robot_{self.id}/platform","fuel_yellow", f"xfuel_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.11281, -0.08612, 0.59517]), np.array([0, 0, -0.70711, -0.70711]))
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

                        {"index":3, "position": transform(np.array([0.915+0.04247-0.08717, -0.1488, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161-0.04247+0.08717, -15.791, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":4, "position": transform(np.array([0.915+0.04247-0.08717, -0.1488, 0.572])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161-0.04247+0.08717, -15.791, 0.814]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":5, "position": transform(np.array([0.915+0.04247-0.08717, -0.1488, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.161-0.04247+0.08717, -15.791, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                       {"index":6, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":7, "position": transform(np.array([-0.68202, -0.09908, 0.42514])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":8, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                        {"index":9, "position": transform(np.array([0.908+0.04247-0.08717, 0.104, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154-0.04247+0.08717, -16.044, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":10, "position": transform(np.array([0.908+0.04247-0.08717, 0.104, 0.572])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154-0.04247+0.08717, -16.044, 0.814]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        {"index":11, "position": transform(np.array([0.908+0.04247-0.08717, 0.104, 0.572+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-5.154-0.04247+0.08717, -16.044, 0.814+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                        
                        {"index":12, "position": transform(np.array([-0.68202, -0.09908, 0.42514+0.3])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.5639, -15.84102, 0.666+0.3]), "goal_orientation":np.array([0,0.70711,0,-0.70711])}]
        # motion_plan = [{"index":0, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":1, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":2, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.15931, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.68786, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":3, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":4, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.5447]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.78736]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":5, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([0.74393, 0.4077, 0.61626]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-4.76508, -16.93625, 0.85892]), "goal_orientation":np.array([0,-0.70711,0,0.70711])},
        #                 {"index":6, "position": self.util.transform_for_screw_ur10_fuel(transform(np.array([-0.04511, 0.7374, 0.41493]))), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-3.97604, -17.26595,0.6576]), "goal_orientation":np.array([0,-0.70711,0,0.70711])}]
        self.util.do_screw_driving(motion_plan,"_fuel")
        if self.util.motion_task_counter==13:
            print("Done screwing fuel")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_remove_fuel(self):
        motion_plan = [{"index":0, "position": np.array([-0.70299-0.16-0.04247, -0.19609, 0.65442]), "orientation": np.array([0, 0, -0.70711, -0.70711]), "goal_position":np.array([-5.39448+0.04247, -15.9671, 0.89604]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":1, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-6.2616, -16.8517, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10(motion_plan,"_fuel")
        if self.util.motion_task_counter==2:
            print("Done arm removal")
            self.util.motion_task_counter=0
            return True
        return False
    
    def move_to_battery_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
            ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, True]],
            ["wait",[]],
                    ["translate", [-12.5, 1, False]],
                    ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.503, True]],
                     ["wait",[]],
                     ["translate", [-9.54, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-17.17, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-16.7, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_battery(self):
        motion_plan = [
                    #     {"index":0, "position": np.array([-0.12728, -0.61362, 0.4+0.1-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.1]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                    #    {"index":1, "position": np.array([-0.12728, -0.61362, 0.4-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                    #    {"index":2, "position": np.array([-0.12728, -0.61362, 0.4+0.1-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.42647, -15.71631, 0.64303+0.1]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                    *self.battery,

                    #    {"index":3, "position": np.array([0.87593, -0.08943, 0.60328-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.42989, -16.24038, 0.8463]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":3, "position": np.array([-0.15683+0.05, 0.95185-0.09326, 0.57477+0.1-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657+0.1]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])},
                       {"index":4, "position": np.array([-0.15683+0.05, 0.95185-0.09326, 0.57477-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])}]
        self.util.move_ur10(motion_plan, "_battery")

        if self.util.motion_task_counter==2 and not self.bool_done[3]:
            self.bool_done[3] = True
            self.util.remove_part("World/Environment", f"battery_0{self.id}")
            self.util.add_part_custom("World/UR10_battery/ee_link","battery", f"qbattery_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.2361, 0.05277, 0.03064]), np.array([0.00253, -0.7071, 0.7071, 0.00253]))
        
        if self.util.motion_task_counter==5:
            print("Done placing battery")
            self.util.motion_task_counter=0
            self.util.remove_part("World/UR10_battery/ee_link", f"qbattery_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","battery", f"xbattery_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.20126, 0.06146, 0.58443]), np.array([0.4099, 0.55722, -0.58171, -0.42791]))
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

                       {"index":3, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":4, "position": transform(np.array([0.255, -1.04157+0, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.12807-0, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":5, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
                       {"index":6, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":7, "position": transform(np.array([-0.21277, 0.62489, 0.44932])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
                       {"index":8, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

                       {"index":9, "position": transform(np.array([0.28749, -0.92175+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.24789-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":10, "position": transform(np.array([0.255, -0.92175+0, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.24789-0, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       {"index":11, "position": transform(np.array([0.28749, -0.92175+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.24789-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
                       {"index":12, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.515, -18.858, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        
        # motion_plan = [{"index":0, "position": transform(np.array([-0.03593, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
        #                {"index":1, "position": transform(np.array([-0.03593, 0.62489, 0.44932])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
        #                {"index":2, "position": transform(np.array([-0.03593, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.3161, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

        #                {"index":3, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":4, "position": transform(np.array([0.255, -1.04157+0, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.12807-0, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":5, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
        #                {"index":6, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":7, "position": transform(np.array([0.255, -1.04157+0, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.12807-0, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":8, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},

        #                {"index":9, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
        #                {"index":10, "position": transform(np.array([-0.21277, 0.62489, 0.44932])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},
        #                {"index":11, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.1])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.1]), "goal_orientation":np.array([0,0.70711,0,-0.70711])},

        #                {"index":12, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":13, "position": transform(np.array([0.255, -1.04157+0, 0.53925])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.60656, -17.12807-0, 0.78133]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
        #                {"index":14, "position": transform(np.array([0.28749, -1.04157+0, 0.61049])), "orientation": np.array([0, 0.58727, 0, -0.80939]), "goal_position":np.array([-16.63905, -17.12807-0, 0.85257]), "goal_orientation":np.array([0.80939, 0, 0.58727, 0])},
                       
        #                {"index":15, "position": transform(np.array([-0.21277, 0.62489, 0.44932+0.2])), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.13927, -18.79419, 0.69132+0.2]), "goal_orientation":np.array([0,0.70711,0,-0.70711])}]
        self.util.do_screw_driving(motion_plan,"_battery")
        if self.util.motion_task_counter==13:
            print("Done screwing battery")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_remove_battery(self):
        motion_plan = [ {"index":0, "position": np.array([-0.15683+0.05, 0.95185, 0.57477+0.1-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895-0.09326, 0.81657+0.1]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])},
                       {"index":1, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.7179, -17.0181, 1.04897]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10(motion_plan, "_battery")
        if self.util.motion_task_counter==2:
            print("Done arm removal")
            self.util.motion_task_counter=0
            return True
        return False

    def move_mp_to_battery_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
            ["translate", [-16.41, 1, False]]]
        self.util.move_mp_battery(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def moving_part_feeders(self):
        print(self.util.path_plan_counter)
        if not self.bool_done[0]:
            path_plan = [
                ["translate", [-16.41, 1, True]]]
            self.util.move_mp_battery(path_plan)
            if len(path_plan) == self.util.path_plan_counter:
                self.bool_done[0]=True
        if not self.bool_done[1]:
            path_plan = [
                ["translate", [-6.84552, 0, False]]]
            self.util.move_mp_fuel(path_plan)
            if len(path_plan) == self.util.path_plan_counter:
                self.bool_done[1]=True
        if not self.bool_done[2]:
            path_plan = [
                ["translate", [-6.491, 0, False]]]
            self.util.move_mp_suspension(path_plan)
            if len(path_plan) == self.util.path_plan_counter:
                self.bool_done[2]=True
        if not self.bool_done[3]:
            path_plan = [
                ["translate", [-5.07, 0, False]]]
            self.util.move_mp_engine(path_plan)
            if len(path_plan) == self.util.path_plan_counter:
                self.bool_done[3]=True
        
        if self.bool_done[0] and self.bool_done[1] and self.bool_done[2] and self.bool_done[3]:
            return True
        return False
    
    def battery_part_feeding(self):
        motion_plan = [{"index":0, "position": np.array([0.73384, 0.33739, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.66717, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.73384, 0.33739, 0.27353-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.66717, 0.51553]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.73384, 0.33739, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.66717, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       
                       {"index":3, "position": np.array([-0.07017, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.48356, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":4, "position": np.array([-0.07017, -0.70695, 0.4101-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.48356, -15.62279, 0.65211]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":5, "position": np.array([-0.07017, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.48356, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":6, "position": np.array([0.73384, 0.06664, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.39642, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":7, "position": np.array([0.73384, 0.06664, 0.27353-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.39642, 0.51553]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":8, "position": np.array([0.73384, 0.06664, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.39642, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       
                       {"index":9, "position": np.array([-0.36564, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.18809, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":10, "position": np.array([-0.36564, -0.70695, 0.4101-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.18809, -15.62279, 0.65211]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":11, "position": np.array([-0.36564, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-16.18809, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":12, "position": np.array([0.73384, -0.20396, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.12582, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":13, "position": np.array([0.73384, -0.20396, 0.27353-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.12582, 0.51553]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":14, "position": np.array([0.73384, -0.20396, 0.27353-0.16+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.288, -16.12582, 0.51553+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":15, "position": np.array([-0.66681, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-15.88692, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":16, "position": np.array([-0.66681, -0.70695, 0.4101-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-15.88692, -15.62279, 0.65211]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":17, "position": np.array([-0.66681, -0.70695, 0.4101-0.16+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-15.88692, -15.62279, 0.65211+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       
                       {"index":18, "position": np.array([-0.15683+0.05, 0.95185-0.09326, 0.57477-0.16]), "orientation": np.array([0.81202, 0, 0.58362, 0]), "goal_position":np.array([-16.39705, -17.18895, 0.81657]), "goal_orientation":np.array([0, -0.58362, 0, 0.81202])}]
        self.util.move_ur10(motion_plan, "_battery")

        if self.util.motion_task_counter==2 and not self.bool_done[3]:
            self.bool_done[3] = True
            self.util.remove_part("battery_bringer/platform", "battery_06")
            self.util.add_part_custom("World/UR10_battery/ee_link","battery", "qbattery_06", np.array([0.001,0.001,0.001]), np.array([0.2361, 0.05277, 0.03064]), np.array([0, -0.70711, 0.70711, 0]))
        
        if self.util.motion_task_counter==8 and not self.bool_done[4]:
            self.bool_done[4] = True
            self.util.remove_part("battery_bringer/platform", "battery_05")
            self.util.add_part_custom("World/UR10_battery/ee_link","battery", "qbattery_05", np.array([0.001,0.001,0.001]), np.array([0.2361, 0.05277, 0.03064]), np.array([0, -0.70711, 0.70711, 0]))
        
        if self.util.motion_task_counter==14 and not self.bool_done[5]:
            self.bool_done[5] = True
            self.util.remove_part("battery_bringer/platform", "battery_01")
            self.util.add_part_custom("World/UR10_battery/ee_link","battery", "qbattery_01", np.array([0.001,0.001,0.001]), np.array([0.2361, 0.05277, 0.03064]), np.array([0, -0.70711, 0.70711, 0]))
        
        if self.util.motion_task_counter==5 and not self.bool_done[6]:
            self.bool_done[6] = True
            print("Done placing battery 1")
            self.util.remove_part("World/UR10_battery/ee_link", "qbattery_06")
            self.util.add_part_custom("World/Environment","battery", "xbattery_06", np.array([0.001,0.001,0.001]), np.array([-16.53656, -15.59236, 0.41568]), np.array([0.70711, 0.70711, 0, 0]))

        if self.util.motion_task_counter==11 and not self.bool_done[7]:
            self.bool_done[7] = True
            print("Done placing battery 2")
            self.util.remove_part("World/UR10_battery/ee_link", "qbattery_05")
            self.util.add_part_custom("World/Environment","battery", "xbattery_05", np.array([0.001,0.001,0.001]), np.array([-16.23873, -15.59236, 0.41568]), np.array([0.70711, 0.70711, 0, 0]))

        if self.util.motion_task_counter==17 and not self.bool_done[8]:
            self.bool_done[8] = True
            print("Done placing battery 3")
            self.util.remove_part("World/UR10_battery/ee_link", "qbattery_01")
            self.util.add_part_custom("World/Environment","battery", "xbattery_01", np.array([0.001,0.001,0.001]), np.array([-15.942, -15.59236, 0.41568]), np.array([0.70711, 0.70711, 0, 0]))
        
        if self.util.motion_task_counter == 18:
            print("Done placing 3 batterys")
            self.util.motion_task_counter=0
            return True
        return False

    def move_to_trunk_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
            ["translate", [-23.49, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
            ["wait",[]],
            ["translate", [-35.47, 0, False]],
            ["wait",[]],
            ["rotate", [np.array([-0.70711, 0, 0, -0.70711]), 0.0042, True]],
            ["wait",[]],
            ["translate", [5.35, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([1, 0, 0, 0]), 0.0032, True]],
            ["wait",[]],
            ["translate", [-26.75, 0, False]]]
        if self.id==1:
            path_plan = [
            ["translate", [-23.49, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
            ["wait",[]],
            ["translate", [-35.47, 0, False]],
            ["wait",[]],
            ["rotate", [np.array([-0.70711, 0, 0, -0.70711]), 0.0042, True]],
            ["wait",[]],
            ["translate", [5.45, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([1, 0, 0, 0]), 0.0032, True]],
            ["wait",[]],
            ["translate", [-26.75, 0, False]]]
        if self.id==2:
            path_plan = [
            ["translate", [-23.49, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
            ["wait",[]],
            ["translate", [-35.47, 0, False]],
            ["wait",[]],
            ["rotate", [np.array([-0.70711, 0, 0, -0.70711]), 0.0042, True]],
            ["wait",[]],
            ["translate", [5.45, 1, False]],
            ["wait",[]],
            ["rotate", [np.array([1, 0, 0, 0]), 0.0032, True]],
            ["wait",[]],
            ["translate", [-26.75, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_trunk(self):
        motion_plan = [{"index":0, "position": np.array([0.9596, 0.21244, -0.16+0.4547+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.9596, 0.21244, -0.16+0.4547]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.9596, 0.21244, -0.16+0.4547+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-28.06, 4.27349, 0.69642+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":3, "position": np.array([0.74919, -0.50484, -0.16+0.64833]), "orientation": np.array([-0.20088, -0.67797, -0.20088, 0.67797]), "goal_position":np.array([-27.85221, 4.99054, 0.89005]), "goal_orientation":np.array([0.67797, -0.20088, 0.67797, 0.20088])},

                        # {"index":4, "position": np.array([0.41663, -0.77637, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.519, 5.262, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":4, "position": np.array([0.55084, -0.81339+0.02396+0, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.65404, 5.26-0.02396-0, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":5, "position": np.array([0.42543, -0.81339+0.02396+0, -0.16+0.75942]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.52862, 5.26-0.02396-0, 1.00113]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])}]
        self.util.move_ur10(motion_plan, "_trunk")

        if self.util.motion_task_counter==2 and not self.bool_done[5]:
            self.bool_done[5] = True
            self.util.remove_part("World/Environment", f"trunk_02_{self.id}")
            self.util.add_part_custom("World/UR10_trunk/ee_link","trunk", f"qtrunk_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.28167, -0.21084, -0.00861]), np.array([0.70711, 0, 0, 0.70711]))
        
        if self.util.motion_task_counter==6:
            print("Done placing trunk")
            self.util.motion_task_counter=0
            self.util.remove_part("World/UR10_trunk/ee_link", f"qtrunk_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","trunk", f"xtrunk_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.79319, -0.21112, 0.70114]), np.array([0.5, 0.5, 0.5, 0.5]))
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
                       
                       {"index":3, "position": transform(np.array([-0.12592, 1.126+0.16-0.24329+0.02396+0.11321-0.2, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.66311-0.02396-0.11321+0.2, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       {"index":4, "position": transform(np.array([-0.12592, 1.33575+0.16-0.24329+0.02396+0.11321-0.2, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.45335-0.02396-0.11321+0.2, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       {"index":5, "position": transform(np.array([-0.12592, 1.126+0.16-0.24329+0.02396+0.11321-0.2, 0.48602])), "orientation": np.array([0.5, 0.5, 0.5, 0.5]), "goal_position":np.array([-27.17401, 5.66311-0.02396-0.11321+0.2, 0.7279]), "goal_orientation":np.array([0.5, 0.5, -0.5, -0.5])},
                       
                       {"index":6, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.4637, 5.85911, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.do_screw_driving(motion_plan,"_trunk")
        if self.util.motion_task_counter==7:
            print("Done screwing trunk")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_remove_trunk(self):
        motion_plan = [{"index":0, "position": np.array([0.42543, -0.81339+0.02396+0, -0.16+0.75942+0.1]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-27.52862, 5.26-0.02396-0, 1.00113+0.1]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":1, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.2673, 3.79761, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10(motion_plan, "_trunk")
        if self.util.motion_task_counter==2:
            print("Done arm removal")
            self.util.motion_task_counter=0
            return True
        return False
    
    def move_to_wheel_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
                    ["translate", [-21.3, 0, False]],
                    ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, -0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [9.3, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([-1, 0, 0, 0]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-16.965, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, True]],
                     ["wait",[]],
                    #  ["translate", [5.39, 1, False]],
                    ["translate", [6, 1, False]]
                     ]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_wheel_01(self):
        motion_plan = [{"index":0, "position": np.array([0.86671, -0.02468, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([0.86671, -0.02468, -0.16+0.4353]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661, 0.67666]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([0.86671, -0.02468, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":3, "position": np.array([0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.935, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":4, "position": np.array([-0.39779, 0.19418, -0.16+0.51592]), "orientation": np.array([0.62501, -0.3307, 0.62501, 0.3307]), "goal_position":np.array([-17.53, 4.68, 0.758]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":5, "position": np.array([-0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.572, 5.04127, 0.723]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":6, "position": np.array([-0.48413-0.16, -0.28768, 0.29642]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.446, 5.16, 0.537]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":7, "position": np.array([-0.27412-0.16, -0.61531, 0.25146]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.656, 5.497, 0.492]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":8, "position": np.array([-0.3-0.16, -0.75, 0.2]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.62, 5.63, 0.44]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},


                       {"index":4, "position": np.array([-0.33031-0.16, -0.78789, 0.15369]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.596, 5.671, 0.394]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":5, "position": np.array([-0.49798-0.16, -0.78789, 0.15369]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.42945, 5.671, 0.394]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":6, "position": np.array([-0.33031-0.16, -0.78789, 0.15369]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.596, 5.671, 0.394]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},

                       {"index":7, "position": np.array([-0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.572, 5.04127, 0.723]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":8, "position": np.array([0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.935, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                       {"index":9, "position": np.array([0.16345, 0.69284, 0.62942-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.09182, 4.18911, 0.87105]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}]

        self.util.move_ur10(motion_plan, "_wheel_01")

        if self.util.motion_task_counter==2 and not self.bool_done[45]:
            self.bool_done[45] = True
            self.util.remove_part("World/Environment", f"wheel_03_{self.id}")
            self.util.add_part_custom("World/UR10_wheel_01/ee_link","FWheel", f"qwheel_03_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.25604, -0.18047, -0.18125]), np.array([0, 0, 0.70711, 0.70711]))
        
        if self.util.motion_task_counter==5 and not self.bool_done[6]:
            self.bool_done[6] = True
            print("Done placing wheel")
            self.util.remove_part("World/UR10_wheel_01/ee_link", f"qwheel_03_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FWheel", f"xwheel_03_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.15255, -0.1948, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counter==10:
            self.util.motion_task_counter=0
            return True
        return False

    def screw_wheel_01(self):
        motion_plan = [{"index":0, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([0.67966, -0.08619, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":3, "position": np.array([-0.49561+0.1-0.16, 0.61097, 0.22823]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.68598, 0.46965]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":4, "position": np.array([-0.49561-0.16, 0.61097, 0.22823]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.68598, 0.46965]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":5, "position": np.array([-0.49561+0.1-0.16, 0.61097, 0.22823]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.68598, 0.46965]), "goal_orientation":np.array([0, -1, 0, 0])},

                       {"index":6, "position": np.array([0.67966, 0.09013, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([0.67966, 0.09013, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":8, "position": np.array([0.67966, 0.09013, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":9, "position": np.array([-0.49561+0.1-0.16, 0.66261, 0.23808]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.63434, 0.47951]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":10, "position": np.array([-0.49561-0.16, 0.66261, 0.23808]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.63434, 0.47951]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":11, "position": np.array([-0.49561+0.1-0.16, 0.66261, 0.23808]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.63434, 0.47951]), "goal_orientation":np.array([0, -1, 0, 0])},

                       {"index":12, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":13, "position": np.array([0.67966, -0.08619, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":14, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":15, "position": np.array([-0.49561+0.1-0.16, 0.6234, 0.27624]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.67355, 0.51766]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":16, "position": np.array([-0.49561-0.16, 0.6234, 0.27624]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.67355, 0.51766]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":17, "position": np.array([-0.49561+0.1-0.16, 0.6234, 0.27624]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.67355, 0.51766]), "goal_orientation":np.array([0, -1, 0, 0])},
                       
                       {"index":18, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}]
        self.util.do_screw_driving(motion_plan,"_wheel_01")
        if self.util.motion_task_counter==19:
            print("Done screwing wheel")
            self.util.motion_task_counter=0
            return True
        return False
    
    def arm_place_wheel(self):
        print("here")
        motion_plan = [{"index":0, "position": np.array([-0.86671, -0.02468, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.86671, -0.02468, -0.16+0.4353]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661, 0.67666]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.86671, -0.02468, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":3, "position": np.array([-0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-15.858, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":4, "position": np.array([-0.39779, 0.19418, -0.16+0.51592]), "orientation": np.array([0.62501, -0.3307, 0.62501, 0.3307]), "goal_position":np.array([-17.53, 4.68, 0.758]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":5, "position": np.array([-0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.572, 5.04127, 0.723]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":6, "position": np.array([-0.48413-0.16, -0.28768, 0.29642]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.446, 5.16, 0.537]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":7, "position": np.array([-0.27412-0.16, -0.61531, 0.25146]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.656, 5.497, 0.492]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":4, "position": np.array([0.5+0.16, -0.112, 0.410]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.364, 5, 0.651]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":5, "position": np.array([0.46203+0.16, -0.76392, 0.15278]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.32833, 5.65751, 0.3945]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":6, "position": np.array([0.65863+0.16, -0.76392, 0.15278]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.52493, 5.65751, 0.3945]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                       {"index":7, "position": np.array([0.46203+0.16, -0.76392, 0.15278]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.32833, 5.65751, 0.3945]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},

                       {"index":8, "position": np.array([0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-16.221, 5.05282, 0.725]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":9, "position": np.array([0.16394, 0.68797, 0.64637]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.0297, 4.20519, 1.04836]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}]
        self.util.move_ur10_extra(motion_plan, "_wheel")

        if self.util.motion_task_counterl==2 and not self.bool_done[7]:
            self.bool_done[7] = True
            self.util.remove_part("World/Environment", f"wheel_01_{self.id}")
            self.util.add_part_custom("World/UR10_wheel/ee_link","FWheel", f"qwheel_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.25604, -0.18047, -0.18125]), np.array([0, 0, 0.70711, 0.70711]))
        
        if self.util.motion_task_counterl==7 and not self.bool_done[8]:
            self.bool_done[8] = True
            print("Done placing wheel")
            self.util.remove_part("World/UR10_wheel/ee_link", f"qwheel_01_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FWheel", f"xwheel_01_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.1522, 0.33709, 0.56377]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counterl==10:
            self.util.motion_task_counterl=0
            return True
        return False

    def screw_wheel(self):
        motion_plan = [{"index":0, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([-0.67966, -0.08051, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":3, "position": np.array([0.67955-0.1+0.16, 0.62439, 0.22894]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.67963, 0.4704]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":4, "position": np.array([0.67955+0.16, 0.62439, 0.22894]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.67963, 0.4704]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":5, "position": np.array([0.67955-0.1+0.16, 0.62439, 0.22894]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.67963, 0.4704]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":6, "position": np.array([-0.67572, 0.09613, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([-0.67572, 0.09613, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":8, "position": np.array([-0.67572, 0.09613, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":9, "position": np.array([0.67955-0.1+0.16, 0.67241, 0.23225]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.6316, 0.47372]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":10, "position": np.array([0.67955+0.16, 0.67241, 0.23225]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.6316, 0.47372]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":11, "position": np.array([0.67955-0.1+0.16, 0.67241, 0.23225]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.6316, 0.47372]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":12, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":13, "position": np.array([-0.67966, -0.08051, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":14, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":15, "position": np.array([0.67955-0.1+0.16, 0.64605, 0.27773]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.65797, 0.51919]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":16, "position": np.array([0.67955+0.16, 0.64605, 0.27773]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.65797, 0.51919]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":17, "position": np.array([0.67955-0.1+0.16, 0.64605, 0.27773]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.65797, 0.51919]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":18, "position": np.array([-0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}]
        self.util.do_screw_driving_extra(motion_plan,"_wheel")
        if self.util.motion_task_counterl==19:
            print("Done screwing wheel")
            self.util.motion_task_counterl=0
            return True
        return False
    
    def move_ahead_in_wheel_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
                    ["translate", [5.04, 1, False]]
                     ]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False
    
    def arm_place_wheel_03(self):
        motion_plan = [{"index":0, "position": np.array([0.86671, -0.54558, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661+0.521, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                        {"index":1, "position": np.array([0.86671, -0.54558, -0.16+0.4353]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661+0.521, 0.67666]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                        {"index":2, "position": np.array([0.86671, -0.54558, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.79517, 4.90661+0.521, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                        {"index":3, "position": np.array([0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.935, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},

                        {"index":4, "position": np.array([-0.31937-0.16, -0.78789, 0.0258]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.596, 5.671, 0.274]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                        {"index":5, "position": np.array([-0.47413-0.16, -0.78789, 0.0258]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.42945, 5.671, 0.274]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                        {"index":6, "position": np.array([-0.31937-0.16, -0.78789, 0.0258]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.596, 5.671, 0.274]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},

                        {"index":7, "position": np.array([-0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.572, 5.04127, 0.723]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                        {"index":8, "position": np.array([0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.935, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        {"index":9, "position": np.array([0.16345, 0.69284, 0.62942-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.09182, 4.18911, 0.87105]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}]
        self.util.move_ur10(motion_plan, "_wheel_01")

        if self.util.motion_task_counter==2 and not self.bool_done[9]:
            self.bool_done[9] = True
            self.util.remove_part("World/Environment", f"wheel_04_{self.id}")
            self.util.add_part_custom("World/UR10_wheel_01/ee_link","FWheel", f"qwheel_04_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.25604, -0.18047, -0.18125]), np.array([0, 0, 0.70711, 0.70711]))
        
        if self.util.motion_task_counter==5 and not self.bool_done[10]:
            self.bool_done[10] = True
            print("Done placing wheel")
            self.util.remove_part("World/UR10_wheel_01/ee_link", f"qwheel_04_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FWheel", f"xwheel_04_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.80845, -0.22143, 0.43737]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counter==10:
            self.util.motion_task_counter=0
            return True
        return False

    def screw_wheel_03(self):
        motion_plan = [{"index":0, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([0.67966, -0.08619, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":3, "position": np.array([-0.49561+0.1-0.16, 0.61097, 0.22823-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.68598, 0.46965-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":4, "position": np.array([-0.49561-0.16, 0.61097, 0.22823-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.68598, 0.46965-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":5, "position": np.array([-0.49561+0.1-0.16, 0.61097, 0.22823-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.68598, 0.46965-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},

                       {"index":6, "position": np.array([0.67966, 0.09013, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([0.67966, 0.09013, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":8, "position": np.array([0.67966, 0.09013, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.20682, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":9, "position": np.array([-0.49561+0.1-0.16, 0.66261, 0.23808-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.63434, 0.47951-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":10, "position": np.array([-0.49561-0.16, 0.66261, 0.23808-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.63434, 0.47951-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":11, "position": np.array([-0.49561+0.1-0.16, 0.66261, 0.23808-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.63434, 0.47951-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},

                       {"index":12, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":13, "position": np.array([0.67966, -0.08619, -0.16+0.44283]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":14, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":15, "position": np.array([-0.49561+0.1-0.16, 0.6234, 0.27624-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.67355, 0.51766-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":16, "position": np.array([-0.49561-0.16, 0.6234, 0.27624-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516, 5.67355, 0.51766-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       {"index":17, "position": np.array([-0.49561+0.1-0.16, 0.6234, 0.27624-0.12]), "orientation": np.array([0,0,1,0]), "goal_position":np.array([-17.43516-0.1, 5.67355, 0.51766-0.12]), "goal_orientation":np.array([0, -1, 0, 0])},
                       
                       {"index":18, "position": np.array([0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.61058, 6.38314, 0.68464+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}]
        self.util.do_screw_driving(motion_plan,"_wheel_01")
        if self.util.motion_task_counter==19:
            print("Done screwing wheel")
            self.util.motion_task_counter=0
            return True
        return False

    def arm_place_wheel_02(self):
        tire_offset = 0.52214
        motion_plan = [{"index":0, "position": np.array([-0.86671, -0.53748, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661+0.512, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":1, "position": np.array([-0.86671, -0.53748, -0.16+0.4353]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661+0.512, 0.67666]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":2, "position": np.array([-0.86671, -0.53748, -0.16+0.4353+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.99517, 4.90661+0.512, 0.67666+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":3, "position": np.array([-0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-15.858, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":4, "position": np.array([-0.39779, 0.19418, -0.16+0.51592]), "orientation": np.array([0.62501, -0.3307, 0.62501, 0.3307]), "goal_position":np.array([-17.53, 4.68, 0.758]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":5, "position": np.array([-0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.572, 5.04127, 0.723]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":6, "position": np.array([-0.48413-0.16, -0.28768, 0.29642]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.446, 5.16, 0.537]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":7, "position": np.array([-0.27412-0.16, -0.61531, 0.25146]), "orientation": np.array([0, 0, 0.70711, 0.70711]), "goal_position":np.array([-17.656, 5.497, 0.492]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":4, "position": np.array([0.5+0.16, -0.112, 0.410]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.364, 5, 0.651]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":5, "position": np.array([0.46203+0.16, -0.76392, 0.0258]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.32833, 5.65751, 0.274]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},
                       {"index":6, "position": np.array([0.65863+0.16, -0.76392, 0.0258]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.52493, 5.65751, 0.274]), "goal_orientation":np.array([0, 0, 0.70711, 0.70711])},
                       {"index":7, "position": np.array([0.46203+0.16, -0.76392, 0.0258]), "orientation": np.array([0.70711, 0.70711, 0, 0]), "goal_position":np.array([-16.32833, 5.65751, 0.274]), "goal_orientation":np.array([0.70711, 0.70711, 0, 0])},

                       {"index":8, "position": np.array([0.35597, -0.15914, -0.16+0.48217]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-16.221, 5.05282, 0.725]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       
                       {"index":9, "position": np.array([-0.00762, 0.77686, -0.16+0.48217]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-15.858, 4.105, 0.723]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},

                       {"index":10, "position": np.array([0.16286, 0.68548, 0.63765-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.02865, 4.20818, 0.87901]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":11, "position": np.array([-0.87307, -0.01687-tire_offset, 0.436-0.16+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-14.9927, 4.91057+tire_offset, 0.67736+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}
                    ]
        self.util.move_ur10_extra(motion_plan, "_wheel")

        if self.util.motion_task_counterl==2 and not self.bool_done[11]:
            self.bool_done[11] = True
            self.util.remove_part("World/Environment", f"wheel_02_{self.id}")
            self.util.add_part_custom("World/UR10_wheel/ee_link","FWheel", f"qwheel_02_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.25604, -0.18047, -0.18125]), np.array([0, 0, 0.70711, 0.70711]))
        
        if self.util.motion_task_counterl==7 and not self.bool_done[12]:
            self.bool_done[12] = True
            print("Done placing wheel")
            self.util.remove_part("World/UR10_wheel/ee_link", f"qwheel_02_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FWheel", f"xwheel_02_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.80934, 0.35041, 0.43888]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counterl==11:
            self.util.motion_task_counterl=0
            return True
        return False

    def screw_wheel_02(self):
        motion_plan = [{"index":0, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([-0.67966, -0.08051, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":3, "position": np.array([0.67955-0.1+0.16, 0.62439, 0.22894-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.67963, 0.4704-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":4, "position": np.array([0.67955+0.16, 0.62439, 0.22894-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.67963, 0.4704-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":5, "position": np.array([0.67955-0.1+0.16, 0.62439, 0.22894-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.67963, 0.4704-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":6, "position": np.array([-0.67572, 0.09613, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([-0.67572, 0.09613, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":8, "position": np.array([-0.67572, 0.09613, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18675, 6.20788, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":9, "position": np.array([0.67955-0.1+0.16, 0.67241, 0.23225-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.6316, 0.47372-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":10, "position": np.array([0.67955+0.16, 0.67241, 0.23225-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.6316, 0.47372-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":11, "position": np.array([0.67955-0.1+0.16, 0.67241, 0.23225-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.6316, 0.47372-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":12, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":13, "position": np.array([-0.67966, -0.08051, -0.16+0.44283]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":14, "position": np.array([-0.67966, -0.08051, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
            
                       {"index":15, "position": np.array([0.67955-0.1+0.16, 0.64605, 0.27773-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.65797, 0.51919-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":16, "position": np.array([0.67955+0.16, 0.64605, 0.27773-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236, 5.65797, 0.51919-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},
                       {"index":17, "position": np.array([0.67955-0.1+0.16, 0.64605, 0.27773-0.12]), "orientation": np.array([0,-1,0,0]), "goal_position":np.array([-16.54236+0.1, 5.65797, 0.51919-0.12]), "goal_orientation":np.array([0, 0, 1, 0])},

                       {"index":18, "position": np.array([-0.67966, -0.08619, -0.16+0.44283+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-15.18282, 6.38452, 0.68279+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}]
        self.util.do_screw_driving_extra(motion_plan,"_wheel")
        if self.util.motion_task_counterl==19:
            print("Done screwing wheel")
            self.util.motion_task_counterl=0
            return True
        return False
    
    def arm_place_fwheel_together(self):

        if not self.right_side:
            self.right_side = self.arm_place_wheel_01()
        if not self.left_side:
            self.left_side = self.arm_place_wheel()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def screw_fwheel_together(self):

        if not self.right_side:
            self.right_side = self.screw_wheel_01()
        if not self.left_side:
            self.left_side = self.screw_wheel()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def arm_place_bwheel_together(self):

        if not self.right_side:
            self.right_side = self.arm_place_wheel_03()
        if not self.left_side:
            self.left_side = self.arm_place_wheel_02()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def screw_bwheel_together(self):

        if not self.right_side:
            self.right_side = self.screw_wheel_03()
        if not self.left_side:
            self.left_side = self.screw_wheel_02()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def move_to_lower_cover_cell(self):
        print(self.util.path_plan_counter)
        # path_plan = [
        #              ["rotate", [np.array([0.73548, 0, 0, -0.67755]), 0.0042, False]],
        #              ["translate", [-0.64, 1, False]],
        #              ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, True]],
        #              ["translate", [-12.037, 1, False]],
        #              ["rotate", [np.array([0, 0, 0, -1]), 0.0042, True]],
        #              ["translate", [-20.15, 0, False]],
        #              ["rotate", [np.array([0.70711, 0, 0, -0.70711]), 0.0042, False]],
        #              ["translate", [-17, 1, False]],
        #              ["rotate", [np.array([0, 0, 0, 1]), 0.0042, True]],
        #              ["translate", [-26.9114, 0, False]]]
        path_plan = [
                     ["rotate", [np.array([-0.73548, 0, 0, 0.67755]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-0.64, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-12.1, 1, False]],
                     ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-21.13755, 0, False]], # 20.2 earlier
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-17.1, 1, False]],
                    ["translate", [-17.34, 1, False]],
                    ["wait",[]],
                     ["rotate", [np.array([0, 0, 0, 1]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-26.9114, 0, False]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_lower_cover(self):
        motion_plan = [
                    #    {"index":0, "position": np.array([-0.49105, 0.76464, -0.16+0.4434+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.54994, -15.49436, 0.6851+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                    #    {"index":1, "position": np.array([-0.49105, 0.76464, -0.16+0.4434]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.54994, -15.49436, 0.6851]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                    #    {"index":2, "position": np.array([-0.49105, 0.76464, -0.16+0.4434+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.54994, -15.49436, 0.6851+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                        *self.lower_cover[0],
                        {"index":3, "position": np.array([-0.70739, -0.00943, -0.16+0.52441]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-26.76661, -16.26838, 0.76611]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                        {"index":4, "position": np.array([-0.59828, -0.85859, -0.16+0.36403+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-26.65756, -17.11785, 0.60573+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        {"index":5, "position": np.array([-0.59828, -0.85859, -0.16+0.36403]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-26.65756, -17.11785, 0.60573]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        {"index":6, "position": np.array([-0.59828, -0.85859, -0.16+0.36403+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-26.65756, -17.11785, 0.60573+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        
                        {"index":7, "position": np.array([-0.49105, 0.76464, -0.16+0.4434+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.54994, -15.49436, 0.6851+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        # {"index":8, "position": np.array([0.16286, 0.68548, 0.63765-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-25.8962, -15.5737, 0.879938]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}
                        {"index":8, "position": np.array([0.54581, 0.04547, 0.73769-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-25.5133, -16.2136, 0.9801]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}
                        ]
        
        self.util.move_ur10(motion_plan, "_lower_cover")

        if self.util.motion_task_counter==2 and not self.bool_done[20]:
            self.bool_done[20] = True
            self.util.remove_part("World/Environment", f"lower_coverr_{self.id}")
            self.util.add_part_custom("World/UR10_lower_cover/ee_link","lower_cover", f"qlower_coverr_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.27893, -0.08083, 0.29584]), np.array([0.5, -0.5, 0.5, 0.5]))
        
        if self.util.motion_task_counter==6 and not self.bool_done[21]:
            self.bool_done[21] = True
            print("Done placing right lower cover")
            self.util.remove_part("World/UR10_lower_cover/ee_link", f"qlower_coverr_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","lower_cover", f"xlower_coverr_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.03325, -0.29278, 0.31255]), np.array([0, 0, 0.70711, 0.70711]))
            # np.array([435.65021, 418.57531,21.83379]), np.array([0.50942, 0.50942,0.4904, 0.4904])
        if self.util.motion_task_counter==9:
            self.util.motion_task_counter=0
            return True
        return False
    
    def screw_lower_cover(self):
        motion_plan = [{"index":0, "position": np.array([0.00434, 0.77877, -0.16+0.43196+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.48421, -15.4803, 0.67396+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":1, "position": np.array([0.00434, 0.77877, -0.16+0.43196]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.48421, -15.4803, 0.67396]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":2, "position": np.array([0.00434, 0.77877, -0.16+0.43196+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.48421, -15.4803, 0.67396+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                       {"index":3, "position": np.array([0.71996, -0.73759, -0.16+0.26658+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -16.99645, 0.50858+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":4, "position": np.array([0.71996, -0.73759, -0.16+0.26658]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -16.99645, 0.50858]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":5, "position": np.array([0.71996, -0.73759, -0.16+0.26658+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -16.99645, 0.50858+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":6, "position": np.array([-0.17139, 0.78152, -0.16+0.43196+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.65994, -15.47755, 0.67396+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":7, "position": np.array([-0.17139, 0.78152, -0.16+0.43196]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.65994, -15.47755, 0.67396]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                       {"index":8, "position": np.array([-0.17139, 0.78152, -0.16+0.43196+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.65994, -15.47755, 0.67396+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                       {"index":9, "position": np.array([0.71996, -0.80562, -0.16+0.26658+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -17.06448, 0.50858+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":10, "position": np.array([0.71996, -0.80562, -0.16+0.26658]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -17.06448, 0.50858]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":11, "position": np.array([0.71996, -0.80562, -0.16+0.26658+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76846, -17.06448, 0.50858+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":12, "position": np.array([0.00434, 0.77877, -0.16+0.43196+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.48421, -15.4803, 0.67396+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])}]
        self.util.do_screw_driving(motion_plan,"_lower_cover")
        if self.util.motion_task_counter==13:
            print("Done screwing right lower cover")
            self.util.motion_task_counter=0
            return True
        return False
    
    def arm_place_lower_cover_01(self):
        motion_plan = [
                    #    {"index":0, "position": np.array([0.49458, 0.74269, -0.16+0.44428+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.55383, -19.06174, 0.68566+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":1, "position": np.array([0.49458, 0.74269, -0.16+0.44428]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.55383, -19.06174, 0.68566]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                    #    {"index":2, "position": np.array([0.49458, 0.74269, -0.16+0.44428+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.55383, -19.06174, 0.68566+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        *self.lower_cover[1],

                        {"index":3, "position": np.array([0.79817, 0.02534, -0.16+0.58377]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.85719, -18.34429, 0.82514]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                        {"index":4, "position": np.array([0.59724, -0.78253, -0.16+0.36033+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-26.65614, -17.53666, 0.6017+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":5, "position": np.array([0.59724, -0.78253, -0.16+0.36033]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-26.65614, -17.53666, 0.6017]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":6, "position": np.array([0.59724, -0.78253, -0.16+0.36033+0.2]), "orientation": np.array([0, -0.70711, 0, 0.70711]), "goal_position":np.array([-26.65614, -17.53666, 0.6017+0.2]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        
                        {"index":7, "position": np.array([0.49458, 0.74269, -0.16+0.44428+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.55383, -19.06174, 0.68566+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                        # {"index":8, "position": np.array([0.16286, 0.68548, 0.63765-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-26.222, -19.0046, 0.879939]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}
                        {"index":8, "position": np.array([-0.54581, 0.04547, 0.73769-0.16]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-25.513, -18.3647, 0.98016]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])}
                        ]
        self.util.move_ur10_extra(motion_plan, "_lower_cover_01")

        if self.util.motion_task_counterl==2 and not self.bool_done[22]:
            self.bool_done[22] = True
            self.util.remove_part("World/Environment", f"lower_coverl_{self.id}")
            self.util.add_part_custom("World/UR10_lower_cover_01/ee_link","lower_cover", f"qlower_coverl_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.28058, 0.07474, -0.29292]), np.array([0.5, 0.5, -0.5, 0.5]))
        
        if self.util.motion_task_counterl==6 and not self.bool_done[23]:
            self.bool_done[23] = True
            print("Done placing right lower cover")
            self.util.remove_part("World/UR10_lower_cover_01/ee_link", f"qlower_coverl_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","lower_cover", f"xlower_coverl_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.03589, 0.13349, 0.30227]), np.array([0, 0, -0.70711, -0.70711]))

        if self.util.motion_task_counterl==9:
            self.util.motion_task_counterl=0
            return True
        return False
    
    def screw_lower_cover_01(self):
        motion_plan = [{"index":0, "position": np.array([-0.00975, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.46819, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([-0.00975, 0.77293, -0.16+0.42757]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.46819, -19.08897, 0.66977]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([-0.00975, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.46819, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":3, "position": np.array([-0.71345, -0.63679, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":4, "position": np.array([-0.71345, -0.63679, -0.16+0.26949]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681, 0.51169]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":5, "position": np.array([-0.71345, -0.63679, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":6, "position": np.array([-0.18665, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.29128, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([-0.18665, 0.77293, -0.16+0.42757]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.29128, -19.08897, 0.66977]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":8, "position": np.array([-0.18665, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.29128, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":9, "position": np.array([-0.00975, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.46819, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":10, "position": np.array([-0.71345, -0.63679, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                        {"index":11, "position": np.array([-0.71345, -0.68679, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681+0.05, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":12, "position": np.array([-0.71345, -0.68679, -0.16+0.26949]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681+0.05, 0.51169]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":13, "position": np.array([-0.71345, -0.68679, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.681+0.05, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                    #    {"index":9, "position": np.array([-0.71345, -0.70416, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.61363, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":10, "position": np.array([-0.71345, -0.70416, -0.16+0.26949]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.61363, 0.51169]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":11, "position": np.array([-0.71345, -0.70416, -0.16+0.26949+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-26.76467, -17.61363, 0.51169+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":14, "position": np.array([-0.00975, 0.77293, -0.16+0.42757+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-27.46819, -19.08897, 0.66977+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}]
        self.util.do_screw_driving_extra(motion_plan,"_lower_cover_01")
        if self.util.motion_task_counterl==15:
            print("Done screwing right lower cover")
            self.util.motion_task_counterl=0
            return True
        return False
    
    def arm_place_lower_cover_together(self):
        if not self.right_side:
            self.right_side = self.arm_place_lower_cover()
        if not self.left_side:
            self.left_side = self.arm_place_lower_cover_01()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def screw_lower_cover_together(self):
        if not self.right_side:
            self.right_side = self.screw_lower_cover()
        if not self.left_side:
            self.left_side = self.screw_lower_cover_01()

        if self.left_side and self.right_side:
            self.left_side = self.right_side = False
            return True
        return False
    
    def move_to_main_cover_cell(self):
        print(self.util.path_plan_counter)
        # path_plan = [
        #              ["translate", [-27.9, 0, False]],
        #              ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
        #              ["translate", [-17.18, 1, True]]]
        path_plan = [
                     ["translate", [-28.6, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-17.18, 1, True]]]
        if self.id == 0:
            path_plan = [
                     ["translate", [-28.7, 0, False]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-17.18, 1, True]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_main_cover(self):
        motion_plan = [
                        # {'index': 15, 'position': np.array([-0.74286,  0.42878,  0.35038]), 'orientation': np.array([ 0.6511, -0.2758,  0.6511,  0.2758]), 'goal_position': np.array([-29.14515, -16.32381,   1.33072]), 'goal_orientation': np.array([ 0.65542,  0.26538,  0.65542, -0.26538])}, 
                        # {'index': 16, 'position': np.array([-0.89016,  0.32513,  0.35038]), 'orientation': np.array([ 0.60698, -0.36274,  0.60698,  0.36274]), 'goal_position': np.array([-29.24913, -16.1764 ,   1.33072]), 'goal_orientation': np.array([ 0.68569,  0.1727 ,  0.68569, -0.1727 ])}, 
                        # {'index': 17, 'position': np.array([-1.09352, -0.27789,  0.42455]), 'orientation': np.array([ 0.5, -0.5,  0.5,  0.5]), 'goal_position': np.array([-29.85252, -15.97435,   1.40075]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])}, 
            
                        {"index":0, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444-11.83808, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":1, "position": np.array([-1.09352, -0.27789, 0.19772-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444-11.83808, -15.97435, 1.01392]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        {"index":2, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444-11.83808, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},

                        {"index":3, "position": np.array([-0.89016, 0.32513, 0.51038-0.16]), "orientation": np.array([0.60698, -0.36274, 0.60698, 0.36274]), "goal_position":np.array([-17.41105-11.83808, -16.1764, 1.33072]), "goal_orientation":np.array([0.68569, 0.1727, 0.68569, -0.1727])},
                        {"index":4, "position": np.array([-0.74286, 0.42878, 0.51038-0.16]), "orientation": np.array([0.6511, -0.2758, 0.6511, 0.2758]), "goal_position":np.array([-17.30707-11.83808, -16.32381, 1.33072]), "goal_orientation":np.array([0.65542, 0.26538, 0.65542, -0.26538])},
                        {"index":5, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
                        {"index":6, "position": np.array([-0.28875, 0.74261, 0.51038-0.16]), "orientation": np.array([0.70458, -0.0597, 0.70458, 0.0597]), "goal_position":np.array([-16.99268-11.83808, -16.77844, 1.33072]), "goal_orientation":np.array([0.54043, 0.456, 0.54043, -0.456])},
                        
                        {"index":7, "position": np.array([0.11095-0.11, 0.94, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":8, "position": np.array([0.11095-0.11, 0.94, 0.2926-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.11226]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":9, "position": np.array([0.11095-0.11, 0.94, 0.19682-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 1.01648]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":10, "position": np.array([0.11095-0.11, 0.94, 0.15697-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.97663]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":11, "position": np.array([0.11095-0.11, 0.94, 0.11895-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.93861]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":12, "position": np.array([0.11095-0.11, 0.94, 0.07882-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995+0.11, 0.89848]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},

                        {"index":13, "position": np.array([0.11095, 0.94627, 0.49096-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-16.79175-11.83808, -17.17995, 1.31062]), "goal_orientation":np.array([0.5,0.5,0.5,-0.5])},
                        {"index":14, "position": np.array([-0.28875, 0.74261, 0.51038-0.16]), "orientation": np.array([0.70458, -0.0597, 0.70458, 0.0597]), "goal_position":np.array([-16.99268-11.83808, -16.77844, 1.33072]), "goal_orientation":np.array([0.54043, 0.456, 0.54043, -0.456])},
                        {"index":15, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
                        # {"index":16, "position": np.array([-0.74286, 0.42878, 0.51038-0.16]), "orientation": np.array([0.6511, -0.2758, 0.6511, 0.2758]), "goal_position":np.array([-17.30707-11.83808, -16.32381, 1.33072]), "goal_orientation":np.array([0.65542, 0.26538, 0.65542, -0.26538])},
                        # {"index":17, "position": np.array([-0.89016, 0.32513, 0.51038-0.16]), "orientation": np.array([0.60698, -0.36274, 0.60698, 0.36274]), "goal_position":np.array([-17.41105-11.83808, -16.1764, 1.33072]), "goal_orientation":np.array([0.68569, 0.1727, 0.68569, -0.1727])},
                        
                        # {"index":18, "position": np.array([-1.09352, -0.27789, 0.58455-0.16]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-18.01444-11.83808, -15.97435, 1.40075]), "goal_orientation":np.array([0.70711, 0, 0.70711, 0])},
                        # {'index': 19, 'position': np.array([-0.89016,  0.32513,  0.35038]), 'orientation': np.array([ 0.60698, -0.36274,  0.60698,  0.36274]), 'goal_position': np.array([-29.24913, -16.1764 ,   1.33072]), 'goal_orientation': np.array([ 0.68569,  0.1727 ,  0.68569, -0.1727 ])}, 
                        # {'index': 20, 'position': np.array([-0.74286,  0.42878,  0.35038]), 'orientation': np.array([ 0.6511, -0.2758,  0.6511,  0.2758]), 'goal_position': np.array([-29.14515, -16.32381,   1.33072]), 'goal_orientation': np.array([ 0.65542,  0.26538,  0.65542, -0.26538])}, 
                        # {"index": 21, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.6954, -0.12814, 0.6954, 0.12814]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])},
                        {"index": 16, "position": np.array([-0.5015, 0.55795, 0.51038-0.16]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.17748-11.83808, -16.5655, 1.33072]), "goal_orientation":np.array([0.58233, 0.40111, 0.58233, -0.40111])}]
                        # {'index': 22, 'position': np.array([0.16394, 0.68799,  0.44663]), 'orientation': np.array([ 0.70711, 0, 0.70711, 0]), 'goal_position': np.array([-28.88652, -17.23535,   1.42725]), 'goal_orientation': np.array([0.70711, 0.     , 0.70711, 0.     ])}]
        self.util.move_ur10(motion_plan, "_main_cover")

        # remove world main cover and add ee main cover
        if self.util.motion_task_counter==2 and not self.bool_done[24]:
            self.bool_done[24] = True
            self.util.remove_part("World/Environment", f"main_cover_{self.id}")
            if self.id%2==0:
                self.util.add_part_custom("World/UR10_main_cover/ee_link","main_cover", f"qmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5, 0.5, -0.5, 0.5]))
            else:
                self.util.add_part_custom("World/UR10_main_cover/ee_link","main_cover_orange", f"qmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.71735, 0.26961, -0.69234]), np.array([0.5, 0.5, -0.5, 0.5]))

        # remove ee main cover and add mobile platform main cover
        if self.util.motion_task_counter==13 and not self.bool_done[25]:
            self.bool_done[25] = True
            self.util.remove_part("World/UR10_main_cover/ee_link", f"qmain_cover_{self.id}")
            if self.id%2==0:
                self.util.add_part_custom(f"mock_robot_{self.id}/platform","main_cover", f"xmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.81508, 0.27909, 0.19789]), np.array([0.70711, 0.70711, 0, 0]))
            else:
                self.util.add_part_custom(f"mock_robot_{self.id}/platform","main_cover_orange", f"xmain_cover_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.81508, 0.27909, 0.19789]), np.array([0.70711, 0.70711, 0, 0]))

        if self.util.motion_task_counter==17:
            print("Done placing main cover")
            self.util.motion_task_counter=0
            return True
        return False

    def move_to_handle_cell(self):
        print(self.util.path_plan_counter)
        # path_plan = [
        #              ["translate", [-13.2, 1, True]],
        #              ["rotate", [np.array([-0.56641, 0, 0, 0.82413]), 0.0042, True]],
        #              ["translate", [-10.46, 1, True]],
        #              ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
        #              ["translate", [-6.69, 1, True]]]
        
        path_plan = [
                     ["translate", [-13.2, 1, True]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.56641, 0, 0, 0.82413]), 0.0042, True]],
                     ["wait",[]],
                    #  ["translate", [-10.46, 1, True]],
                    #  ["translate", [-10.3, 1, True]],
                     ["translate", [-10.43, 1, True]],
                     ["wait",[]],
                     ["rotate", [np.array([-0.70711, 0, 0, 0.70711]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-6.69, 1, True]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_handle(self):
        motion_plan = [{"index":0, "position": np.array([0.83713, -0.36166, -0.16+0.31902+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33969, -6.83473, 0.56077+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":1, "position": np.array([0.83713, -0.36166, -0.16+0.31902]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33969, -6.83473, 0.56077]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":2, "position": np.array([0.83713, -0.36166, -0.16+0.31902+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-29.33969, -6.83473, 0.56077+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":3, "position": np.array([-0.00141, 0.74106, -0.16+0.61331]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.5011, -7.93748, 0.85506]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":4, "position": np.array([-1.01604+0.07, -0.16743-0.09, -0.13+0.76104]), "orientation": np.array([0.34531, -0.61706, 0.61706, 0.34531]), "goal_position":np.array([-27.48648-0.07, -7.02902, 1.00275]), "goal_orientation":np.array([-0.34531, -0.61706, -0.61706, 0.34531])},
                       {"index":5, "position": np.array([-1.01604+0.07, -0.26045-0.09, -0.13+0.7032]), "orientation": np.array([0.34531, -0.61706, 0.61706, 0.34531]), "goal_position":np.array([-27.48648-0.07, -6.93599, 0.94492]), "goal_orientation":np.array([-0.34531, -0.61706, -0.61706, 0.34531])},
                       {"index":6, "position": np.array([-1.01604+0.07, -0.16743-0.09, -0.13+0.76104]), "orientation": np.array([0.34531, -0.61706, 0.61706, 0.34531]), "goal_position":np.array([-27.48648-0.07, -7.02902, 1.00275]), "goal_orientation":np.array([-0.34531, -0.61706, -0.61706, 0.34531])},

                        {"index":7, "position": np.array([-0.00141, 0.74106, -0.16+0.61331]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-28.5011, -7.93748, 0.85506]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        self.util.move_ur10(motion_plan, "_handle")

        if self.util.motion_task_counter==2 and not self.bool_done[26]:
            self.bool_done[26] = True
            self.util.remove_part("World/Environment", f"handle_{self.id}")
            self.util.add_part_custom("World/UR10_handle/ee_link","handle", f"qhandle_{self.id}", np.array([0.001,0.001,0.001]), np.array([-0.5218, 0.42317, 0.36311]), np.array([0.5, -0.5, 0.5, -0.5]))
        
        if self.util.motion_task_counter==6 and not self.bool_done[27]:
            self.bool_done[27] = True
            print("Done placing handle")
            self.util.remove_part("World/UR10_handle/ee_link", f"qhandle_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","handle", f"xhandle_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.82439, 0.44736, 1.16068]), np.array([0.20721, 0.68156, -0.67309, -0.19874]))
            
        if self.util.motion_task_counter==8:
            self.util.motion_task_counter=0
            return True
        return False

    def screw_handle(self):
        motion_plan = [{"index":0, "position": np.array([-0.78213, -0.03592, -0.16+0.4263+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91341, -6.95701, 0.66945+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":1, "position": np.array([-0.78213, -0.03592, -0.16+0.4263]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91341, -6.95701, 0.66945]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":2, "position": np.array([-0.78213, -0.03592, -0.16+0.4263+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91341, -6.95701, 0.66945+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":3, "position": np.array([0.7448+0.13899, -0.02487, -0.16+0.7457+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.4413-0.13899, -6.96836, 0.98886+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":4, "position": np.array([0.7448+0.13899, -0.02487, -0.16+0.7457]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.4413-0.13899, -6.96836, 0.98886]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":5, "position": np.array([0.7448+0.13899, -0.02487, -0.16+0.7457+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.4413-0.13899, -6.96836, 0.98886+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":6, "position": np.array([-0.77943, -0.21316, -0.16+0.4263+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91611, -6.77978, 0.66945+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":7, "position": np.array([-0.77943, -0.21316, -0.16+0.4263]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91611, -6.77978, 0.66945]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":8, "position": np.array([-0.77943, -0.21316, -0.16+0.4263+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91611, -6.77978, 0.66945+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":9, "position": np.array([0.83599-0.024, -0.02487, -0.16+0.7457+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.53249+0.024, -6.96836, 0.98886+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":10, "position": np.array([0.83599-0.024, -0.02487, -0.16+0.7457]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.53249+0.024, -6.96836, 0.98886]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       {"index":11, "position": np.array([0.83599-0.024, -0.02487, -0.16+0.7457+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-27.53249+0.024, -6.96836, 0.98886+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},

                       {"index":12, "position": np.array([-0.77943, -0.21316, -0.16+0.4263+0.2]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-25.91611, -6.77978, 0.66945+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        self.util.do_screw_driving(motion_plan,"_handle")
        if self.util.motion_task_counter==13:
            print("Done screwing handle")
            self.util.motion_task_counter=0
            return True
        return False

    def move_to_light_cell(self):
        print(self.util.path_plan_counter)
        # path_plan = [["translate", [-6.41, 1, True]],
        #              ["rotate", [np.array([-1, 0, 0, 0]), 0.0042, False]],
        #              ["translate", [-18.56, 0, False]]]
        path_plan = [["translate", [-5.8, 1, True]],
                     ["wait",[]],
                    #  ["rotate", [np.array([-1, 0, 0, 0]), 0.008, False]],
                    ["rotate", [np.array([-1, 0, 0, 0]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-18.56, 0, False]]]
        if self.id==1 or self.id==2:
            path_plan = [["translate", [-5.77, 1, True]],
                     ["wait",[]],
                    #  ["rotate", [np.array([-1, 0, 0, 0]), 0.008, False]],
                    ["rotate", [np.array([-1, 0, 0, 0]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-18.56, 0, False]]]
        if self.id==6:
            path_plan = [["translate", [-5.85, 1, True]],
                     ["wait",[]],
                    #  ["rotate", [np.array([-1, 0, 0, 0]), 0.008, False]],
                    ["rotate", [np.array([-1, 0, 0, 0]), 0.0042, False]],
                     ["wait",[]],
                     ["translate", [-18.56, 0, False]]]

        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def arm_place_light(self):
        if not self.bool_done[30]:
            self.bool_done[30] = True
            motion_plan = [{"index":0, "position": np.array([0.03769, -0.74077, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
            self.util.do_screw_driving(motion_plan, "_light")
            self.util.motion_task_counter=0
        light_offset=0.1098
        motion_plan = [
                    #    {"index":0, "position": np.array([0.5517, 0.68287, -0.16+0.34371+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-18.38395, -7.23584, 0.58583+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                    #    {"index":1, "position": np.array([0.5517, 0.68287, -0.16+0.34371]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-18.38395, -7.23584, 0.58583]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                    #    {"index":2, "position": np.array([0.5517, 0.68287, -0.16+0.34371+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-18.38395, -7.23584, 0.58583+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])},
                       *self.light,

                       {"index":3, "position": np.array([-0.57726, -0.00505, -0.16+0.65911]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.25466, -6.54783, 0.90123]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                    #    {"index":4, "position": np.array([0.32558+0.12, -0.65596+0.04105-light_offset, -0.16+0.65947]), "orientation": np.array([0.65861, -0.65861, 0.25736, 0.25736]), "goal_position":np.array([-18.15724, -5.8969-0.04105, 0.90133]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":5, "position": np.array([0.36852+0.12, -0.65596+0.04105-light_offset, -0.16+0.61986]), "orientation": np.array([0.65861, -0.65861, 0.25736, 0.25736]), "goal_position":np.array([-18.2, -5.8969-0.04105, 0.86172]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                    #    {"index":6, "position": np.array([0.32558+0.12, -0.65596+0.04105-light_offset, -0.16+0.65947]), "orientation": np.array([0.65861, -0.65861, 0.25736, 0.25736]), "goal_position":np.array([-18.15724, -5.8969-0.04105, 0.90133]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":4, "position": np.array([0.29868+0.12, -0.76576+0, -0.16+0.68019]), "orientation": np.array([0.659, -0.65796, 0.25749, 0.25789]), "goal_position":np.array([-18.13043, -5.78712-0, 0.922310]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":5, "position": np.array([0.3648+0.12, -0.76576+0, -0.16+0.61948]), "orientation": np.array([0.659, -0.65796, 0.25749, 0.25789]), "goal_position":np.array([-18.19655, -5.78712-0, 0.8616]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":6, "position": np.array([0.29868+0.12, -0.76576+0, -0.16+0.68019]), "orientation": np.array([0.659, -0.65796, 0.25749, 0.25789]), "goal_position":np.array([-18.13043, -5.78712-0, 0.922310]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                        # {"index":7, "position": np.array([0.11405, 0.9514, -0.16+0.34371+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-17.94611, -7.50448, 0.58583+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])}
                        # {"index":7, "position": np.array([-0.57726, -0.00505, -0.16+0.65911]), "orientation": np.array([0.5, -0.5, 0.5, 0.5]), "goal_position":np.array([-17.25466, -6.54783, 0.90123]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":7, "position": np.array([0.5517, 0.68287, -0.16+0.34371+0.2]), "orientation": np.array([0.5, 0.5, 0.5, -0.5]), "goal_position":np.array([-18.38395, -7.23584, 0.58583+0.2]), "goal_orientation":np.array([0.5, -0.5, 0.5, 0.5])}
                       ]
        self.util.move_ur10(motion_plan, "_light")

        if self.util.motion_task_counter==2 and not self.bool_done[28]:
            self.bool_done[28] = True
            self.util.remove_part("World/Environment", f"light_0{self.id}")
            self.util.add_part_custom("World/UR10_light/ee_link","FFrontLightAssembly", f"qlight_{self.id}", np.array([0.001,0.001,0.001]), np.array([1.30826, -0.30485, -0.12023]), np.array([0.36036, -0.00194, 0.00463, 0.9328]))
        
        if self.util.motion_task_counter==6 and not self.bool_done[29]:
            self.bool_done[29] = True
            print("Done placing light")
            self.util.remove_part("World/UR10_light/ee_link", f"qlight_{self.id}")
            self.util.add_part_custom(f"mock_robot_{self.id}/platform","FFrontLightAssembly", f"xlight_{self.id}", np.array([0.001,0.001,0.001]), np.array([0.8669, -0.10851, 1.76492]), np.array([0.47905, -0.49076, 0.50734, 0.52179]))

        if self.util.motion_task_counter==8:
            self.util.motion_task_counter=0
            return True
        return False

    def screw_light(self):
        light_offset=0.12284
        # motion_plan = [{"index":0, "position": np.array([0.03769, -0.74077, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                {"index":1, "position": np.array([0.03769, -0.74077, -0.16+0.43386]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                {"index":2, "position": np.array([0.03769, -0.74077, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       
        #                {"index":3, "position": np.array([0.10078, 0.73338+0.04105-light_offset, -0.16+0.5969+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.85288-0.04105, 0.83931+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
        #                {"index":4, "position": np.array([0.10078, 0.73338+0.04105-light_offset, -0.16+0.5969]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.85288-0.04105, 0.83931]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
        #                {"index":5, "position": np.array([0.10078, 0.73338+0.04105-light_offset, -0.16+0.5969+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.85288-0.04105, 0.83931+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

        #                {"index":6, "position": np.array([0.21628, -0.7386, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                {"index":7, "position": np.array([0.21628, -0.7386, -0.16+0.43386]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
        #                {"index":8, "position": np.array([0.21628, -0.7386, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

        #                {"index":9, "position": np.array([0.10078, 0.82997+0.04105-light_offset, -0.16+0.5969+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.94947-0.04105, 0.83931+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
        #                {"index":10, "position": np.array([0.10078, 0.82997+0.04105-light_offset, -0.16+0.5969]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.94947-0.04105, 0.83931]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
        #                {"index":11, "position": np.array([0.10078, 0.82997+0.04105-light_offset, -0.16+0.5969+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.13269, -5.94947-0.04105, 0.83931+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

        #                {"index":12, "position": np.array([0.03932, -0.71305, -0.16+0.42576+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.0712, -4.4, 0.66799+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        

        motion_plan = [{"index":0, "position": np.array([0.03769, -0.74077, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":1, "position": np.array([0.03769, -0.74077, -0.16+0.43386]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":2, "position": np.array([0.03769, -0.74077, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.06964, -4.37873, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       
                       {"index":3, "position": np.array([0.07394, 0.72035+0, -0.16+0.57846+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.84019-0, 0.82088+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                       {"index":4, "position": np.array([0.07394, 0.72035+0, -0.16+0.57846]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.84019-0, 0.82088]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":5, "position": np.array([0.07394, 0.72035+0, -0.16+0.57846+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.84019-0, 0.82088+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":6, "position": np.array([0.21628, -0.7386, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":7, "position": np.array([0.21628, -0.7386, -0.16+0.43386]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},
                       {"index":8, "position": np.array([0.21628, -0.7386, -0.16+0.43386+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.24824, -4.38091, 0.67627+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])},

                       {"index":9, "position": np.array([0.07394, 0.61525+0, -0.16+0.57846+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.73509-0, 0.82088+0.2]), "goal_orientation":np.array([0, 0.70711, 0, -0.70711])},
                       {"index":10, "position": np.array([0.07394, 0.61525+0, -0.16+0.57846]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.73509-0, 0.82088]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},
                       {"index":11, "position": np.array([0.07394, 0.61525+0, -0.16+0.57846+0.2]), "orientation": np.array([0.70711, 0, 0.70711, 0]), "goal_position":np.array([-18.10573, -5.73509-0, 0.82088+0.2]), "goal_orientation":np.array([0, -0.70711, 0, 0.70711])},

                       {"index":12, "position": np.array([0.03932, -0.71305, -0.16+0.42576+0.2]), "orientation": np.array([0, 0.70711, 0, -0.70711]), "goal_position":np.array([-18.0712, -4.4, 0.66799+0.2]), "goal_orientation":np.array([0.5, 0.5, 0.5, -0.5])}]
        
        self.util.do_screw_driving(motion_plan,"_light")
        if self.util.motion_task_counter==13:
            print("Done screwing light")
            self.util.motion_task_counter=0
            return True
        return False
    

    def go_to_end_goal(self):
        print(self.util.path_plan_counter)
        path_plan = [["translate", [-9.95, 0, True]],
                     ["wait",[]],
                    ["rotate", [np.array([0.99341, 0, 0, -0.11458]), 0.0042, True]],
                    ["wait",[]],
                    ["translate", [-9.16, 0, True]],
                    ["wait",[]],
                    ["rotate", [np.array([1,0,0,0]), 0.0042, False]],
                    ["wait",[]],
                    ["translate", [8, 0, True]],
                     ]
        if self.id==2 or self.id==4:
            path_plan = [["translate", [-9.95, 0, True]],
                     ["wait",[]],
                    ["rotate", [np.array([0.99341, 0, 0, -0.11458]), 0.0042, True]],
                    ["wait",[]],
                    ["translate", [-9.16, 0, True]],
                    ["wait",[]],
                    ["rotate", [np.array([1,0,0,0]), 0.0042, False]],
                    ["wait",[]],
                    ["wait",[]],
                    ["wait",[]],
                    ["wait",[]],
                    ["wait",[]],
                    ["wait",[]],
                    ["translate", [8, 0, True]],
                     ]

        self.util.move_mp(path_plan)
        # current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            self.moving_platform.apply_action(self.util._my_custom_controller.forward(command=[0.5,0]))

            self.util.remove_part(f"mock_robot_{self.id}","platform")
            self.util.remove_part(f"mock_robot_{self.id}","wheel_top_right")
            self.util.remove_part(f"mock_robot_{self.id}","wheel_top_left")
            self.util.remove_part(f"mock_robot_{self.id}","wheel_bottom_right")
            self.util.remove_part(f"mock_robot_{self.id}","wheel_bottom_left")


            # self.util.remove_part(None,f"mock_robot_{self.id}")
            # self.util.remove_part(None,f"mock_robot_{self.id}")
            # self.util.remove_part(None,f"mock_robot_{self.id}")
            # self.util.remove_part(None,f"mock_robot_{self.id}")
            # self.util.remove_part(None,f"mock_robot_{self.id}")
            # self.util.remove_part(None,f"mock_robot_{self.id}")
            return True
        return False
    
    def move_to_contingency_cell(self):
        print(self.util.path_plan_counter)
        path_plan = [
                     ["translate", [-23.937, 1, True]],
                     ["wait",[]],
                     ["rotate", [np.array([0,0,0,1]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-35.63314, 0, True]],
                     ["wait",[]],
                     ["rotate", [np.array([0.70711, 0, 0, 0.70711]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [17.6, 1, True]],
                     ["wait",[]],
                     ["rotate", [np.array([1,0,0,0]), 0.0042, True]],
                     ["wait",[]],
                     ["translate", [-20.876, 0, True]]]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False

    def disassemble(self):
        print(self.disassembly_event)

        def delay_for(amount):
            if self.delay == amount:
                self.delay=0
                self.disassembly_event+=1
                return True
            else:
                self.delay+=1
            return False

        # remove and add part
        if self.disassembly_event == 0:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xmain_cover_{self.id}")
                self.util.add_part_custom("World/Environment","main_cover", "qmain_cover", np.array([0.001, 0.001, 0.001]), np.array([-24.35262, 19.39603, -0.02519]), np.array([0.70711, 0.70711, 0, 0]))
                self.isDone[self.disassembly_event-1]=True

        # remove and add part
        elif self.disassembly_event == 1:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xlower_coverr_{self.id}")
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xlower_coverl_{self.id}")
                self.util.add_part_custom("World/Environment","lower_cover", "qlower_coverr", np.array([0.001, 0.001, 0.001]), np.array([-20.65511, 19.03402, 0.25662]), np.array([0,0,0.70711,0.70711]))
                self.util.add_part_custom("World/Environment","lower_cover", "qlower_coverl", np.array([0.001, 0.001, 0.001]), np.array([-20.29928, 19.04322, 0.25662]), np.array([0,0,-0.70711,-0.70711]))
                self.isDone[self.disassembly_event-1]=True

        # remove and add part
        elif self.disassembly_event == 2:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xwheel_01_{self.id}")
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xwheel_02_{self.id}")
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xwheel_03_{self.id}")
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xwheel_04_{self.id}")
                self.util.add_part_custom("World/Environment","FWheel", "qwheel_01", np.array([0.001, 0.001, 0.001]), np.array([-18.86967, 18.93091, 0.25559]), np.array([0.70711, 0, -0.70711, 0]))
                self.util.add_part_custom("World/Environment","FWheel", "qwheel_02", np.array([0.001, 0.001, 0.001]), np.array([-18.34981, 18.93091, 0.25559]), np.array([0.70711, 0, -0.70711, 0]))
                self.util.add_part_custom("World/Environment","FWheel", "qwheel_03", np.array([0.001, 0.001, 0.001]), np.array([-17.85765, 18.93091, 0.25559]), np.array([0.70711, 0, -0.70711, 0]))
                self.util.add_part_custom("World/Environment","FWheel", "qwheel_04", np.array([0.001, 0.001, 0.001]), np.array([-17.33776, 18.93091, 0.25559]), np.array([0.70711, 0, -0.70711, 0]))
                self.isDone[self.disassembly_event-1]=True

        # remove and add part
        elif self.disassembly_event == 3:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xfuel_{self.id}")
                self.util.add_part_custom("World/Environment","fuel", "qfuel", np.array([0.001, 0.001, 0.001]), np.array([-23.63045, 16.54196, 0.26478]), np.array([0.5,0.5,-0.5,-0.5]))
                self.isDone[self.disassembly_event-1]=True
        
        # remove and add part
        elif self.disassembly_event == 4:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xbattery_{self.id}")
                self.util.add_part_custom("World/Environment","battery", "qbattery", np.array([0.001, 0.001, 0.001]), np.array([-21.87392, 16.38365, 0.25864]), np.array([0.70711, 0.70711, 0, 0]))
                self.isDone[self.disassembly_event-1]=True

        # remove and add part
        elif self.disassembly_event == 5:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"xFSuspensionBack_{self.id}")
                self.util.add_part_custom("World/Environment","FSuspensionBack", "qsuspension_back", np.array([0.001, 0.001, 0.001]), np.array([-20.82231, 16.78007, 0.24643]), np.array([0.5, 0.5, -0.5, 0.5]))
                self.isDone[self.disassembly_event-1]=True

        elif self.disassembly_event == 6:
            if not self.isDone[self.disassembly_event] and delay_for(100):
                self.util.remove_part(f"mock_robot_{self.id}/platform",f"engine_{self.id}")
                self.util.add_part_custom("World/Environment","engine_no_rigid", "qengine", np.array([0.001, 0.001, 0.001]), np.array([-19.07782, 16.35288, 0.43253]), np.array([0.99457,0,-0.10407,0]))
                self.isDone[self.disassembly_event-1]=True
        
        elif self.disassembly_event == 7:
            return True
        
        return False


    def carry_on(self):
        print(self.util.path_plan_counter)
        path_plan = [
                     ["translate", [-10.0529, 0, True]]
                     ]
        self.util.move_mp(path_plan)
        if len(path_plan) == self.util.path_plan_counter:
            self.util.path_plan_counter=0
            return True
        return False