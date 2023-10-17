import carb
import asyncio
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

import rospy
import rosgraph
# import actionlib
# # Isaac sim cannot find these two modules for some reason...
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from obstacle_map import GridMap
# # Using /move_base_simple/goal for now
from geometry_msgs.msg import PoseStamped
import numpy as np

class FollowTarget(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        # FIXME
        add_reference_to_stage(usd_path='/home/lm-2023/Isaac_Sim/navigation/Collected_real_microfactory_show/real_microfactory_show.usd', prim_path="/World")
        world.scene.add(Robot(prim_path="/World/damobile_platform", name="damobile_platform"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._mp = self._world.scene.get_object("damobile_platform")
        
        if not rosgraph.is_master_online():
            print("Please run roscore before executing this script")
            return
        
        try:
            rospy.init_node("set_goal_py")
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")
        
        # FIXME
        # self._initial_goal_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        # self.__send_initial_pose()
        # await asyncio.sleep(1.0)
        # self._action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        return
    
    # def __send_initial_pose(self):
    #     mp_position, mp_orientation = self._mp.get_world_pose()

    #     current_pose = PoseWithCovarianceStamped()
    #     current_pose.header.frame_id = "map"
    #     current_pose.header.stamp = rospy.get_rostime()
    #     current_pose.pose.pose.position.x = mp_position[0]
    #     current_pose.pose.pose.position.y = mp_position[1]
    #     current_pose.pose.pose.position.z = mp_position[2]
    #     current_pose.pose.pose.orientation.x = mp_orientation[0]
    #     current_pose.pose.pose.orientation.y = mp_orientation[1]
    #     current_pose.pose.pose.orientation.z = mp_orientation[2]
    #     current_pose.pose.pose.orientation.w = mp_orientation[3]
    #     # rospy.sleep will stall Isaac sim simulation
    #     # rospy.sleep(1)
    #     self._initial_goal_publisher.publish(current_pose)

    # def __goal_response_callback(self, current_state, result):
    #     if current_state not in [0, 1, 3]:
    #         print("Goal rejected :(")
    #         return

    #     print("Goal accepted :)")
    #     wait = self._action_client.wait_for_result()
    #     self.__get_result_callback(wait)

    # def __get_result_callback(self, wait):
    #     print("Result: "+str(wait))
    
    # def send_navigation_goal(self, x, y, a):
    #     FIXME
    #     map_yaml_path = "/home/shihanzh/ros_ws/src/mp_2dnav/map/mp_microfactory_navigation.yaml"
    #     grid_map = GridMap(map_yaml_path)
        
    #     # generate random goal
    #     if x is None:
    #         print("No goal received. Generating random goal...")
    #         range_ = grid_map.get_range()
    #         x = np.random.uniform(range_[0][0], range_[0][1])
    #         y = np.random.uniform(range_[1][0], range_[1][1])
    #         orient_x = 0  # not needed because robot is in x,y plane
    #         orient_y = 0  # not needed because robot is in x,y plane
    #         orient_z = np.random.uniform(0, 1)
    #         orient_w = np.random.uniform(0, 1)
    #     else:
    #         orient_x, orient_y, orient_z, orient_w = quaternion_from_euler(0, 0, a)
        
    #     pose = [x, y, orient_x, orient_y, orient_z, orient_w]
    #     if grid_map.is_valid_pose([x, y], 0.2):
    #         self._action_client.wait_for_server()

    #         goal_msg = MoveBaseGoal()
    #         goal_msg.target_pose.header.frame_id = "map"
    #         goal_msg.target_pose.header.stamp = rospy.get_rostime()
    #         print("goal pose: "+str(pose))
    #         goal_msg.target_pose.pose.position.x = pose[0]
    #         goal_msg.target_pose.pose.position.y = pose[1]
    #         goal_msg.target_pose.pose.orientation.x = pose[2]
    #         goal_msg.target_pose.pose.orientation.y = pose[3]
    #         goal_msg.target_pose.pose.orientation.z = pose[4]
    #         goal_msg.target_pose.pose.orientation.w = pose[5]

    #         self._action_client.send_goal(goal_msg, done_cb=self.__goal_response_callback)
    #     else:
    #         print("Invalid goal "+str(pose))
    #     return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
