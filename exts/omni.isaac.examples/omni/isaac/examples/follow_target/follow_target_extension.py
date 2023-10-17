import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.follow_target import FollowTarget

import omni.ext
import omni.ui as ui
from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder
import asyncio

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import pi


class FollowTargetExtension(BaseSampleExtension):
    # same as in mp_2dnav package base_local_planner_params.yaml
    _xy_goal_tolerance = 0.25
    _yaw_goal_tolerance = 0.05
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="Navigation",
            title="My Awesome Example",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html",
            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            file_path=os.path.abspath(__file__),
            sample=FollowTarget(),
        )
        return
    
    def _build_ui(
        self, name, title, doc_link, overview, file_path, number_of_extra_frames, window_width, keep_window_open
    ):
        self._window = omni.ui.Window(
            name, width=window_width, height=0, visible=keep_window_open, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                setup_ui_headers(self._ext_id, file_path, title, doc_link, overview)
                self._controls_frame = ui.CollapsableFrame(
                    title="World Controls",
                    width=ui.Fraction(1),
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    for i in range(number_of_extra_frames):
                        self._extra_frames.append(
                            ui.CollapsableFrame(
                                title="",
                                width=ui.Fraction(0.33),
                                height=0,
                                visible=False,
                                collapsed=False,
                                style=get_style(),
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            )
                        )
                with self._controls_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        dict = {
                            "label": "Load World",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Load World and Task",
                            "on_clicked_fn": self._on_load_world,
                        }
                        self._buttons["Load World"] = btn_builder(**dict)
                        self._buttons["Load World"].enabled = True
                        dict = {
                            "label": "Reset",
                            "type": "button",
                            "text": "Reset",
                            "tooltip": "Reset robot and environment",
                            "on_clicked_fn": self._on_reset,
                        }
                        self._buttons["Reset"] = btn_builder(**dict)
                        self._buttons["Reset"].enabled = False
                        dict = {
                            "label": "Send Goal",
                            "type": "button",
                            "text": "Goal",
                            "tooltip": "",
                            "on_clicked_fn": self._send_navigation_goal,
                        }
                        self._buttons["Send Goal"] = btn_builder(**dict)
                        self._buttons["Send Goal"].enabled = False
        return
    
    def post_load_button_event(self):
        self._buttons["Send Goal"].enabled = True

    def _check_goal_reached(self, goal_pose):
        # Cannot get result from ROS because /move_base/result also uses move_base_msgs module
        mp_position, mp_orientation = self._sample._mp.get_world_pose()
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
        x,y,a = -1,7,3.14
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

        self._sample._goal_pub.publish(goal_msg)

        world = self.get_world()
        if not world.physics_callback_exists("mp_nav_check"):
            world.add_physics_callback("mp_nav_check", lambda step_size: self._check_goal_reached(pose))
        # Overwrite check with new goal
        else:
            world.remove_physics_callback("mp_nav_check")
            world.add_physics_callback("mp_nav_check", lambda step_size: self._check_goal_reached(pose))
