# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid

# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

# Note that this is not the system level rospy, but one compiled for omniverse
import numpy as np
import rospy
from std_msgs.msg import Empty
import time


class Subscriber:
    def __init__(self):
        # setting up the world with a cube
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()
        # add a cube in the world
        cube_path = "/cube"
        self.ros_world.scene.add(
            VisualCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 10]), size=0.2)
        )
        self._cube_position = np.array([0, 0, 0])

        # setup the ros subscriber here
        self.ros_sub = rospy.Subscriber("/move_cube", Empty, self.move_cube_callback, queue_size=10)

        self.ros_world.reset()

    def move_cube_callback(self, data):
        # callback function to set the cube position to a new one upon receiving a (empty) ros message
        if self.ros_world.is_playing():
            self._cube_position = np.array([np.random.rand() * 0.40, np.random.rand() * 0.40, 0.10])

    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 0:
                    self.ros_world.reset()

                # the actual setting the cube pose is done here
                self.ros_world.scene.get_object("cube_1").set_world_pose(self._cube_position)

        # Cleanup
        self.ros_sub.unregister()
        rospy.signal_shutdown("subscriber example complete")
        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    rospy.init_node("tutorial_subscriber", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    subscriber = Subscriber()
    subscriber.run_simulation()
