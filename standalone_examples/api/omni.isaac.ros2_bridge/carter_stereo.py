# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.kit import SimulationApp
import argparse

parser = argparse.ArgumentParser(description="Generate Occluded and Unoccluded data")
parser.add_argument("--test", action="store_true")
parser.add_argument(
    "--ros2_bridge",
    default="omni.isaac.ros2_bridge",
    nargs="?",
    choices=["omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble"],
)
args, unknown = parser.parse_known_args()

# Example ROS2 bridge sample showing manual control over messages
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import SimulationContext
from pxr import Sdf

from omni.isaac.core.utils.extensions import enable_extension

import omni.graph.core as og

# enable ROS2 bridge extension
enable_extension(args.ros2_bridge)

# Locate assets root folder to load sample
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()

usd_path = assets_root_path + "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
omni.usd.get_context().open_stage(usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

ros_cameras_graph_path = "/World/Carter_ROS/ROS_Cameras"

# Enabling rgb and depth image publishers for left camera. Cameras will automatically publish images each frame
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_left.inputs:condition"), True)
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_left_rgb.inputs:condition"), True)
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_left_depth.inputs:condition"), True)

simulation_context.play()
simulation_context.step()

# Enabling rgb and depth image publishers for right camera after left cameras are initialized. Cameras will automatically publish images each frame
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_right.inputs:condition"), True)
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_right_rgb.inputs:condition"), True)
og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_right_depth.inputs:condition"), True)

# Simulate for one second to warm up sim and let everything settle
for frame in range(60):
    simulation_context.step()

# Dock the second camera window
left_viewport = omni.ui.Workspace.get_window("Viewport")
right_viewport = omni.ui.Workspace.get_window("Viewport 2")
if right_viewport is not None and left_viewport is not None:
    right_viewport.dock_in(left_viewport, omni.ui.DockPosition.RIGHT)
right_viewport = None
left_viewport = None

# Create a rostopic to publish message to spin robot in place
# Note that this is not the system level rclpy, but one compiled for omniverse

import rclpy

rclpy.init()

from geometry_msgs.msg import Twist

node = rclpy.create_node("carter_stereo")
publisher = node.create_publisher(Twist, "cmd_vel", 10)

frame = 0
while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)

    # Publish the ROS Twist message every 2 frames
    if frame % 2 == 0:
        message = Twist()
        message.angular.z = 0.5  # spin in place
        publisher.publish(message)

    if args.test and frame > 120:
        break
    frame = frame + 1
node.destroy_node()
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
