# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni
from omni.isaac.sensor import _sensor
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import omni.kit.commands
from pxr import Gf

# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

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

try:
    from isaac_tutorials.msg import ContactSensor
except ModuleNotFoundError:
    carb.log_error("isaac_tutorials message definition was not found, please source the ros workspace")
    simulation_app.close()
    exit()

rospy.init_node("contact_sample", anonymous=True, disable_signals=True, log_level=rospy.ERROR)

timeline = omni.timeline.get_timeline_interface()
contact_pub = rospy.Publisher("/contact_report", ContactSensor, queue_size=0)
cs = _sensor.acquire_contact_sensor_interface()

meters_per_unit = 1.0
ros_world = World(stage_units_in_meters=1.0)

# add a cube in the world
cube_path = "/cube"
cube_1 = ros_world.scene.add(
    DynamicCuboid(prim_path=cube_path, name="cube_1", position=np.array([0, 0, 1.5]), size=1.0)
)

simulation_app.update()

# Add a plane for cube to collide with
ros_world.scene.add_default_ground_plane()

simulation_app.update()


# putting contact sensor in the ContactSensor Message format
def format_contact(c_out, contact):
    c_out.time = float(contact["time"])
    c_out.value = float(contact["value"] * meters_per_unit)
    c_out.in_contact = bool(contact["inContact"])
    return c_out


# Setup contact sensor on cube
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateContactSensor",
    path="/Contact_Sensor",
    parent=cube_path,
    min_threshold=0,
    max_threshold=100000000,
    color=Gf.Vec4f(1, 1, 1, 1),
    radius=-1,
    sensor_period=1.0 / 60.0,
    translation=Gf.Vec3d(0, 0, 0),
)
simulation_app.update()

# initiate the message handle
c_out = ContactSensor()

# start simulation
timeline.play()


for frame in range(10000):
    ros_world.step(render=False)

    # Get processed contact data
    reading = cs.get_sensor_readings(cube_path + "/Contact_Sensor")
    if reading.shape[0]:
        for r in reading:
            print(r)
            # pack the raw data into ContactSensor format and publish it
            c = format_contact(c_out, r)
            contact_pub.publish(c)


# Cleanup
timeline.stop()
contact_pub.unregister()
rospy.signal_shutdown("contact_sample complete")
simulation_app.close()
