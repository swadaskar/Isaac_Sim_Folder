# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import time
import carb
from omni.isaac.kit import SimulationApp


# Example ROS bridge sample showing rospy and rosclock interaction
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})
import omni
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core import SimulationContext

import omni.graph.core as og

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
from rosgraph_msgs.msg import Clock
import rospy

clock_topic = "sim_time"
system_clock_topic = "system_time"
manual_clock_topic = "manual_time"

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ReadSystemTime", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ("PublishSystemClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("PublishManualClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                # Connecting execution of OnPlaybackTick node to PublishClock and PublishSystemClock to automatically publish each frame
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishSystemClock.inputs:execIn"),
                # Connecting execution of OnImpulseEvent node to PublishManualClock so it will only publish when an impulse event is triggered
                ("OnImpulseEvent.outputs:execOut", "PublishManualClock.inputs:execIn"),
                # Connecting simulationTime data of ReadSimTime to the clock publisher nodes
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishManualClock.inputs:timeStamp"),
                # Connecting systemTime data of ReadSystemTime to the system clock publisher node
                ("ReadSystemTime.outputs:systemTime", "PublishSystemClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Assigning topic names to clock publishers
                ("PublishClock.inputs:topicName", clock_topic),
                ("PublishSystemClock.inputs:topicName", system_clock_topic),
                ("PublishManualClock.inputs:topicName", manual_clock_topic),
            ],
        },
    )
except Exception as e:
    print(e)


simulation_app.update()
simulation_app.update()


# Define ROS callbacks
def sim_clock_callback(data):
    print("sim time:", data.clock.to_sec())


def system_clock_callback(data):
    print("system time:", data.clock.to_sec())


def manual_clock_callback(data):
    print("manual stepped sim time:", data.clock.to_sec())


# Create rospy ndoe
rospy.init_node("isaac_sim_clock", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
# create subscribers
sim_clock_sub = rospy.Subscriber(clock_topic, Clock, sim_clock_callback)
system_clock_sub = rospy.Subscriber(system_clock_topic, Clock, system_clock_callback)
manual_clock_sub = rospy.Subscriber(manual_clock_topic, Clock, manual_clock_callback)
time.sleep(1.0)
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

# perform a fixed number of steps with fixed step size
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        simulation_context.render()  # This updates rendering/app loop which calls the system and sim clocks

    simulation_context.step(render=False)  # runs with a non-realtime clock
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# perform a fixed number of steps with realtime clock
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

    simulation_app.update()  # runs with a realtime clock
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# cleanup and shutdown
sim_clock_sub.unregister()
system_clock_sub.unregister()
manual_clock_sub.unregister()
simulation_context.stop()
simulation_app.close()
