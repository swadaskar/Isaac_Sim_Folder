# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import gc
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from .common import wait_for_rosmaster
from omni.isaac.core.utils.physics import simulate_async
import carb
import omni.graph.core as og

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRosClock(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        from omni.isaac.ros_bridge.scripts.roscore import Roscore
        import rospy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = omni.usd.get_context().get_stage()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)
        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()

        self._roscore = Roscore()
        await wait_for_rosmaster()
        # You must disable signals so that the init node call does not take over the ctrl-c callback for kit
        try:
            rospy.init_node("isaac_sim_test_clock", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")
        pass

    # After running each test
    async def tearDown(self):
        self._stage = None
        self._timeline = None
        # rospy.signal_shutdown("test_complete")
        self._roscore.shutdown()
        self._roscore = None

        gc.collect()
        pass

    async def test_sim_clock(self):
        import rospy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.to_sec()

        clock_sub = rospy.Subscriber("/clock", Clock, clock_callback)

        self._timeline.play()

        await simulate_async(2.1)
        self._timeline.stop()
        clock_sub.unregister()
        self.assertGreater(self._time_sec, 2.0)

        pass

    async def test_sim_clock_manual(self):
        import rospy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("Impulse", "omni.graph.action.OnImpulseEvent"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.SET_VALUES: [("IsaacClock.inputs:resetOnStop", True)],
                keys.CONNECT: [
                    ("Impulse.outputs:execOut", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.to_sec()

        clock_sub = rospy.Subscriber("/clock", Clock, clock_callback)
        await asyncio.sleep(1.0)
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(self._time_sec, 0.0)
        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # after first step we need to wait for ros node to initialize
        await asyncio.sleep(1.0)

        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # wait for message
        await asyncio.sleep(1.0)
        self.assertGreater(self._time_sec, 0.0)

        self._timeline.stop()
        clock_sub.unregister()

        pass

    async def test_system_clock(self):
        import rospy
        from rosgraph_msgs.msg import Clock
        import time

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                    ("RosPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:systemTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )
        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.to_sec()

        clock_sub = rospy.Subscriber("/clock", Clock, clock_callback)

        self._timeline.play()

        await simulate_async(1.0)
        self.assertAlmostEqual(self._time_sec, time.time(), delta=0.5)
        self._timeline.stop()
        clock_sub.unregister()
        pass
