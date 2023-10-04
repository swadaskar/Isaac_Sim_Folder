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
import omni.graph.core as og
import carb
import asyncio

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
from omni.isaac.core.utils.physics import simulate_async
from .common import get_qos_profile

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2BridgeCommands(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = omni.usd.get_context().get_stage()
        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        rclpy.init()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        import rclpy

        self._stage = None
        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    async def test_sim_clock(self):
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()

        await simulate_async(2.1, callback=spin)
        self._timeline.stop()
        self.assertGreater(self._time_sec, 2.0)
        spin()
        pass

    async def test_sim_clock_manual(self):
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("Impulse", "omni.graph.action.OnImpulseEvent"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
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
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        await simulate_async(1.0, callback=spin)
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._time_sec, 0.0)
        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # after first step we need to wait for ros node to initialize
        await simulate_async(1.0, callback=spin)

        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # wait for message
        await simulate_async(1.0, callback=spin)
        self.assertGreater(self._time_sec, 0.0)

        self._timeline.stop()
        spin()
        pass

    async def test_system_clock(self):
        import rclpy
        from rosgraph_msgs.msg import Clock
        import time

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSystemTime"),
                    ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:systemTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )
        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()

        await simulate_async(1.0, callback=spin)
        self.assertAlmostEqual(self._time_sec, time.time(), delta=0.5)
        self._timeline.stop()
        spin()
        pass
