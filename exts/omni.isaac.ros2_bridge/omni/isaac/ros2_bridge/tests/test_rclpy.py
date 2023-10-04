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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRclpy(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        gc.collect()
        pass

    async def test_rclpy(self):
        import rclpy

        rclpy.init()

        from std_msgs.msg import String

        msg = String()
        node = rclpy.create_node("minimal_publisher")
        publisher = node.create_publisher(String, "topic", 10)
        publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()
        pass
