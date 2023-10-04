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
import carb
import asyncio
import copy

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

from .common import add_cube, wait_for_rosmaster, add_carter_ros
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Sdf
from omni.isaac.core.utils.physics import simulate_async

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRosLidar(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        from omni.isaac.ros_bridge.scripts.roscore import Roscore
        import rospy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()

        self._roscore = Roscore()
        await wait_for_rosmaster()
        await omni.kit.app.get_app().next_update_async()

        try:
            rospy.init_node("isaac_sim_test_rospy", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")

        pass

    # After running each test
    async def tearDown(self):
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        # rospy.signal_shutdown("test_complete")
        self._roscore.shutdown()
        self._roscore = None
        self._timeline = None
        gc.collect()
        pass

    async def test_lidar(self):
        import rospy

        from sensor_msgs.msg import LaserScan

        await add_carter_ros()
        await add_cube("/cube", 0.75, (2.00, 0, 0.75))

        self._lidar_data = None
        self._lidar_data_prev = None

        def lidar_callback(data: LaserScan):
            self._lidar_data = data

        lidar_sub = rospy.Subscriber("scan", LaserScan, lidar_callback)

        def standard_checks():
            self.assertIsNotNone(self._lidar_data)
            self.assertGreater(self._lidar_data.angle_max, self._lidar_data.angle_min)
            self.assertEqual(self._lidar_data.intensities[0], 0.0)
            self.assertEqual(len(self._lidar_data.intensities), 900)
            self.assertEqual(self._lidar_data.intensities[450], 255.0)

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"), value=0.0, prev=None
        )

        # 0.0 Hz Lidar rotation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0)

        standard_checks()
        self.assertEqual(self._lidar_data.time_increment, 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
            value=121.0,
            prev=None,
        )

        await omni.kit.app.get_app().next_update_async()
        # 121.0 Hz Lidar rotation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)
        self.assertGreater(self._lidar_data.time_increment, 0.0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
            value=201.0,
            prev=None,
        )

        # 201.0 Hz Lidar rotation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)

        self.assertGreater(self._lidar_data_prev.time_increment, self._lidar_data.time_increment)

        self._timeline.stop()

        lidar_sub.unregister()
        pass
