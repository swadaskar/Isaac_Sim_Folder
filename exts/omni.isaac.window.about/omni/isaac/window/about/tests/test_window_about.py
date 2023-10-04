## Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
##
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.
##
import omni.kit.app
import omni.kit.test
import omni.ui as ui
from omni.ui.tests.test_base import OmniUiTest

from pxr import Kind, Sdf, Gf
import pathlib


class TestAboutWindow(OmniUiTest):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        # from omni.isaac.window.about.about import TEST_DATA_PATH

        # self._golden_img_dir = TEST_DATA_PATH.absolute().joinpath("golden_img").absolute()

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    async def test_about_ui(self):
        class FakePluginImpl:
            def __init__(self, name):
                self.name = name

        class FakePlugin:
            def __init__(self, name):
                self.libPath = "Lib Path " + name
                self.impl = FakePluginImpl("Impl " + name)
                self.interfaces = "Interface " + name

        about = omni.isaac.window.about.get_instance()
        about.kit_version = "#Version#"
        about.nucleus_version = "#Nucleus Version#"
        about.client_library_version = "#Client Library Version#"
        about.app_name = "#App Name#"
        about.app_version = "#App Version#"
        about_window = about.menu_show_about(
            [FakePlugin("Test 1"), FakePlugin("Test 2"), FakePlugin("Test 3"), FakePlugin("Test 4")]
        )

        about.get_values()

        await self.docked_test_window(window=about_window, width=400, height=510)

        # await self.finalize_test(golden_img_dir=self._golden_img_dir, golden_img_name="test_about_ui.png")
