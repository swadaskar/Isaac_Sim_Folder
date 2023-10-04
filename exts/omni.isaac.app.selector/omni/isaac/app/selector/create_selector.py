# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import asyncio

import carb
import carb.settings
import carb.tokens
import carb.windowing

import omni.ext

import omni.kit.app

from omni.isaac.version import get_version
from .selector_window import SelectorWindow
from .settings import (
    AUTO_START_SETTING,
    DEFAULT_APP_SETTING,
    SHOW_CONSOLE_SETTING,
    PERSISTENT_SELECTOR_SETTING,
    EXTRA_ARGS_SETTING,
    PERSISTENT_ROS_BRIDGE_SETTING,
    ROS_BRIDGE_EXTENSIONS,
)

from .start_app import start_app


class CreateSelectorExtension(omni.ext.IExt):
    """"""

    def __init__(self):
        self._settings = carb.settings.get_settings()
        self._selector_window = None
        self._app_version = None
        self._app_version, _, _, _, _, _, _, _ = get_version()

    def on_startup(self, ext_id: str):
        # Initialize settings
        default_app = self._settings.get(DEFAULT_APP_SETTING)
        user_auto_start = self._settings.get(AUTO_START_SETTING)
        persistent_selector = self._settings.get(PERSISTENT_SELECTOR_SETTING)
        user_show_console = self._settings.get(SHOW_CONSOLE_SETTING)
        user_extra_args = self._settings.get(EXTRA_ARGS_SETTING)
        user_ros_bridge_extension = self._settings.get(PERSISTENT_ROS_BRIDGE_SETTING)

        app_extra_args = []

        if default_app is None:
            default_app = self._settings.get("/ext/omni.isaac.selector/default_app")
            self._settings.set(DEFAULT_APP_SETTING, default_app)
        if default_app is None:
            self._settings.set(DEFAULT_APP_SETTING, "isaac-sim")

        if user_auto_start is None:
            user_auto_start = self._settings.get("/ext/omni.isaac.selector/auto_start")
            self._settings.set(AUTO_START_SETTING, user_auto_start)
        if user_auto_start is None:
            self._settings.set(AUTO_START_SETTING, False)

        if persistent_selector is None:
            persistent_selector = self._settings.get("/ext/omni.isaac.selector/persistent_selector")
            self._settings.set(PERSISTENT_SELECTOR_SETTING, persistent_selector)
        if persistent_selector is None:
            self._settings.set(PERSISTENT_SELECTOR_SETTING, False)

        if user_show_console is None:
            user_show_console = self._settings.get("/ext/omni.isaac.selector/show_console")
            self._settings.set(SHOW_CONSOLE_SETTING, user_show_console)
        if user_show_console is None:
            self._settings.set(SHOW_CONSOLE_SETTING, True)

        if user_extra_args is None:
            user_extra_args = self._settings.get("/ext/omni.isaac.selector/extra_args")
            self._settings.set(EXTRA_ARGS_SETTING, user_extra_args)
        if user_extra_args is None:
            self._settings.set(EXTRA_ARGS_SETTING, "")

        if user_ros_bridge_extension is None:
            user_ros_bridge_extension = self._settings.get("/ext/omni.isaac.selector/ros_bridge_extension")
        if user_ros_bridge_extension:
            ros_bridge_extension = self._settings.set(PERSISTENT_ROS_BRIDGE_SETTING, user_ros_bridge_extension)

        ros_bridge_extension = self._settings.get(PERSISTENT_ROS_BRIDGE_SETTING)
        if ros_bridge_extension is not None:
            app_extra_args.append(
                "--/isaac/startup/ros_bridge_extension=" + ROS_BRIDGE_EXTENSIONS[ros_bridge_extension]
            )

        # Auto-starting default app
        if user_auto_start:
            default_app = self._settings.get(DEFAULT_APP_SETTING)
            if not default_app:
                default_app = self._settings.get("/ext/omni.isaac.selector/default_app")

            all_additional_args = str.split(user_extra_args)
            all_additional_args.extend(app_extra_args)

            start_app(
                app_id=default_app,
                app_version=self._app_version,
                app_become_new_default=False,
                persistent_selector=persistent_selector,
                extra_args=all_additional_args,
            )
            if not persistent_selector:
                return

        # We only load the UI App if we have not auto-started
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if not ext_manager.is_extension_enabled("omni.kit.uiapp"):
            ext_manager.set_extension_enabled_immediate("omni.kit.uiapp", True)
            ext_manager.set_extension_enabled_immediate("omni.kit.window.title", True)

        from omni.kit.window.title import get_main_window_title

        extension_path = ext_manager.get_extension_path(ext_id)

        # Setup window title and version
        window_title = get_main_window_title()
        app_version = self._app_version
        window_title.set_app_version(app_version)

        self._selector_window = SelectorWindow(extension_path, app_version)
        self.__build_task = asyncio.ensure_future(self.__build_layout())

    async def __build_layout(self):
        await omni.kit.app.get_app().next_update_async()
        import omni.ui as ui

        selector_handle = ui.Workspace.get_window("AppSelector")
        if selector_handle is None:
            return

        # setup the docking Space
        main_dockspace = ui.Workspace.get_window("DockSpace")

        selector_handle.dock_in(main_dockspace, ui.DockPosition.SAME)
        selector_handle.dock_tab_bar_visible = False

        await omni.kit.app.get_app().next_update_async()

    def on_shutdown(self):
        if self._selector_window:
            self._selector_window.destroy()
            self._selector_window = None
