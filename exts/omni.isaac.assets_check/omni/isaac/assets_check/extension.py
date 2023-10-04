# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import omni.ext
import omni.ui as ui
import carb.settings
import omni.kit.commands
import carb.tokens

import webbrowser

import omni.kit.app
import omni.kit.ui
import omni.appwindow
from omni.client._omniclient import Result
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description

DOCS_URL = "https://docs.omniverse.nvidia.com"
ASSETS_GUIDE_URL = DOCS_URL + "/app_isaacsim/app_isaacsim/install_basic.html#isaac-sim-first-run"


class Extension(omni.ext.IExt):
    """Create Final Configuration"""

    def on_startup(self, ext_id: str):
        """ setup the window layout, menu, final configuration of the extensions etc """
        self._settings = carb.settings.get_settings()

        # this is a work around as some Extensions don't properly setup their default setting in time
        self._set_defaults()

        self._menu_items = [make_menu_item_description(ext_id, "Nucleus Check", self._menu_callback)]
        add_menu_items(self._menu_items, "Isaac Utils")

        self.__await_new_scene = asyncio.ensure_future(self._nucleus_check_window())

    def _set_defaults(self):
        # do not display sever check pop-up on start up
        self._nucleus_check = False
        self._startup_run = True
        self._cancel_download_btn = None
        self._server_window = None
        self._check_success = None
        self._nucleus_server = None

    def _open_browser(self, path):
        import subprocess
        import platform

        if platform.system().lower() == "windows":
            webbrowser.open(path)
        else:
            # use native system level open, handles snap based browsers better
            subprocess.Popen(["xdg-open", path])

    def _menu_callback(self):
        if self._cancel_download_btn and self._cancel_download_btn.visible:
            self._server_window.visible = True
        else:
            if self._server_window and self._server_window.visible:
                self._server_window.visible = False
                self._server_window = None
            if self._check_success and self._check_success.visible:
                self._check_success.visible = False
                self._check_success = None
            asyncio.ensure_future(self._nucleus_check_window())

    async def _nucleus_check_success_window(self):
        self._check_success = ui.Window(
            "Isaac Sim Assets Check Successful",
            style={"alignment": ui.Alignment.CENTER},
            height=0,
            width=0,
            padding_x=10,
            padding_y=10,
            auto_resize=True,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_TITLE_BAR,
            visible=True,
        )

        def hide(w):
            w.visible = False

        with self._check_success.frame:
            with ui.VStack():
                ui.Spacer(height=1)
                ui.Label("Isaac Sim assets found:", style={"font_size": 18}, alignment=ui.Alignment.CENTER)
                ui.Label("{}".format(self._nucleus_server), style={"font_size": 18}, alignment=ui.Alignment.CENTER)
                ui.Spacer(height=5)
                ui.Button(
                    "OK", spacing=10, alignment=ui.Alignment.CENTER, clicked_fn=lambda w=self._check_success: hide(w)
                )
                ui.Spacer()

        await omni.kit.app.get_app().next_update_async()

    async def _nucleus_check_window(self):
        # Check Nucleus server for assets
        if self._nucleus_check is False and self._startup_run:
            self._startup_run = False
            pass
        else:
            from omni.isaac.core.utils.nucleus import get_assets_root_path

            omni.kit.app.get_app().print_and_log("Checking for Isaac Sim assets...")
            self._check_window = ui.Window("Check Isaac Sim assets", height=120, width=600)
            with self._check_window.frame:
                with ui.VStack(height=80):
                    ui.Spacer()
                    ui.Label("Checking for Isaac Sim assets", alignment=ui.Alignment.CENTER, style={"font_size": 18})
                    ui.Label(
                        "Please login to the Nucleus if a browser window appears",
                        alignment=ui.Alignment.CENTER,
                        style={"font_size": 18},
                    )
                    ui.Label(
                        "Restart of Isaac Sim is required if the browser window is closed without logging in.",
                        alignment=ui.Alignment.CENTER,
                        style={"font_size": 18},
                    )
                    ui.Spacer()
            await omni.kit.app.get_app().next_update_async()

            # Looks for assets root
            self._nucleus_server = get_assets_root_path()

            self._check_window.visible = False
            self._check_window = None
            if self._nucleus_server is None:
                self._startup_run = False

                frame_height = 150
                self._server_window = ui.Window(
                    "Checking Isaac Sim Assets", width=350, height=frame_height, visible=True
                )
                with self._server_window.frame:
                    with ui.VStack():
                        ui.Label("Warning: Isaac Sim assets not found", style={"color": 0xFF00FFFF})
                        ui.Line()
                        ui.Label("See the documentation for details")
                        ui.Button("Open Documentation", clicked_fn=lambda: self._open_browser(ASSETS_GUIDE_URL))
                        ui.Spacer()
                        ui.Label("See terminal for additional information")
            else:
                omni.kit.app.get_app().print_and_log(f"Isaac Sim assets found: {self._nucleus_server}")
                if not self._startup_run:
                    asyncio.ensure_future(self._nucleus_check_success_window())
                self._startup_run = False

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._server_window = None
        self._check_success = None
