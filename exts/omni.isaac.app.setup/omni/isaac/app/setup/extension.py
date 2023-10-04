# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import sys
import os.path
import omni.ext
import omni.ui as ui
import carb.settings
import omni.kit.commands
import carb.imgui as _imgui
import carb.tokens
import webbrowser
import typing

import omni.kit.app
import omni.kit.ui
import omni.appwindow
import omni.kit.stage_templates as stage_templates
from omni.kit.window.title import get_main_window_title
from carb.input import KeyboardInput as Key
from omni.client._omniclient import Result, CopyBehavior
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.version import get_version


DOCS_URL = "https://docs.omniverse.nvidia.com"
REFERENCE_GUIDE_URL = DOCS_URL + "/isaacsim"
ASSETS_GUIDE_URL = DOCS_URL + "/app_isaacsim/app_isaacsim/install_basic.html#isaac-sim-first-run"
FORUMS_URL = "https://forums.developer.nvidia.com/c/omniverse/simulation/69"
KIT_MANUAL_URL = DOCS_URL + "/py/kit/index.html"

from pathlib import Path

DATA_PATH = Path(__file__).parent.parent.parent.parent.parent


async def _load_layout(layout_file: str, keep_windows_open=False):
    try:
        from omni.kit.quicklayout import QuickLayout

        # few frames delay to avoid the conflict with the layout of omni.kit.mainwindow
        for i in range(3):
            await omni.kit.app.get_app().next_update_async()
        QuickLayout.load_file(layout_file, keep_windows_open)

    except Exception as exc:
        pass

        QuickLayout.load_file(layout_file)


class CreateSetupExtension(omni.ext.IExt):
    """Create Final Configuration"""

    def on_startup(self, ext_id: str):
        """setup the window layout, menu, final configuration of the extensions etc"""
        self._settings = carb.settings.get_settings()
        self._ext_manager = omni.kit.app.get_app().get_extension_manager()

        self._menu_layout = []

        # this is a work around as some Extensions don't properly setup their default setting in time
        self._set_defaults()

        # adjust couple of viewport settings
        self._settings.set("/app/viewport/grid/enabled", True)
        self._settings.set("/app/viewport/outline/enabled", True)
        self._settings.set("/app/viewport/boundingBoxes/enabled", False)

        # Adjust the Window Title to show the Isaac Sim Version
        window_title = get_main_window_title()
        app_version_core, app_version_prerel, _, _, _, _, _, _ = get_version()
        window_title.set_app_version(app_version_core)
        self.app_title = self._settings.get("/app/window/title")
        omni.kit.app.get_app().print_and_log(f"{self.app_title} Version: {app_version_core}-{app_version_prerel}")

        # setup some imgui Style overide
        imgui = _imgui.acquire_imgui()
        imgui.push_style_color(_imgui.StyleColor.ScrollbarGrab, carb.Float4(0.4, 0.4, 0.4, 1))
        imgui.push_style_color(_imgui.StyleColor.ScrollbarGrabHovered, carb.Float4(0.6, 0.6, 0.6, 1))
        imgui.push_style_color(_imgui.StyleColor.ScrollbarGrabActive, carb.Float4(0.8, 0.8, 0.8, 1))

        imgui.push_style_var_float(_imgui.StyleVar.DockSplitterSize, 2)

        self.__setup_window_task = asyncio.ensure_future(self.__dock_windows())
        self.__setup_property_window = asyncio.ensure_future(self.__property_window())
        self.__enable_ros_bridge()
        self.__menu_update()
        self.__add_app_icon(ext_id)
        self.__await_new_scene = asyncio.ensure_future(self.__new_stage())

        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)

    def _set_defaults(self):
        """this is trying to setup some defaults for extensions to avoid warning"""
        self._settings.set_default("/persistent/app/omniverse/bookmarks", {})
        self._settings.set_default("/persistent/app/stage/timeCodeRange", [0, 100])

        self._settings.set_default("/persistent/audio/context/closeAudioPlayerOnStop", False)

        self._settings.set_default("/persistent/app/primCreation/PrimCreationWithDefaultXformOps", True)
        self._settings.set_default("/persistent/app/primCreation/DefaultXformOpType", "Scale, Orient, Translate")
        self._settings.set_default("/persistent/app/primCreation/DefaultRotationOrder", "ZYX")
        self._settings.set_default("/persistent/app/primCreation/DefaultXformOpPrecision", "Double")

        # omni.kit.property.tagging
        self._settings.set_default("/persistent/exts/omni.kit.property.tagging/showAdvancedTagView", False)
        self._settings.set_default("/persistent/exts/omni.kit.property.tagging/showHiddenTags", False)
        self._settings.set_default("/persistent/exts/omni.kit.property.tagging/modifyHiddenTags", False)

        # force this on startup for better performance on larger scenes when selecting objects.
        self._settings.set("persistent/app/viewport/pickingMode", "models")
        # camera settings
        self._settings.set("persistent/app/viewport/camShowSpeedOnStart", False)
        self._settings.set("persistent/app/omniverse/gamepadCameraControl", False)
        # physics settings
        self._settings.set("persistent/physics/resetOnStop", True)
        # omnigraph settings
        self._settings.set("persistent/omnigraph/disablePrimNodes", True)
        self._settings.set("persistent/omnigraph/useSchemaPrims", True)

    async def __new_stage(self):

        from omni.kit.viewport.utility import next_viewport_frame_async, get_active_viewport

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
        if omni.usd.get_context().can_open_stage():
            stage_templates.new_stage(template=None)
        # 10 frame delay to allow Layout
        await next_viewport_frame_async(get_active_viewport())
        await omni.kit.app.get_app().next_update_async()

        # Let users know when app is ready for use and live-streaming
        omni.kit.app.get_app().print_and_log(f"{self.app_title} App is loaded.")

        await omni.kit.app.get_app().next_update_async()

    def _start_app(self, app_id, console=True, custom_args=None):
        """start another Kit app with the same settings"""
        import sys
        import subprocess
        import platform

        kit_exe_path = os.path.join(os.path.abspath(carb.tokens.get_tokens_interface().resolve("${kit}")), "kit")
        if sys.platform == "win32":
            kit_exe_path += ".exe"

        app_path = carb.tokens.get_tokens_interface().resolve("${app}")
        kit_file_path = os.path.join(app_path, app_id)

        run_args = [kit_exe_path]
        run_args += [kit_file_path]
        if custom_args:
            run_args.extend(custom_args)

        # Pass all exts folders
        exts_folders = self._settings.get("/app/exts/folders")
        if exts_folders:
            for folder in exts_folders:
                run_args.extend(["--ext-folder", folder])

        kwargs = {"close_fds": False}
        if platform.system().lower() == "windows":
            if console:
                kwargs["creationflags"] = subprocess.CREATE_NEW_CONSOLE | subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

        subprocess.Popen(run_args, **kwargs)

    def _show_ui_docs(self):
        """show the omniverse ui documentation as an external Application"""
        self._start_app("omni.app.uidoc.kit")

    def _show_selector(self):
        """show the app selector as an external Application"""
        self._start_app(
            "omni.isaac.sim.selector.kit",
            console=False,
            custom_args={"--/persistent/ext/omni.isaac.selector/auto_start=false"},
        )

    async def __dock_windows(self):
        await omni.kit.app.get_app().next_update_async()

        content = ui.Workspace.get_window("Content")
        stage = ui.Workspace.get_window("Stage")
        layer = ui.Workspace.get_window("Layer")
        console = ui.Workspace.get_window("Console")

        await omni.kit.app.get_app().next_update_async()
        if layer:
            layer.dock_order = 1
        if stage:
            stage.dock_order = 0
            stage.focus()

        await omni.kit.app.get_app().next_update_async()
        if console:
            console.dock_order = 1
        if content:
            content.dock_order = 0
            content.focus()

    async def __property_window(self):
        await omni.kit.app.get_app().next_update_async()
        import omni.kit.window.property as property_window_ext

        property_window = property_window_ext.get_window()
        property_window.set_scheme_delegate_layout(
            "Create Layout", ["path_prim", "material_prim", "xformable_prim", "shade_prim", "camera_prim"]
        )

    def _add_menu(self, *argv, **kwargs):
        new_menu = omni.kit.ui.get_editor_menu().add_item(*argv, **kwargs)
        self.menus.append(new_menu)
        return new_menu

    def _open_browser(self, path):
        import subprocess
        import platform

        if platform.system().lower() == "windows":
            webbrowser.open(path)
        else:
            # use native system level open, handles snap based browsers better
            subprocess.Popen(["xdg-open", path])

    def _open_web_file(self, path):
        filepath = os.path.abspath(path)
        if os.path.exists(filepath):
            self._open_browser("file://" + filepath)
        else:
            carb.log_warn("Failed to open " + filepath)

    def get_manual_url_path(self):
        manual_path = os.path.abspath(carb.tokens.get_tokens_interface().resolve("${app}/../docs/py/index.html"))
        if os.path.isfile(manual_path):
            return manual_path
        return None

    def __menu_update(self):

        self.HELP_REFERENCE_GUIDE_MENU = (
            f'Help/{omni.kit.ui.get_custom_glyph_code("${glyphs}/cloud.svg")} Isaac Sim Online Guide'
        )
        self.HELP_SCRIPTING_MANUAL = (
            f'Help/{omni.kit.ui.get_custom_glyph_code("${glyphs}/book.svg")} Isaac Sim Scripting Manual'
        )
        self.HELP_FORUMS_URL = (
            f'Help/{omni.kit.ui.get_custom_glyph_code("${glyphs}/cloud.svg")} Isaac Sim Online Forums'
        )
        self.HELP_UI_DOCS = f'Help/{omni.kit.ui.get_custom_glyph_code("${glyphs}/book.svg")} Omni UI Docs'
        self.HELP_KIT_MANUAL = f'Help/{omni.kit.ui.get_custom_glyph_code("${glyphs}/cloud.svg")} Kit Programming Manual'
        self.menus = []

        # seperator
        priority = 50

        editor_menu = omni.kit.ui.get_editor_menu()

        ref_guide_menu = editor_menu.add_item(self.HELP_REFERENCE_GUIDE_MENU, None, priority=-23)
        ref_guide_menu_action = omni.kit.menu.utils.add_action_to_menu(
            self.HELP_REFERENCE_GUIDE_MENU,
            lambda *_: self._open_browser(REFERENCE_GUIDE_URL),
            "OpenRefGuide",
            (0, Key.F3),
        )
        self.menus.append((ref_guide_menu, ref_guide_menu_action))

        manual_url_path = self.get_manual_url_path()
        self._add_menu(self.HELP_SCRIPTING_MANUAL, lambda *_: self._open_web_file(manual_url_path), priority=-22)
        if manual_url_path is None:
            editor_menu.set_enabled(self.HELP_SCRIPTING_MANUAL, False)

        forums_link = editor_menu.add_item(self.HELP_FORUMS_URL, None, priority=-21)
        forums_link_action = omni.kit.menu.utils.add_action_to_menu(
            self.HELP_FORUMS_URL, lambda *_: self._open_browser(FORUMS_URL), "OpenForums"
        )
        self.menus.append((forums_link, forums_link_action))

        kit_manual = editor_menu.add_item(self.HELP_KIT_MANUAL, None, priority=-9)
        kit_manual_action = omni.kit.menu.utils.add_action_to_menu(
            self.HELP_KIT_MANUAL, lambda *_: self._open_browser(KIT_MANUAL_URL), "OpenKitManual"
        )
        self.menus.append((kit_manual, kit_manual_action))

        # set omnu.ui Help Menu
        self._ui_doc_menu_item = editor_menu.add_item(self.HELP_UI_DOCS, lambda *_: self._show_ui_docs())
        editor_menu.set_priority(self.HELP_UI_DOCS, -10)

        # set Selector Menu
        self._ui_selector_menu_path = "Help/Isaac Sim App Selector"
        self._ui_selector_menu_item = editor_menu.add_item(
            self._ui_selector_menu_path, lambda *_: self._show_selector()
        )
        editor_menu.set_priority(self._ui_selector_menu_path, 20)

        editor_menu.set_priority("Help", 200)

        from omni.kit.menu.utils import MenuLayout

        self._menu_layout = [
            MenuLayout.Menu(
                "Window",
                [
                    MenuLayout.SubMenu(
                        "Animation",
                        [
                            MenuLayout.Item("Timeline"),
                            MenuLayout.Item("Sequencer"),
                            MenuLayout.Item("Curve Editor"),
                            MenuLayout.Item("Retargeting"),
                            MenuLayout.Item("Animation Graph"),
                            MenuLayout.Item("Animation Graph Samples"),
                            # MenuLayout.Item("Keyframer"),
                            # MenuLayout.Item("Recorder"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Layout",
                        [MenuLayout.Item("Quick Save", remove=True), MenuLayout.Item("Quick Load", remove=True)],
                    ),
                    MenuLayout.SubMenu(
                        "Browsers",
                        [
                            MenuLayout.Item("Content", source="Window/Content"),
                            MenuLayout.Item("Materials"),
                            MenuLayout.Item("Skies"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Rendering",
                        [
                            MenuLayout.Item("Render Settings"),
                            MenuLayout.Item("Movie Capture"),
                            MenuLayout.Item("MDL Material Graph"),
                            MenuLayout.Item("Tablet XR"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Simulation",
                        [
                            MenuLayout.Group("Flow", source="Window/Flow"),
                            MenuLayout.Group("Blast Destruction", source="Window/Blast"),
                            MenuLayout.Group("Blast Destruction", source="Window/Blast Destruction"),
                            MenuLayout.Group("Boom Collision Audio", source="Window/Boom"),
                            MenuLayout.Group("Boom Collision Audio", source="Window/Boom Collision Audio"),
                            MenuLayout.Group("Physics", source="Window/Physics"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Utilities",
                        [
                            MenuLayout.Item("Console"),
                            MenuLayout.Item("Profiler"),
                            MenuLayout.Item("USD Paths"),
                            MenuLayout.Item("Statistics"),
                            MenuLayout.Item("Activity Monitor"),
                            MenuLayout.Item("Actions"),
                        ],
                    ),
                    MenuLayout.Sort(exclude_items=["Extensions"], sort_submenus=True),
                    MenuLayout.Item("New Viewport Window", remove=True),
                    # MenuLayout.Item("Material Preview", remove=True),
                ],
            ),
            MenuLayout.Menu(
                "Layout",
                [
                    # MenuLayout.Item("Default", source="Reset Layout"),
                    # MenuLayout.Item("Animation"),
                    # MenuLayout.Item("Animation Graph"),
                    # MenuLayout.Item("Paint"),
                    # MenuLayout.Item("Rendering"),
                    # MenuLayout.Item("Visual Scripting"),
                    # MenuLayout.Seperator(),
                    MenuLayout.Item("UI Toggle Visibility", source="Window/UI Toggle Visibility"),
                    MenuLayout.Item("Fullscreen Mode", source="Window/Fullscreen Mode"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item("Save Layout", source="Window/Layout/Save Layout..."),
                    MenuLayout.Item("Load Layout", source="Window/Layout/Load Layout..."),
                    MenuLayout.Seperator(),
                    MenuLayout.Item("Quick Save", source="Window/Layout/Quick Save"),
                    MenuLayout.Item("Quick Load", source="Window/Layout/Quick Load"),
                ],
            ),
            MenuLayout.Menu(
                "Replicator",
                [
                    MenuLayout.Item("Synthetic Data Recorder", source="Replicator/Synthetic Data Recorder"),
                    MenuLayout.Item("Semantics Schema Editor", source="Replicator/Semantics Schema Editor"),
                    MenuLayout.Item("Composer", source="Replicator/Composer"),
                    MenuLayout.Seperator(),
                ],
            ),
        ]
        omni.kit.menu.utils.add_layout(self._menu_layout)

        self._layout_menu_items = []
        self._current_layout_priority = 20

        def add_layout_menu_entry(name, parameter, key):
            import inspect

            menu_path = f"Layout/{name}"
            menu = editor_menu.add_item(menu_path, None, False, self._current_layout_priority)
            self._current_layout_priority = self._current_layout_priority + 1

            if inspect.isfunction(parameter):
                menu_action = omni.kit.menu.utils.add_action_to_menu(
                    menu_path,
                    lambda *_: asyncio.ensure_future(parameter()),
                    name,
                    (carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, key),
                )
            else:
                menu_action = omni.kit.menu.utils.add_action_to_menu(
                    menu_path,
                    lambda *_: asyncio.ensure_future(_load_layout(f"{DATA_PATH}/layouts/{parameter}.json")),
                    name,
                    (carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, key),
                )

            self._layout_menu_items.append((menu, menu_action))

        # add_layout_menu_entry("Reset Layout", "default", carb.input.KeyboardInput.KEY_1)
        # add_layout_menu_entry("Animation", "animation", carb.input.KeyboardInput.KEY_2)
        # add_layout_menu_entry("Animation Graph", "animationGraph", carb.input.KeyboardInput.KEY_3)
        # add_layout_menu_entry("Paint", "paint", carb.input.KeyboardInput.KEY_4)
        # add_layout_menu_entry("Rendering", "rendering", carb.input.KeyboardInput.KEY_5)
        # add_layout_menu_entry("Visual Scripting", "visualScripting", carb.input.KeyboardInput.KEY_6)

        # create Quick Load & Quick Save
        from omni.kit.quicklayout import QuickLayout

        async def quick_save():
            QuickLayout.quick_save(None, None)

        async def quick_load():
            QuickLayout.quick_load(None, None)

        add_layout_menu_entry("Quick Save", quick_save, carb.input.KeyboardInput.KEY_7)
        add_layout_menu_entry("Quick Load", quick_load, carb.input.KeyboardInput.KEY_8)

    def __add_app_icon(self, ext_id):

        extension_path = self._ext_manager.get_extension_path(ext_id)
        if sys.platform == "win32":
            pass
        else:
            user_apps_folder = os.path.expanduser("~/.local/share/applications")
            if os.path.exists(user_apps_folder):
                with open(os.path.expanduser("~/.local/share/applications/IsaacSim.desktop"), "w") as file:
                    omni.kit.app.get_app().print_and_log("Writing Isaac Sim icon file")
                    file.write(
                        f"""[Desktop Entry]
Version=1.0
Name=Isaac Sim
Icon={extension_path}/data/omni.isaac.sim.png
Terminal=false
Type=Application
StartupWMClass=IsaacSim"""
                    )

    def __enable_ros_bridge(self):
        ros_bridge_name = self._settings.get("isaac/startup/ros_bridge_extension")
        if ros_bridge_name is not None and len(ros_bridge_name):
            self._ext_manager.set_extension_enabled_immediate(ros_bridge_name, True)

    def on_shutdown(self):
        omni.kit.menu.utils.remove_layout(self._menu_layout)
        self._menu_layout = None
        self._layout_menu_items = None
        self._ui_doc_menu_item = None
