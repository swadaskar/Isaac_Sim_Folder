# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys
import os
import subprocess
import textwrap

import carb.settings
import carb.tokens
import omni.kit.app

from typing import List
from pathlib import Path

from .start_app import start_app
from .settings import (
    APPS_SETTING,
    EXPERIMENTAL_APPS_SETTING,
    AUTO_START_SETTING,
    DEFAULT_APP_SETTING,
    SHOW_CONSOLE_SETTING,
    PERSISTENT_SELECTOR_SETTING,
    EXTRA_ARGS_SETTING,
    PERSISTENT_ROS_BRIDGE_SETTING,
    ROS_BRIDGE_EXTENSIONS,
)

CURRENT_PATH = Path(__file__).parent
ICON_PATH = CURRENT_PATH.parent.parent.parent.parent.joinpath("icons")

GRAY = 0xFF4A4A4A
LIGHT_GRAY = 0xFFA8A8A8
DARK_GRAY = 0xFF363636
GREEN = 0xFF00B976
BLACK = 0xFF000000
BLUE = 0xFFF6A66B
LIGHT_BLUE = 0xFF8A8777

selector_style = {
    "Rectangle::gray_bg": {"background_color": GRAY},
    "Rectangle::active_bg": {"background_color": GREEN},
    "ScrollingFrame": {"background_color": DARK_GRAY, "padding": 15},
    "RadioButton::app": {"background_color": GRAY, "margin": 10},
    "RadioButton.Label": {"font_size": 22},
    "RadioButton:checked": {"background_color": LIGHT_BLUE},
    "Label::app_label": {"font_size": 20, "color": LIGHT_GRAY},
    # "CheckBox::checked": {"color": BLACK, "background_color": LIGHT_GRAY},
}


class SelectorWindow:
    def __init__(self, ext_path: str, app_version: str) -> None:
        """ create the window """

        self._settings = carb.settings.get_settings()
        self._radio_collection = None
        self._ext_path = ext_path
        self._app_version = app_version

        self._auto_start = None
        self._app_as_default = None
        self._apps = []
        if self._settings.get(APPS_SETTING):
            self._apps: List[str] = self._settings.get(APPS_SETTING)
        self._experimental_apps = []
        if self._settings.get(EXPERIMENTAL_APPS_SETTING):
            self._experimental_apps: List[str] = self._settings.get(EXPERIMENTAL_APPS_SETTING)

        self._auto_start = self._settings.get(AUTO_START_SETTING)
        self._default_app = self._settings.get(DEFAULT_APP_SETTING)
        self._show_console = self._settings.get(SHOW_CONSOLE_SETTING)
        self._extra_args = self._settings.get(EXTRA_ARGS_SETTING)

        self._app_list_frame = None  # the frame for the application list
        self._detail_label = None  # label of the active app
        self._detail_description = None  # label of the active app
        self._window = None

        # currently only building the compact window
        self._build_compact_window()

    def _get_all_apps(self):
        """ get the list of apps and experimental apps """
        all_apps: list[str] = self._settings.get(APPS_SETTING)
        all_apps.extend(self._settings.get(EXPERIMENTAL_APPS_SETTING))
        return all_apps

    def _start_app(self, app_id: str, app_version: str):
        """ wrapper function to help collecting the right settings to be used in the class """
        all_additional_args = str.split(self._extra_args.get_value_as_string())
        all_additional_args.extend(
            [
                "--/isaac/startup/ros_bridge_extension="
                + ROS_BRIDGE_EXTENSIONS[self._ros_bridge_selection.get_item_value_model().as_int]
            ]
        )
        start_app(
            app_id=app_id,
            app_version=app_version,
            app_become_new_default=self._app_as_default.get_value_as_bool(),
            persistent_selector=self._persistent_selector.get_value_as_bool(),
            extra_args=all_additional_args,
        )

    def _get_selected_app_id(self):
        application_index = self._radio_collection.model.get_value_as_int()
        all_apps = self._get_all_apps()
        app_id = all_apps[application_index]
        return app_id

    def _start_selected_app(self):
        app_id = self._get_selected_app_id()
        self._start_app(app_id=app_id, app_version=self._app_version)
        if self._persistent_selector.get_value_as_bool():
            # update the default app display if needed
            self._default_app = self._settings.get(DEFAULT_APP_SETTING)
            self._build_compact_app_list()

    def _close(self):
        sys.exit()

    def destroy(self):
        self._window = None

    def _appid_to_title(self, app_id: str):
        app_title = app_id.replace("omni.", "")
        app_title = app_title.replace(".", " ")
        title_words = app_title.split(" ")
        cap_words = [w.capitalize() for w in title_words]
        app_title = " ".join(cap_words)
        if app_id in self._experimental_apps:
            app_title = f"{app_title} [Experimental]"

        # Override the app title
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_dict = ext_manager.get_extension_dict(f"{app_id}-{self._app_version}")
        if ext_dict:
            app_title = ext_dict["package"]["title"]

        return app_title

    def _appid_to_description(self, app_id: str):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_dict = ext_manager.get_extension_dict(f"{app_id}-{self._app_version}")

        description = ""
        if ext_dict:
            description = ext_dict["package"]["description"]
            if "details_description" in ext_dict["package"]:
                description = ext_dict["package"]["details_description"]

        return description

    def _exit(self):
        omni.kit.app.get_app().post_quit()

    def _build_app_widget(self, app_id):
        import omni.ui as ui

        with ui.VStack(width=0):
            with ui.ZStack(style={"ZStack": {"margin": 10}}):
                bg_color = "gray_bg"
                if app_id == self._default_app:
                    bg_color = "active_bg"
                ui.Rectangle(name=bg_color)
                ui.RadioButton(
                    text=".",
                    style={"Button": {"padding": 20}, "Button.Image": {"alignment": ui.Alignment.CENTER}},
                    # image_url=f"{ICON_PATH}/{an_app}.png",
                    name="app",
                    height=175,
                    width=175,
                    clicked_fn=lambda app=app_id: self._show_details(app),
                    mouse_double_clicked_fn=lambda x, y, m, b, app=app_id: self._start_app(app, self._app_version),
                    radio_collection=self._radio_collection,
                )
            app_title = self._appid_to_title(app_id)
            ui.Label(app_title, name="app_label", height=0, alignment=ui.Alignment.CENTER)
            ui.Spacer(height=20)

    def _build_compact_app_widget(self, app_id):
        import omni.ui as ui

        with ui.HStack(width=480, height=40):
            with ui.ZStack(style={"ZStack": {"margin": 5}}):
                bg_color = "gray_bg"
                if app_id == self._default_app:
                    bg_color = "active_bg"
                ui.Rectangle(name=bg_color)
                app_title = self._appid_to_title(app_id)
                description = self._appid_to_description(app_id)

                ui.RadioButton(
                    text=app_title,
                    image_width=ui.Pixel(40),
                    image_height=ui.Pixel(32),
                    style={
                        ":": {
                            "margin": 0,
                            "stack_direction": ui.Direction.LEFT_TO_RIGHT,
                            "alignment": ui.Alignment.LEFT_CENTER,
                        },
                        "Label": {"alignment": ui.Alignment.LEFT},
                    },
                    image_url=f"{ICON_PATH}/{app_id}.png",
                    name="app",
                    # clicked_fn=lambda app=app_id: self._show_details(app),
                    mouse_double_clicked_fn=lambda x, y, m, b, app=app_id: self._start_app(app, self._app_version),
                    radio_collection=self._radio_collection,
                    tooltip=textwrap.fill(description, 40),
                )

    def _build_app_list(self):
        import omni.ui as ui

        # Application Column
        if not self._app_list_frame:
            self._app_list_frame = ui.ScrollingFrame(width=620)
        else:
            self._app_list_frame.clear()

        with self._app_list_frame:
            self._radio_collection = ui.RadioCollection()

            with ui.VStack():
                ui.Label("   Main Apps", height=0, style={"font_size": 20})
                with ui.HStack(height=230):
                    for an_app in self._apps:
                        self._build_app_widget(an_app)

                ui.Spacer(height=10)
                ui.Label("   Experimental Apps", height=0, style={"font_size": 20})
                with ui.VGrid(column_count=3, row_height=230):
                    for an_app in self._experimental_apps:
                        self._build_app_widget(an_app)

            default_index = 0
            if self._default_app:
                try:
                    all_apps = self._get_all_apps()
                    default_index = all_apps.index(self._default_app)
                except:
                    default_index = 0

            self._radio_collection.model.set_value(default_index)

    def _build_compact_app_list(self):
        import omni.ui as ui

        # Application Column
        if not self._app_list_frame:
            self._app_list_frame = ui.ScrollingFrame(width=500)
        else:
            self._app_list_frame.clear()

        with self._app_list_frame:
            self._radio_collection = ui.RadioCollection()

            with ui.VStack():
                for an_app in self._apps:
                    self._build_compact_app_widget(an_app)

                ui.Line(height=20, style={"color": GREEN, "border_width": 2})

                for an_app in self._experimental_apps:
                    self._build_compact_app_widget(an_app)

                with ui.HStack(height=0):
                    app_folder = self._settings.get_as_string("/app/folder")
                    if app_folder == "":
                        app_folder = carb.tokens.get_tokens_interface().resolve("${app}/../")
                    package_path = os.path.abspath(app_folder)
                    ui.Label("Package Path:", width=0)
                    ui.Spacer(width=5)
                    ui.StringField(tooltip=textwrap.fill(package_path, 60), read_only=True).model.set_value(
                        package_path
                    )
                    ui.Button(
                        width=25,
                        height=25,
                        style={
                            "color": 0xFF000000,
                            "Button": {"background_color": GRAY},
                            "Button:hovered": {"background_color": LIGHT_GRAY},
                        },
                        image_url=f"{ICON_PATH}/copy.svg",
                        clicked_fn=lambda: self._copy_to_clipboard(to_copy=package_path),
                        tooltip="Copy path to clipboard",
                    )
                with ui.HStack(height=0):
                    ui.Button(
                        "Open in File Browser",
                        clicked_fn=lambda: self._open_file_browser(),
                        width=ui.Percent(30),
                        style={
                            "Button": {"background_color": GRAY},
                            "Button:hovered": {"background_color": LIGHT_GRAY},
                        },
                    )
                    ui.Button(
                        "Open in Terminal",
                        clicked_fn=lambda: self._open_terminal(),
                        width=ui.Percent(30),
                        style={
                            "Button": {"background_color": GRAY},
                            "Button:hovered": {"background_color": LIGHT_GRAY},
                        },
                    )

            default_index = 0
            if self._default_app:
                try:
                    all_apps = self._get_all_apps()
                    default_index = all_apps.index(self._default_app)
                except:
                    default_index = 0

            self._radio_collection.model.set_value(default_index)

    def _show_details(self, app_id):
        app_title = self._appid_to_title(app_id)
        self._detail_label.text = app_title

        # ext_manager = omni.kit.app.get_app().get_extension_manager()
        # ext_dict = ext_manager.get_extension_dict(f"{app_id}-{self._app_version}")

        description = self._appid_to_description(app_id)

        self._detail_description.text = description

    def _build_detail_panel(self):
        import omni.ui as ui

        with ui.VStack():
            ui.Spacer(height=30)
            with ui.HStack(height=200):
                ui.Spacer()
                with ui.ZStack(width=200, heigh=200):
                    ui.Rectangle(style={"background_color": 0xFF666666})
                    with ui.Frame(style={"margin": 3}):
                        ui.Rectangle(style={"background_color": 0xFF000000})
                ui.Spacer()
            ui.Spacer(height=10)
            self._detail_label = ui.Label(
                "isaac-sim", height=0, style={"font_size": 22, "alignment": ui.Alignment.CENTER}
            )
            ui.Spacer(height=20)
            ui.Label(
                "DESCRIPTIONS",
                height=0,
                width=150,
                style={"font_size": 22, "color": 0xFFFFAA44, "alignment": ui.Alignment.CENTER},
            )
            ui.Spacer(height=10)
            with ui.HStack():
                ui.Spacer(width=12)
                self._detail_description = ui.Label(
                    "",
                    width=350,
                    word_wrap=True,
                    style={"font_size": 18, "color": 0xFFBBBBBB, "alignment": ui.Alignment.LEFT},
                )

            self._build_selector_controls()

    def _build_selector_controls(self):
        import omni.ui as ui

        with ui.VStack(height=0, style={"VStack": {"margin": 10}}):

            with ui.HStack(height=0):

                def on_value_changed(model):
                    self._settings.set_string(EXTRA_ARGS_SETTING, model.get_value_as_string())

                ui.Spacer(width=10)
                ui.Label("Extra Args:", width=0)
                ui.Spacer(width=10)
                self._extra_args = ui.StringField(
                    tooltip=textwrap.fill("Extra command line arguments to use when starting the selected app", 80)
                ).model

                self._extra_args.set_value(self._settings.get_as_string(EXTRA_ARGS_SETTING))
                self._extra_args.add_end_edit_fn(on_value_changed)

            ui.Spacer(height=5)

            with ui.HStack(height=0):
                ui.Spacer(width=10)
                ui.Label("ROS Bridge Extension", width=100, style={"font_size": 18, "color": 0xFFBBBBBB})

                ui.Spacer(width=10)
                self._ros_bridge_selection = ui.ComboBox(
                    self._settings.get_as_int(PERSISTENT_ROS_BRIDGE_SETTING),
                    *ROS_BRIDGE_EXTENSIONS,
                    tooltip=textwrap.fill("ROS Bridge to enable on startup", 80),
                ).model

                def on_clicked_wrapper(model, val):
                    self._settings.set(PERSISTENT_ROS_BRIDGE_SETTING, model.get_item_value_model().as_int)

                self._ros_bridge_selection.add_item_changed_fn(on_clicked_wrapper)
            ui.Spacer(height=5)
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                self._app_as_default = ui.CheckBox(height=10, width=30).model
                ui.Label("Set selection as new default app", width=100, style={"font_size": 18, "color": 0xFFBBBBBB})

                def on_selection_as_default_changed(model):
                    value = model.get_value_as_bool()
                    if value:
                        app_id = self._get_selected_app_id()
                        self._settings.set(DEFAULT_APP_SETTING, app_id)

                self._app_as_default.add_value_changed_fn(on_selection_as_default_changed)

            ui.Spacer(height=5)
            with ui.HStack(height=0):
                ui.Spacer(width=10)

                def on_value_changed(model):
                    value = model.get_value_as_bool()
                    self._settings.set(AUTO_START_SETTING, value)

                self._auto_start_chk = ui.CheckBox(height=10, width=30).model
                self._auto_start_chk.set_value(self._auto_start)
                self._auto_start_chk.add_value_changed_fn(on_value_changed)

                ui.Label("Automaticly start default app", width=100, style={"font_size": 18, "color": 0xFFBBBBBB})

            ui.Spacer(height=5)
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                self._persistent_selector = ui.CheckBox(height=10, width=30).model

                def on_persistent_selector_value_changed(model):
                    value = model.get_value_as_bool()
                    self._settings.set(PERSISTENT_SELECTOR_SETTING, value)

                self._persistent_selector.set_value(self._settings.get(PERSISTENT_SELECTOR_SETTING))
                self._persistent_selector.add_value_changed_fn(on_persistent_selector_value_changed)

                ui.Label("Keep App Selector window opened", width=100, style={"font_size": 18, "color": 0xFFBBBBBB})

            ui.Spacer(height=5)
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                self._show_app_console = ui.CheckBox(height=10, width=30).model

                def on_console_value_changed(model):
                    value = model.get_value_as_bool()
                    self._settings.set(SHOW_CONSOLE_SETTING, value)

                self._show_app_console.set_value(self._settings.get(SHOW_CONSOLE_SETTING))
                self._show_app_console.add_value_changed_fn(on_console_value_changed)

                ui.Label("Show startup console", width=100, style={"font_size": 18, "color": 0xFFBBBBBB})

            ui.Spacer(height=5)
            with ui.HStack(height=45, style={"font_size": 18, "margin": 4}):
                ui.Spacer(width=200)
                ui.Button(
                    "START",
                    clicked_fn=self._start_selected_app,
                    style={
                        "Button": {"background_color": GREEN},
                        "Button.Label": {"color": 0xFFFFFFFF},
                        "Button:hovered": {"background_color": 0xFF00A922, "color": 0xFFFFFFFF},
                    },
                )
                ui.Button(
                    "CLOSE",
                    clicked_fn=lambda: self._exit(),
                    style={
                        "color": 0xFF444444,
                        "Button": {"background_color": 0xFFBBBBBB},
                        "Button:hovered": {"background_color": LIGHT_GRAY},
                    },
                )

    def _build_nvidia_status_bar(self):
        import omni.ui as ui

        with ui.ZStack(height=30):
            ui.Rectangle(style={"background_color": 0xFF000000})
            with ui.HStack():
                ui.Spacer()
                with ui.VStack(width=0):
                    ui.Spacer()
                    ui.Image(f"{self._ext_path}/icons/NVIDIA_logo.png", height=18, width=150)
                    ui.Spacer()

    def _build_compact_window(self):
        import omni.ui as ui

        self._window = ui.Window("AppSelector", padding_x=0, padding_y=0, style={"Window": {"pading": 0}})
        self._window.frame.set_style(selector_style)
        with self._window.frame:
            with ui.VStack():
                self._build_compact_app_list()

                self._build_selector_controls()

                self._build_nvidia_status_bar()

    def _build_large_window(self):
        import omni.ui as ui

        self._window = ui.Window("AppSelector", padding_x=0, padding_y=0, style={"Window": {"pading": 0}})
        self._window.frame.set_style(selector_style)
        with self._window.frame:
            with ui.VStack():
                with ui.HStack():
                    # app List
                    self._build_app_list()
                    # Details Column
                    self._build_detail_panel()

                self._build_nvidia_status_bar()

    def _open_file_browser(self):
        app_folder = self._settings.get_as_string("/app/folder")
        if app_folder == "":
            app_folder = carb.tokens.get_tokens_interface().resolve("${app}/../")

        if sys.platform == "win32":
            try:
                subprocess.Popen(["start", os.path.abspath(app_folder)], shell=True)
            except OSError:
                carb.log_warn("Could not open file browser.")
        else:
            try:
                subprocess.Popen(["xdg-open", os.path.abspath(app_folder)])
            except OSError:
                carb.log_warn("Could not open file browser.")

    def _open_terminal(self):
        app_folder = self._settings.get_as_string("/app/folder")
        if app_folder == "":
            app_folder = carb.tokens.get_tokens_interface().resolve("${app}/../")

        if sys.platform == "win32":
            try:
                subprocess.Popen(["start", "cmd", "/k", "cd /d", os.path.abspath(app_folder)], shell=True)
            except OSError:
                carb.log_warn("Could not open terminal.")
        else:
            try:
                subprocess.Popen(["gnome-terminal", f"--working-directory={os.path.abspath(app_folder)}"])
            except OSError:
                carb.log_warn("Could not open terminal.")

    def _copy_to_clipboard(self, to_copy):
        try:
            import pyperclip
        except ImportError:
            carb.log_warn("Could not import pyperclip.")
            return
        try:
            pyperclip.copy(to_copy)
        except pyperclip.PyperclipException:
            carb.log_warn(pyperclip.EXCEPT_MSG)
            return
