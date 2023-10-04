# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import weakref
import asyncio
import gc
import carb
import omni
import omni.ui as ui
import omni.usd
import omni.timeline
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.widgets import DynamicComboBoxModel

from omni.isaac.ui.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    state_btn_builder,
    float_builder,
    int_builder,
    xyz_builder,
    color_picker_builder,
    setup_ui_headers,
    get_style,
    str_builder,
)
import os

from .template_generator import TemplateGenerator

EXTENSION_NAME = "Generate Extension Templates"

MAX_DOF_NUM = 100


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ui.Window(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        # self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Utils")

        self._template_generator = TemplateGenerator()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, "Isaac Utils")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            self._build_ui()

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _build_ui(self):
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_configuration_tooling_template_ui()

                self._build_loaded_scenario_template_ui()

                self._build_component_library_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html"

        overview = "Generate Extension Templates"

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_configuration_tooling_template_ui(self):
        frame = ui.CollapsableFrame(
            title="Configuration Tooling Template",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def control_generate_btn(model=None):
                    path = self._models["configuration_tooling_path"].get_value_as_string()
                    title = self._models["configuration_tooling_title"].get_value_as_string()

                    if path != "" and path[-1] != os.sep and title.strip(" ") != "":
                        self._models["configuration_tooling_generate"].enabled = True
                    else:
                        self._models["configuration_tooling_generate"].enabled = False

                self._models["configuration_tooling_path"] = str_builder(
                    label="Extension Path",
                    tooltip="Directory where the extension template will be populated. The path must not end in a slash.",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder,
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                )
                self._models["configuration_tooling_path"].add_value_changed_fn(control_generate_btn)

                self._models["configuration_tooling_title"] = str_builder(
                    label="Extension Title",
                    default_val="",
                    tooltip="Title of Extension that will show up on Isaac Sim Toolbar",
                )
                self._models["configuration_tooling_title"].add_value_changed_fn(control_generate_btn)

                self._models["configuration_tooling_description"] = str_builder(
                    label="Extension Description", default_val="", tooltip="Short description of extension"
                )

                def on_generate_extension(model=None, val=None):
                    path = self._models["configuration_tooling_path"].get_value_as_string()
                    title = self._models["configuration_tooling_title"].get_value_as_string()
                    description = self._models["configuration_tooling_description"].get_value_as_string()
                    self._template_generator.generate_configuration_tooling_template(path, title, description)

                self._models["configuration_tooling_generate"] = btn_builder(
                    label="Generate Extension",
                    text="Generate Extension",
                    tooltip="Generate Configuration Tooling Extension Template",
                    on_clicked_fn=on_generate_extension,
                )
                self._models["configuration_tooling_generate"].enabled = False

    def _build_loaded_scenario_template_ui(self):
        frame = ui.CollapsableFrame(
            title="Loaded Scenario Template",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def control_generate_btn(model=None):
                    path = self._models["loaded_scenario_path"].get_value_as_string()
                    title = self._models["loaded_scenario_title"].get_value_as_string()

                    if path != "" and path[-1] != os.sep and title.strip(" ") != "":
                        self._models["loaded_scenario_generate"].enabled = True
                    else:
                        self._models["loaded_scenario_generate"].enabled = False

                self._models["loaded_scenario_path"] = str_builder(
                    label="Extension Path",
                    tooltip="Directory where the extension template will be populated.  The path must not end in a slash",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder,
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                )
                self._models["loaded_scenario_path"].add_value_changed_fn(control_generate_btn)

                self._models["loaded_scenario_title"] = str_builder(
                    label="Extension Title",
                    default_val="",
                    tooltip="Title of Extension that will show up on Isaac Sim Toolbar",
                )
                self._models["loaded_scenario_title"].add_value_changed_fn(control_generate_btn)

                self._models["loaded_scenario_description"] = str_builder(
                    label="Extension Description", default_val="", tooltip="Short description of extension"
                )

                def on_generate_extension(model=None, val=None):
                    path = self._models["loaded_scenario_path"].get_value_as_string()
                    title = self._models["loaded_scenario_title"].get_value_as_string()
                    description = self._models["loaded_scenario_description"].get_value_as_string()
                    self._template_generator.generate_loaded_scenario_template(path, title, description)

                self._models["loaded_scenario_generate"] = btn_builder(
                    label="Generate Extension",
                    text="Generate Extension",
                    tooltip="Generate Loaded Scenario Extension Template",
                    on_clicked_fn=on_generate_extension,
                )
                self._models["loaded_scenario_generate"].enabled = False

    def _build_component_library_ui(self):
        frame = ui.CollapsableFrame(
            title="UI Component Library",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def control_generate_btn(model=None):
                    path = self._models["component_library_path"].get_value_as_string()
                    title = self._models["component_library_title"].get_value_as_string()

                    if path != "" and path[-1] != os.sep and title.strip(" ") != "":
                        self._models["component_library_generate"].enabled = True
                    else:
                        self._models["component_library_generate"].enabled = False

                self._models["component_library_path"] = str_builder(
                    label="Extension Path",
                    tooltip="Directory where the extension template will be populated.  The path must not end in a slash",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder,
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                )
                self._models["component_library_path"].add_value_changed_fn(control_generate_btn)

                self._models["component_library_title"] = str_builder(
                    label="Extension Title",
                    default_val="",
                    tooltip="Title of Extension that will show up on Isaac Sim Toolbar",
                )
                self._models["component_library_title"].add_value_changed_fn(control_generate_btn)

                self._models["component_library_description"] = str_builder(
                    label="Extension Description", default_val="", tooltip="Short description of extension"
                )

                def on_generate_extension(model=None, val=None):
                    path = self._models["component_library_path"].get_value_as_string()
                    title = self._models["component_library_title"].get_value_as_string()
                    description = self._models["component_library_description"].get_value_as_string()
                    self._template_generator.generate_component_library_template(path, title, description)

                self._models["component_library_generate"] = btn_builder(
                    label="Generate Extension",
                    text="Generate Extension",
                    tooltip="Generate Loaded Scenario Extension Template",
                    on_clicked_fn=on_generate_extension,
                )
                self._models["loaded_scenario_generate"].enabled = False
