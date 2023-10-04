# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import weakref
import asyncio

from pxr import Usd, UsdGeom, Sdf

import omni.ext
import omni.ui as ui
import omni.client
import omni.kit.commands

from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_path, is_prim_path_valid
from omni.kit.window.filepicker import FilePickerDialog
from omni.isaac.ui.ui_utils import btn_builder, get_style, setup_ui_headers


from .. import _partition
from .widgets import *


EXTENSION_NAME = "Isaac Partition"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._partition = _partition.acquire_partition_interface()
        self._usd_context = omni.usd.get_context()
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=400, height=550, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)
        menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Utils")
        self._models = {}
        self._camera_model = None
        self._camera_delegate = None
        self._style = get_style()
        self._file_picker = None

    def build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_info_ui()
                self._build_camera_ui()
                self._build_save_ui()

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
        doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_partition.html"

        overview = (
            "The Partition utility is used to convert a large USD stage to a stage with "
            "smaller layers.  The layers are generated from the view of selected cameras "
            "in the source stage.  This reorganization allows developers to work in the "
            "smaller layer(s) with all of the benefits of a smaller stage while keeping "
            "the original structure of the source stage."
        )

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_camera_ui(self):
        frame = ui.CollapsableFrame(
            title="Select Cameras",
            height=0,
            collapsed=False,
            style=self._style,
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=self._style, spacing=5, height=0):
                stage = self._usd_context.get_stage()
                prims = stage.Traverse()
                cameras = []
                i = 0
                if prims and stage:
                    for prim in prims:
                        if prim.IsA(UsdGeom.Camera):
                            label = f"{prim.GetName()}"
                            tooltip = f"{get_prim_path(prim)}"
                            targs = {
                                "label": label,
                                "id": i,
                                "default_val": False,
                                "on_value_changed_fn": self._on_camera_list_changed,
                                "tooltip": tooltip,
                            }
                            cameras.append(targs)
                            i += 1

                self._camera_model = ListItemModel(*cameras)
                self._camera_delegate = ListItemDelegate()

                self._models["camera_list"] = ui.TreeView(
                    self._camera_model,
                    delegate=self._camera_delegate,
                    root_visible=False,
                    header_visible=False,
                    style_type_name_override="CollapsableFrame",
                )

    def _build_save_ui(self):
        frame = ui.CollapsableFrame(
            title="Output",
            height=0,
            collapsed=False,
            style=self._style,
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=self._style, spacing=5, height=0):
                #
                item_filter_options = ["USD Files (*.usd, *.usda, *.usdc, *.usdz)", "All Files (*)"]

                kwargs = {"title": "Export Partition", "apply_button_label": "Export", "allow_multi_selection": False}

                def file_select_cancel(a, b):
                    self._file_picker.hide()

                self._file_picker = weakref.proxy(
                    FilePickerDialog(
                        **kwargs,
                        click_apply_handler=weakref.proxy(self).on_file_select_callback,
                        click_cancel_handler=lambda a, b: file_select_cancel(a, b),
                        item_filter_options=item_filter_options,
                    )
                )
                self._file_picker.hide()
                self._models["partition_button"] = btn_builder(
                    "Export Partition", text="SELECT CAMERAS", on_clicked_fn=self._save_partition
                )
                self._models["partition_button"].enabled = False

    def _on_camera_list_changed(self, name, model, id):
        self._models["partition_button"].enabled = False
        self._models["partition_button"].text = "SELECT CAMERAS"
        for camera in self._camera_model.get_item_children(None):
            if camera.model.get_value_as_bool():
                self._models["partition_button"].enabled = True
                self._models["partition_button"].text = "CLICK TO EXPORT"

    def on_open_picker(self):
        self._file_picker.show()

    def on_file_select_callback(self, file, dir):
        # clear the cameras.
        self._partition.clear_cameras()

        # add new cameras iff we have a path.
        if file and dir:
            for camera in self._camera_model.get_item_children(None):
                if camera.model.get_value_as_bool():
                    self._partition.add_camera_path(camera.tooltip)

            self._partition.set_export_path(f"{dir}/{file}" if dir else file)
            self._partition.save_to_usd()

        self._file_picker.hide()

    def on_recursive_btn_callback(self, checked):
        pass

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        if self._window.visible:
            self.build_ui()
        self._file_picker.hide()

    def _save_partition(self, path=None):
        self._file_picker.show()

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._file_picker
        self._window = None
        _partition.release_partition_interface(self._partition)
        self._partition = None
