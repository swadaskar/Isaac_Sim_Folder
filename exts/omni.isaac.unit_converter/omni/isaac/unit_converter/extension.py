# Copyright (c) 2021-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from functools import partial
import omni.ext
from omni.kit.window.filepicker.dialog import FilePickerDialog
import omni.ui as ui
import weakref
from omni.client._omniclient import Result
from pxr import Usd
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import (
    cb_builder,
    combo_cb_str_builder,
    float_builder,
    setup_ui_headers,
    get_style,
    btn_builder,
    scrolling_frame_builder,
    str_builder,
    combo_floatfield_slider_builder,
)


from pathlib import PurePosixPath, PurePath
from omni.isaac.unit_converter.unit_conversion_utils import set_stage_meters_per_unit
import os


def join(base, name):
    if base.startswith("omniverse://"):
        if name.startswith("./"):
            name = name[2:]
        while name.startswith("../"):
            base = os.path.dirname(base)
            name = name[3:]
        if base.endswith("/"):
            base = base[:-1]
        return base + "/" + name
    else:
        return os.path.join(base, name)


def filter_usd(item, ret_=False) -> bool:
    _, ext = os.path.splitext(item)
    if ext in [".usd", ".usda"]:
        return True
    return ret_


def list_sub_files(abs_path, filter_fn=lambda a: True):
    sub_folders = []
    remaining_folders = [abs_path]
    files = []
    while len(remaining_folders):
        path = remaining_folders.pop()
        result, entries = omni.client.list(path)
        if result == Result.OK:
            files = files + [
                join(path, e.relative_path) for e in entries if (e.flags & 4) == 0 and filter_fn(e.relative_path)
            ]
            remaining_folders = remaining_folders + [join(path, e.relative_path) for e in entries if (e.flags & 4) > 0]
    return files


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class UsdUnitConverter(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        self._style = get_style()
        self.folder_picker = None

        self._menu_items = [make_menu_item_description(ext_id, "USD Unit Converter", self._menu_callback)]

        add_menu_items(self._menu_items, "Isaac Utils")

    def _menu_callback(self):
        self._build_ui()

    def _build_ui(self):
        self._window = ui.Window("Usd Unit Converter", width=400, height=200)
        with self._window.frame:

            kwargs = {
                "title": "Select File/Folder",
                "apply_button_label": "Select",
                "allow_multi_selection": True,
                "item_filter_fn": lambda a: filter_usd(a.path, a.is_folder),
            }

            def file_select_cancel(a, b):
                if not self.paths:
                    self.Update_offline_assets[0].set_value(False)
                    self.Update_offline_assets[1].set_value("(Open Stage)")
                else:
                    self.on_file_select_callback(None, None)
                self.folder_picker.hide()

            self.folder_picker = weakref.proxy(
                FilePickerDialog(
                    **kwargs,
                    click_apply_handler=weakref.proxy(self).on_file_select_callback,
                    click_cancel_handler=lambda a, b: file_select_cancel(a, b),
                )
            )
            self.folder_picker.hide()
            with ui.VStack(style=self._style, spacing=5, height=0):
                with ui.ZStack(spacing=3, height=0):
                    with ui.HStack():
                        self.Update_offline_assets = combo_cb_str_builder(
                            "Update Files/folders",
                            tooltip="If selected, Updates a selected files or all files on folder, otherwise updates currently open stage",
                            on_clicked_fn=weakref.proxy(self).on_use_offline_callback,
                            default_val=[False, "(Open Stage)"],
                            read_only=True,
                        )
                        ui.Spacer(width=3)
                    with ui.HStack():
                        ui.Spacer(width=ui.Fraction(1))
                        with ui.Frame(tooltip="Select File/Folder", width=0):
                            self.picker_btn = ui.Button(
                                name="IconButton",
                                width=22,
                                height=22,
                                clicked_fn=self.on_open_picker,
                                style=get_style()["IconButton.Image::FolderPicker"],
                                alignment=ui.Alignment.RIGHT_TOP,
                            )
                        self.picker_btn.visible = False
                    self.paths = []

                with ui.HStack():
                    with ui.VStack(spacing=5):
                        self.recursive_model = cb_builder(
                            "Update All Referenced Stages",
                            default_val=False,
                            tooltip="Updates all USD files that are referenced by this asset",
                        )
                        self.mpu_model = float_builder("New Meters per Unit", default_val=1.0)
                        btn_builder(
                            "Process Stage(s)", text="Process", on_clicked_fn=weakref.proxy(self).on_process_stages
                        )

                    ui.Spacer(width=3)

    def on_process_stages(self):
        usds = []
        processed_files = set()
        if self.Update_offline_assets[0].get_value_as_bool():
            for path in self.paths:
                if filter_usd(path):
                    usds.append(path)
                else:
                    usds += list_sub_files(path, filter_usd)
            for file in usds:
                set_stage_meters_per_unit(
                    Usd.Stage.Open(file),
                    self.mpu_model.get_value_as_float(),
                    self.recursive_model.get_value_as_bool(),
                    processed_files,
                )
        else:
            set_stage_meters_per_unit(
                omni.usd.get_context().get_stage(),
                self.mpu_model.get_value_as_float(),
                self.recursive_model.get_value_as_bool(),
                processed_files,
            )

    def on_use_offline_callback(self, value):
        self.picker_btn.visible = value
        if not value:
            self.Update_offline_assets[1].set_value("(Open Stage)")
            self.folder_picker.hide()
        else:
            self.folder_picker.show()

    def on_open_picker(self):
        self.folder_picker.show()

    def on_file_select_callback(self, file, path):
        self.paths = self.folder_picker.get_current_selections(pane=omni.kit.widget.filebrowser.LISTVIEW_PANE)
        if len(self.paths) > 1:
            self.Update_offline_assets[1].set_value("(multiple)")
        elif len(self.paths) == 1:
            self.Update_offline_assets[1].set_value(self.paths[0])
        else:
            self.Update_offline_assets[0].set_value(False)
        self.folder_picker.hide()

    def on_recursive_btn_callback(self, checked):
        pass

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._window = None
        if self.folder_picker:
            self.folder_picker.destroy()
            self.folder_picker = None
