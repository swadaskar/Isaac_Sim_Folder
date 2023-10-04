# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb, omni.ext, omni.kit.commands, omni.ui as ui, os, asyncio
from enum import Enum
import pxr
from pxr import UsdGeom, UsdShade, Sdf, Gf, Vt, UsdLux, Usd, Kind
from pxr.Vt import IntArray, Vec3fArray, Vec2fArray, DoubleArray
import io
from PIL import Image, ImageChops
import numpy as np
import asyncio
import threading
import time
import signal
import json
import omni
import uuid

import base64

from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.client import OnshapeClient


class PhysicsMaterial(ui.AbstractItem):
    def __init__(self, item=None, lib=None, from_metadata=False):
        super().__init__()
        self._metadata_dict = None
        if item and from_metadata:
            # print(item)
            self._metadata_dict = item
            self._item = {
                "displayName": item["displayName"],
                "id": item["id"],
                "propertyValues": {"DENS": item["properties"][0]["value"]},
            }

            # mat_id = item["id"]
            # mat = [m for m in lib["materials"] if m["id"] == mat_id]
            # if mat:
            #     self._item = mat
        elif item:
            self._item = item
        else:
            self._item = {
                "displayName": "Select a Material",
                "category": "None",
                "id": None,
                "propertyValues": {"DENS": "1.0"},
            }
        if not self._metadata_dict:
            self._metadata_dict = {
                "libraryName": lib["displayName"] if lib else "",
                "displayName": self._item["displayName"],
                "properties": [
                    {
                        "units": "kg/m^3",
                        "displayName": "Density",
                        "type": "REAL",
                        "name": "DENS",
                        "value": self._item["propertyValues"]["DENS"],
                        "description": "Density",
                        "category": "Physical",
                    }
                ],
                "id": self.get_id(),
                "libraryReference": lib["externalElementReference"] if lib else None,
            }
        self._value_cols = [self.get_name, self.get_density, self.get_category, self.get_lib_id]

    def get_lib_id(self):
        return self._metadata_dict["libraryName"]

    def get_id(self):
        return self._item["id"]

    def get_display_name(self):
        return "{} ({} kg/m^3)".format(self.get_name(), self.get_density())

    def get_name(self):
        return self._metadata_dict["displayName"]

    def get_density(self):
        return float(self._item["propertyValues"]["DENS"])

    def get_category(self):
        return self._item["category"]

    def get_metadata_dict(self):
        return self._metadata_dict

    def get_value(self, column_id):
        return self._value_cols[column_id]()


class PhysicsMaterialsModel(ui.AbstractItemModel):
    def __init__(self, libraries=[], material=None):
        super().__init__()
        self._selection = None
        self._materials = []

        if not libraries:
            libraries = OnshapeClient.get_default_material_libraries()
        # print(self, str(len(libraries)))
        self._libraries = libraries
        # Ensure material library is on set of libraries
        if material:
            mlib = material["libraryReference"]
            if mlib["documentId"] + mlib["elementId"] not in [
                l["externalElementReference"]["documentId"] + l["externalElementReference"]["elementId"]
                for l in self._libraries
            ]:
                self._libraries += [OnshapeClient.get_material_library(mlib["documentId"], mlib["elementId"])]
        for lib in self._libraries:
            self._materials += [PhysicsMaterial(item, lib) for item in lib["materials"]]
        if material:
            # print(self, "setting selection")
            self._selection = PhysicsMaterial(material, None, from_metadata=True)
        # print(self, "selection set")
        self._categories = set([c.get_category() for c in self._materials])
        self._children = [m for m in self._materials]

    @property
    def selection(self):
        return self._selection

    @selection.setter
    def selection(self, sel):
        if isinstance(sel, PhysicsMaterial):
            self._selection = sel
        self._item_changed(None)

    def list_materials(self, query="", lib_filter=None, category_filter=None):
        self._children = [
            c
            for c in self._materials
            if query.lower() in c.get_name().lower()
            and (not lib_filter or c.get_lib_id() in lib_filter)
            and (not category_filter or c.get_category() in category_filter)
        ]
        self._item_changed(None)

    def get_item_value(self, item=None, column_id=0):
        if item:
            return item.get_value(column_id)
        return None

    def get_item_value_model(self, item=None, column_id=0):
        return super().get_item_value_model(item=item, column_id=column_id)

    def get_item_value_model_count(self, item=None):
        return 3  # Name, Density, Category, Library

    def get_item_children(self, item=None):
        if item is not None:
            return []
        return self._children

    def get_libraries(self):
        return [l["name"] for l in self._libraries]

    def get_categories(self):
        return self._categories


class PhysicsMaterialsItemDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self.header = ["Name", "Density", "Category", "Library"]

    def build_branch(self, model, item=None, column_id=0, level=0, expanded=False):
        pass

    def build_header(self, column_id=0):
        ui.Label(self.header[column_id])

    def build_widget(self, model, item=None, index=0, level=0, expanded=False):
        ui.Label(str(model.get_item_value(item, index)))


class PhysicsMaterialsWidget(ui.Widget):
    def __init__(self, **kwargs):
        self.theme = kwargs.get("theme", "NvidiaDark")
        self._on_selection_changed_fn = kwargs.get("selection_changed_fn", None)
        self._style = UI_STYLES[self.theme]
        material = kwargs.get("selection", None)
        self.delegate = PhysicsMaterialsItemDelegate()
        self._frame = ui.Frame(**kwargs)
        self._libs = []
        self._cats = []
        self.edit_stack = ui.Window(
            str(uuid.uuid1()),
            width=100,
            height=200,
            flags=ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_TITLE_BAR
            | ui.WINDOW_FLAGS_NO_MOVE,
            visible=False,
        )
        self.model = kwargs.get("model", PhysicsMaterialsModel(None, material))
        self.model.subscribe_item_changed_fn(self.build_ui)
        self.build_ui()
        self.abort_display = False

    @property
    def selection(self):
        return self.model.selection

    @selection.setter
    def selection(self, sel):
        self.model.selection = sel
        if self.model.selection not in self.mat_list.selection:
            self.mat_list.selection = [self.model.selection]
            if self._on_selection_changed_fn:
                self._on_selection_changed_fn(self.model.selection)
        self.build_ui()

    def get_display_name(self):
        # print("Get mat name")
        return self.selection.get_display_name() if self.selection else "--None--"

    def display_tooltip(self):
        ui.Label(self.get_display_name(), style_type_name_override="Tooltip")

    def build_ui(self, *args):
        with self._frame:
            with ui.ZStack(width=ui.Percent(100)):
                # self.edit_stack = ui.VStack(visible=False)
                ui.Rectangle(height=ui.Pixel(22), style={"background_color": 0x55000000, "border_radius": 3})

                self.display_stack = ui.HStack()
                with self.display_stack:
                    ui.Spacer(width=ui.Pixel(5))
                    self.search_label = ui.Label(
                        self.get_display_name(),
                        height=22,
                        name="search" if self.selection else "error",
                        style=self._style,
                        elided_text=True,
                        tooltip_fn=self.display_tooltip,
                    )

                    self.open_drop_btn = ui.Button(
                        name="arrow_down", style=self._style, clicked_fn=self.edit_mode, width=22, height=22
                    )
                self.display_stack.set_mouse_pressed_fn(self.edit_mode)
        with self.edit_stack.frame:
            with ui.VStack():
                with ui.ZStack(height=ui.Pixel(22)):
                    ui.Rectangle(
                        width=ui.Percent(100),
                        height=ui.Pixel(22),
                        style={"background_color": 0x55000000, "border_radius": 3},
                    )
                    with ui.HStack(height=22):
                        ui.Spacer(width=ui.Pixel(2))
                        self.queryField = ui.StringField(height=ui.Pixel(22))
                        self._query_changed_subs = self.queryField.model.subscribe_value_changed_fn(
                            lambda a: self.query_changed()
                        )
                        self._end_edit_subs = self.queryField.model.subscribe_end_edit_fn(lambda a: self.display_mode())
                        self._filter_btn = ui.Button(
                            name="filter",
                            width=22,
                            height=22,
                            clicked_fn=lambda: self.show_filter_menu(),
                            style=self._style,
                        )
                        self.menu_bar = ui.MenuBar()
                        self.menu_bar.visible = False
                        with self.menu_bar:
                            self._filter_menu = ui.Menu("Filter")
                            with self._filter_menu:
                                with ui.Menu("Libraries"):
                                    ui.MenuItem(
                                        "Clear selection",
                                        checkable=False,
                                        triggered_fn=lambda: self.clear_selection(self._libs),
                                    )
                                    ui.Separator()
                                    for _lib in self.model.get_libraries():
                                        self._libs.append(ui.MenuItem(_lib, checkable=True, checked=False))
                                        self._libs[-1].set_checked_changed_fn(lambda a: self.query_changed())
                                with ui.Menu("Categories"):
                                    ui.MenuItem(
                                        "Clear selection",
                                        checkable=False,
                                        triggered_fn=lambda: self.clear_selection(self._cats),
                                    )
                                    ui.Separator()
                                    for _cat in self.model.get_categories():
                                        self._cats.append(ui.MenuItem(_cat, checkable=True, checked=False))
                                        self._cats[-1].set_checked_changed_fn(lambda a: self.query_changed())
                with ui.ScrollingFrame(style=self._style, style_type_name_override="TreeView"):

                    self.mat_list = self._tree_view = ui.TreeView(
                        self.model,
                        delegate=self.delegate,
                        header_visible=True,
                        root_visible=False,
                        columns_resizable=False,
                        column_widths=[ui.Fraction(1), 50, 60],
                    )
                    self.mat_list.set_selection_changed_fn(lambda a: self.on_selection_changed(a))

    def show_filter_menu(self):
        self.abort_display = True
        self._filter_menu.show()

    def clear_selection(self, _list):
        for item in _list:
            item.checked = False

    def edit_mode(self, *args):
        if len(args) < 3 or args[2] == 0:
            self.mat_list.clear_selection()
            self.display_stack.visible = False
            self.edit_stack.position_x = self._frame.screen_position_x - 4
            self.edit_stack.position_y = self._frame.screen_position_y - 4
            self.edit_stack.width = self._frame.computed_width + 6
            self.edit_stack.visible = True
            self.query_changed()

    def display_mode(self, immediate=False):
        def display():
            self.abort_display = False
            if not immediate:
                time.sleep(0.2)
            if not self.abort_display:
                self.display_stack.visible = True
                self.edit_stack.visible = False
                self.search_label.name = "search" if self.selection else "error"
                self.search_label.text = self.get_display_name()
                self.search_label.set_tooltip(self.get_display_name())

        task = threading.Thread(target=display)
        task.start()

    def on_selection_changed(self, a=None):
        self.abort_display = True
        if isinstance(a, ui.SimpleStringModel):
            self.display_mode()
            return
        if self.edit_stack.visible:
            # print(self.mat_list.selection, self.selection)
            self.selection = self.mat_list.selection[0] if self.mat_list.selection else self.selection

            self.display_mode(immediate=True)
            if self._on_selection_changed_fn:
                # print("Material changed:", self.selection)
                self._on_selection_changed_fn(self.selection)

    def get_selection(self):
        if self.mat_list.selection:
            return self.mat_list.selection[0]
        return None

    def query_changed(self):
        q = self.queryField.model.get_value_as_string()
        libs = [i.text for i in self._libs if i.checked]
        cats = [i.text for i in self._cats if i.checked]
        self.model.list_materials(q, libs, cats)
        self.queryField.focus_keyboard()
