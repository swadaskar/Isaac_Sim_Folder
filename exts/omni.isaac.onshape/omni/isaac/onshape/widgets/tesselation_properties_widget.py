# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
import carb, omni.ext, omni.kit.commands, omni.ui as ui, os, asyncio
from enum import Enum
from pathlib import Path

import uuid
from math import radians, degrees

from omni.isaac.onshape.scripts.style import UI_STYLES

from ..scripts.definitions import ONSHAPE_CHORD_TOLERANCE, ONSHAPE_ANGLE_TOLERANCE, ONSHAPE_MAX_CHORD

# defaults = [1.0, 0.25, 0.02]
defaults = [0.001, 15, 0.2]
mins = [0.0, 0.0, 0.0]
maxs = [10.0, 90, 10]

if carb.settings.get_settings().get(ONSHAPE_CHORD_TOLERANCE) is None:
    carb.settings.get_settings().set(ONSHAPE_CHORD_TOLERANCE, 0.001)
if carb.settings.get_settings().get(ONSHAPE_ANGLE_TOLERANCE) is None:
    carb.settings.get_settings().set(ONSHAPE_ANGLE_TOLERANCE, 15)
if carb.settings.get_settings().get(ONSHAPE_MAX_CHORD) is None:
    carb.settings.get_settings().set(ONSHAPE_MAX_CHORD, 0.2)


class TesselationProperties:
    def __init__(
        self, chord_tolerance=None, angle_tolerance=None, max_chord=carb.settings.get_settings().get(ONSHAPE_MAX_CHORD)
    ):
        self.angle_tolerance = radians(
            angle_tolerance if angle_tolerance else carb.settings.get_settings().get(ONSHAPE_ANGLE_TOLERANCE)
        )
        self.chord_tolerance = (
            chord_tolerance if chord_tolerance else carb.settings.get_settings().get(ONSHAPE_CHORD_TOLERANCE)
        )
        self.max_chord = max_chord if max_chord else carb.settings.get_settings().get(ONSHAPE_MAX_CHORD)


def image_tooltip(text, image_src, width, height):
    """Base Image tooltip function used on the UI"""
    with ui.VStack(width=width, height=height):
        ui.Label(text, width=width, height=20, style_type_name_override="Tooltip")
        ui.Image(image_src, height=(height - 20))


def chord_tolerance_tooltip():
    ui.Label("The maximum distance between a curve and its approximation chord (m)", style_type_name_override="Tooltip")
    # image_tooltip(
    #     "The maximum distance between a curve and its approximation chord (m)",
    #     str(Path(__file__).parent.joinpath("data/linear_displacement.png")),
    #     400,
    #     220,
    # )


def angle_tolerance_tooltip():
    ui.Label(
        "The maximum angle difference between two consecutive tangents across the tesselated surface (degrees)",
        style_type_name_override="Tooltip",
    )


# image_tooltip(
#     "The maximum angle difference between two consecutive tangents across the tesselated surface (degrees)",
#     str(Path(__file__).parent.joinpath("data/angular_displacement.png")),
#     400,
#     220,
# )


def max_chord_tooltip():
    ui.Label("Maximum size of a chord (m)", style_type_name_override="Tooltip")


def chord_tolerance(model, props):
    if props.chord_tolerance != model.get_value_as_float():
        props.chord_tolerance = model.get_value_as_float()
        return True
    return False


def angle_tolerance(model, props):
    if props.angle_tolerance != radians(model.get_value_as_float()):
        props.angle_tolerance = radians(model.get_value_as_float())
        return True
    return False


def max_chord(model, props):
    if props.max_chord != model.get_value_as_float():
        props.max_chord = model.get_value_as_float()
        return True
    return False


class TesselationPropertiesModel(ui.AbstractItemModel):
    def item_changed(self, changed):
        self.changed = changed

    def __init__(self, props):
        super().__init__()
        if props:
            self.props = props
        else:
            self.props = TesselationProperties()
        self.models = [
            ui.SimpleFloatModel(self.props.chord_tolerance),
            ui.SimpleFloatModel(degrees(self.props.angle_tolerance)),
            ui.SimpleFloatModel(self.props.max_chord),
        ]
        self.vcfn = [0 for i in self.models]
        self.vcfn[0] = self.models[0].add_value_changed_fn(
            lambda m, b=self.props: self.item_changed(chord_tolerance(m, b))
        )
        self.vcfn[1] = self.models[1].add_value_changed_fn(
            lambda m, b=self.props: self.item_changed(angle_tolerance(m, b))
        )
        self.vcfn[2] = self.models[2].add_value_changed_fn(lambda m, b=self.props: self.item_changed(max_chord(m, b)))
        self.changed = False

    def __del__(self):
        for i, m in enumerate(self.models):
            m.remove_value_changed_fn(self.vcfn[i])
        self.models = None
        self.vcfn = None

    def get_value(self):
        return self.props

    def set_value(self, props):
        self.models[0].set_value(props.chord_tolerance)
        self.models[1].set_value(degrees(props.angle_tolerance))
        self.models[2].set_value(props.max_chord)
        self._item_changed(None)

    def get_value_model(self, column_id):
        if column_id < 3:
            return self.models[column_id]


class TesselationPropsDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self._highlighting_enabled = True
        self._highlighting_text = None
        self.tooltip_fns = [chord_tolerance_tooltip, angle_tolerance_tooltip, max_chord_tooltip]
        self.column_names = ["Chord Tolerance (m)", "Angle Tolerance (d)", "Max Chord (m)"]
        self.num_columns = len(self.column_names)
        self.listView = None

    def build_branch(self, model, item, column_id, level, expanded):
        pass

    def add_List_view(self, listView):
        self.listView = listView

    def build_widget(self, model, item, column_id, level, expanded):
        """Create a widget per item"""
        value_model = model.get_item_value_model(item, column_id)
        if not value_model:
            return
        if column_id < self.num_columns:
            with ui.HStack(spacing=4, height=20, alignment=(ui.Alignment.CENTER)):
                # ui.Spacer(width=3)
                ui.FloatDrag(model=value_model, min=(mins[column_id]), max=(maxs[column_id]), step=0.001)
                ui.Spacer(width=2)

    def build_header(self, column_id):
        style_type_name = "TreeView.Header"
        if column_id < self.num_columns:
            with ui.HStack(height=15):
                ui.Spacer(width=4)
                with ui.Frame(horizontal_clipping=True):
                    ui.Label(
                        (self.column_names[column_id]),
                        name="columnname",
                        style_type_name_override=style_type_name,
                        tooltip_fn=(self.tooltip_fns[column_id]),
                        tooltip_offset_y=15,
                    )


class TesselationPropertiesItem(ui.AbstractItem):
    def __init__(self, props=None):
        super().__init__()
        self.model = TesselationPropertiesModel(props)

    def get_value(self):
        return self.model.get_value()


class TesselationPropertiesItemWidget:
    def __init__(self, **kwargs):
        self.model = kwargs.get("model", TesselationPropertiesModel(TesselationProperties()))
        self.theme = kwargs.get("theme", "NvidiaDark")

        self.tooltip_fns = [chord_tolerance_tooltip, angle_tolerance_tooltip, max_chord_tooltip]
        self.column_names = ["Chord Tolerance (m)", "Angle Tolerance (d)", "Max Cord (m)"]
        # self._on_value_changed_fn = kwargs.get("value_changed_fn", None)
        self._on_end_edit_fn = kwargs.get("end_edit_fn", None)
        self._style = kwargs.get("style", UI_STYLES[self.theme])
        self._frame = ui.Frame(**kwargs)
        self._edit_popup = ui.Window(
            str(uuid.uuid1()),
            width=400,
            height=50,
            flags=ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_TITLE_BAR
            | ui.WINDOW_FLAGS_POPUP,
            visible=False,
        )
        self.model.subscribe_item_changed_fn(self.build_ui)
        self.build_ui()

    def on_end_edit(self):
        if self.model.changed:
            if self._on_end_edit_fn:
                self._on_end_edit_fn(self.model)
            self.model.changed = False

    def get_button_text(self):
        return "({:0.3f}, {:0.3f}, {:0.3f})".format(
            *[self.model.get_value_model(i).get_value_as_float() for i in range(3)]
        )

    def __del__(self):
        self._edit_popup = None
        self._frame = None

    def build_ui(self, *args):
        with self._frame:
            with ui.HStack(height=22, style={"alignment": ui.Alignment.LEFT_CENTER}):
                self._display_lbl = ui.Label(self.get_button_text())
                self._display_lbl.set_mouse_pressed_fn(self.edit_mode)
                self.button = ui.Button(name="arrow_down", height=20, width=20, style=self._style)
                self.button.set_clicked_fn(self.edit_mode)
        with self._edit_popup.frame:
            with ui.HStack(style={"alignment": ui.Alignment.CENTER}):
                for i in range(3):
                    value_model = self.model.get_value_model(i)
                    if value_model:
                        with ui.VStack():
                            ui.Label(
                                self.column_names[i],
                                name="columnname",
                                style_type_name_override="TreeView.Header",
                                tooltip_fn=(self.tooltip_fns[i]),
                                tooltip_offset_y=15,
                            )
                            ui.FloatDrag(model=value_model, min=(mins[i]), max=(maxs[i]), step=0.001)
                        ui.Spacer(width=2)
                with ui.VStack(width=25):
                    ui.Spacer(height=20)
                    self.confirm_button = ui.Button(
                        "",
                        name="accept",
                        style=self._style,
                        width=25,
                        height=25,
                        clicked_fn=lambda: self.display_mode(True),
                    )
                with ui.VStack(width=25):
                    ui.Spacer(height=20)
                    self.cancel_button = ui.Button(
                        "",
                        name="cancel",
                        style=self._style,
                        width=25,
                        height=25,
                        clicked_fn=lambda: self.display_mode(False),
                    )
        self.on_end_edit()

    def edit_mode(self, *args):
        if not args or args[2] == 0:
            self._edit_popup.position_x = self._frame.screen_position_x - 4
            self._edit_popup.position_y = self._frame.screen_position_y - 4
            # self._edit_popup.width = self._frame.computed_width + 6
            self._edit_popup.visible = True
            self._edit_popup.visible = True

    def display_mode(self, confirm, *args):
        self.button.enabled = True
        self._edit_popup.visible = False
        if confirm and self.model.changed:
            self._display_lbl.text = self.get_button_text()
            self.on_end_edit()


class TesselationPropertiesListWidget:
    def __init__(self, props=[TesselationProperties()], style={}):
        self.model = TesselationPropertiesListModel(props)
        self.delegate = TesselationPropsDelegate()
        self.style = style
        self.build_ui()

    # def __del__(self):
    #     self.model._chi

    def get_props(self):
        return self.model.get_props()

    def remove_selected_lod(self):
        self.model.remove_item(self._tesselation_properties_list.selection)
        self._tesselation_properties_list.clear_selection()

    def build_ui(self):
        with ui.CollapsableFrame("LOD properties", height=ui.Pixel(0)):
            with ui.HStack(height=ui.Percent(100)):
                with ui.VStack(width=ui.Pixel(20)):
                    ui.Spacer(height=ui.Pixel(13))
                    self._add_lod = ui.Button(
                        "+", clicked_fn=self.model.add_prop, height=ui.Pixel(20), width=ui.Pixel(20)
                    )
                    self._remove_lod = ui.Button(
                        "-",
                        clicked_fn=self.remove_selected_lod,
                        height=ui.Pixel(20),
                        width=ui.Pixel(20),
                        tooltip="Removes selected element from the list. if None is selected, removes last.",
                    )
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    # vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    # style_type_name_override="TreeView",
                    style=self.style,
                    height=ui.Pixel(90),
                    # width=ui.Percent(100)
                ):
                    self._tesselation_properties_list = ui.TreeView(
                        self.model,
                        style=self.style,
                        delegate=self.delegate,
                        header_visible=True,
                        height=ui.Pixel(88),
                        alignment=ui.Alignment.CENTER_TOP,
                    )


class TesselationPropertiesListModel(ui.AbstractItemModel):
    def __init__(self, props_list):
        super().__init__()
        self._children = [TesselationPropertiesItem(props) for props in props_list]

    def __del__(self):
        self._children = None

    def get_props(self):
        return [item.get_value() for item in self._children]

    def reset(self):
        self._children.clear()
        self._item_changed(None)

    def add_prop(self):
        self._children.append(TesselationPropertiesItem())
        self._item_changed(None)

    def remove_item(self, items):
        if len(items) > 0:
            for item in items:
                if item in self._children:
                    self._children.remove(item)

        else:
            self._children.pop()
        if len(self._children) == 0:
            self._children.append(TesselationPropertiesItem())
        self._item_changed(None)

    def get_item_children(self, item):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            return self._children

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 3

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        """
        if item:
            if isinstance(item, TesselationPropertiesItem):
                return item.model.get_value_model(column_id)
