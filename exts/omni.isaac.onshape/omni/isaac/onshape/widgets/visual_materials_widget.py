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

from functools import partial
from omni.isaac.onshape.widgets.color_name import Color, ColorName


from omni.isaac.onshape.scripts.style import UI_STYLES


class FloatItem(ui.AbstractItem):
    def __init__(self, value):
        super().__init__()
        self.model = ui.SimpleFloatModel(value)


class FloatListModel(ui.AbstractItemModel):
    def begin_edit(self, *args):
        pass

    def end_edit(self, *args):
        pass

    def add_value_changed_fn(self, fn):
        self.value_changed_fn = fn

    def __del__(self):
        self.value_changed_fn = None

    def __init__(self, values):
        super().__init__()
        self._children = [FloatItem(i) for i in values]
        self.value_changed_fn = None

    def get_item_children(self, item=None):
        """Returns all the children when the widget asks it."""
        if item is not None:
            # Since we are doing a flat list, we return the children of root only.
            # If it's not root we return.
            return []

        return self._children

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 1

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        In our case we use ui.SimpleStringModel.
        """
        return item.model

    def set_item_value(self, item_idx, value):
        self._children[item_idx].model.set_value(value)

    def get_value(self):
        return [v.model.get_value_as_float() for v in self._children]


class EditableStringFieldWidget(ui.Widget):
    def __init__(self, model):
        # super().__init__()
        self.model = model
        self._widget = ui.VStack()
        self._selected = False
        with self._widget:
            self.label = ui.Label(model.get_value_as_string(), word_wrap=True, mouse_double_clicked_fn=self.begin_edit)
            self.field = ui.StringField(self.model, visible=False)
        self._edit_subs = self.model.subscribe_end_edit_fn(self.end_edit)

    def begin_edit(self, *args):
        self.label.visible = False
        self.field.visible = True
        self.field.focus_keyboard()

    def end_edit(self, *args):
        self.label.text = self.model.get_value_as_string()
        self.label.visible = True
        self.field.visible = False

    @property
    def checked(self) -> bool:
        return self.label.checked

    @checked.setter
    def checked(self, value: bool):
        self.label.checked = value

    @property
    def selected(self) -> bool:
        return self._selected

    @selected.setter
    def selected(self, value: bool):
        if self._widget:
            self._widget.checked = value
            self.label.checked = value
            self._selected = value
        if not value:
            self.end_edit()


class VisualMaterial:
    def __init__(self, rgba):
        self.color = Color(rgba)
        self.name = ColorName.get_name(self.color)
        self.roughness = 0.5
        self.metallic = 0.0
        self.emissive = Color([0.0, 0.0, 0.0, 0.0])


class VisualMaterialItem(ui.AbstractItem):
    def __init__(self, material: VisualMaterial, **kwargs):

        self.model = VisualMaterialModel(material, _on_value_changed_fn=self._on_value_changed)

    def get_value(self):
        return self.model.get_value()

    def _on_value_changed(self):
        if self._on_value_changed_fn:
            self._on_value_changed_fn(self)


def set_color_component(model, i, value):
    if model:
        model.material.color.set_index_value(i, value)
        model._on_value_changed()


def set_color(model, material):
    if material:
        material.color.set_value(model.get_value_model(0).get_value())
        model._on_value_changed()


def set_name(model, material):
    if model:
        material.name = model.get_value_model(1).get_value_as_string()
        model._on_value_changed()


def set_roughness(model, value):
    if model and model.material.roughness != value:
        model.material.roughness = value
        model._on_value_changed()


def set_metallic(model, value):
    if model and model.material.metallic != value:
        model.material.metallic = value
        model._on_value_changed()


def set_emissive(model, material):
    if model:
        model.material.emissive.set_value(model.get_value_model(4).get_value())
        model._on_value_changed()


def set_emissive_component(model, i, value):
    if model and model.material.emissive.get_as_list()[i] != value:
        model.material.emissive.set_index_value(i, value)
        model._on_value_changed()


class VisualMaterialCard(ui.Widget):
    def __init__(self, visualmaterial, **kwargs):
        self.material = visualmaterial
        self._selected = False
        self._widget = None
        theme = kwargs.get("style", "NvidiaDark")
        self._mouse_pressed_fn = kwargs.get("mouse_pressed_fn", None)
        self._theme = kwargs.get("theme", theme)
        self._style = UI_STYLES[self._theme]
        self._label = None

    def build_card(self):
        # print(self.material, dir(self.material))

        def on_mouse_pressed(card, *args):
            if self._mouse_pressed_fn:
                self._mouse_pressed_fn(card, *args)

        def mouse_hovered_fn(hovered: bool):
            self._label.checked = hovered

        color_model = self.material.model.get_value_model(0)
        with ui.ZStack(
            width=0, height=0, alignment=(ui.Alignment.CENTER), style_type_name_override="Card", style=self._style
        ):
            self._widget = ui.Rectangle(
                mouse_pressed_fn=partial(on_mouse_pressed, self),
                style={
                    "margin_width": ui.Pixel(3),
                    "background_color": 0x00FFFFFF,
                    "border_radius": 4,
                    ":checked": {"background_color": 0x44FFFFFF},
                },
            )
            self._widget.set_mouse_hovered_fn(mouse_hovered_fn)
            with ui.VStack(
                spacing=4,
                width=65,
                height=70,
                style=self._style,
                style_type_name_override="Card",
                alignment=(ui.Alignment.CENTER),
            ):
                ui.Spacer(height=3)
                m = ui.ColorWidget(height=40).model
                for i, it in enumerate(m.get_item_children()):
                    item_model = m.get_item_value_model(it)
                    item_model.set_value(color_model.get_item_children()[i].model.get_value_as_float())
                    item_model.add_value_changed_fn(
                        lambda m, b=color_model, i=i: set_color_component(
                            self.material.model, i, m.get_value_as_float()
                        )
                    )
                self._label = EditableStringFieldWidget(self.material.model.get_value_model(1))

    @property
    def selected(self) -> bool:
        # print(self._selected)
        return self._selected

    @selected.setter
    def selected(self, value: bool):
        if self._widget:
            self._widget.checked = value
            self._label.checked = value
            self._label.selected = value
            self._selected = value


class VisualMaterialModel(ui.AbstractItemModel):
    def __init__(self, material, **kwargs):

        self._on_value_changed_fn = kwargs.get("value_changed_fn", None)
        self.material = material
        self.models = [
            FloatListModel(self.material.color.get_as_list()),
            ui.SimpleStringModel(self.material.name),
            ui.SimpleFloatModel(self.material.roughness),
            ui.SimpleFloatModel(self.material.metallic),
            FloatListModel(self.material.emissive.get_as_list()),
        ]
        self.vcfn = [
            self.models[i].add_value_changed_fn(lambda _, m=self, b=self.material: fn(m, b))
            for i, fn in enumerate([set_color, set_name, set_roughness, set_metallic, set_emissive])
            if i != 1
        ]
        self.end_edit_fn = self.models[1].subscribe_end_edit_fn(lambda _, m=self, b=self.material: set_name(m, b))

    def _on_value_changed(self):
        if self._on_value_changed_fn:
            self._on_value_changed_fn()

    def _value_changed(self):
        if self._on_value_changed_fn:
            self._on_value_changed_fn()

    def __del__(self):
        for i, m in enumerate(self.models):
            m.remove_value_changed_fn(self.vcfn[i])

    def get_value(self):
        return self.material

    def get_value_model(self, column_id):
        return self.models[column_id]


class VisualMaterialListModel(ui.AbstractItemModel):
    def __init__(self, mats_list):
        super().__init__()
        self._children = [VisualMaterialItem(mat) for mat in mats_list]

    def get_materials(self):
        return [item.get_value() for item in self._children]

    def add_material(self, mat):
        self._children.append(VisualMaterialItem(mat))

    def get_item_children(self, item=None):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            return self._children

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 1

    def get_item_value_model(self, item, column_id):
        """ 
        Return value model.
        It's the object that tracks the specific value.
        """
        if item:
            if isinstance(item, VisualMaterialItem):
                return item.model.get_value_model(column_id)


class VisualMaterialWidget:
    def __init__(self, materials_list, parent, **kwargs):
        self._cards = {}
        self._selections = []
        self.model = VisualMaterialListModel(materials_list)
        # print(self.model, self.model.get_item_children())
        self._mouse_pressed_fn = kwargs.get("mouse_pressed_fn", None)
        self._mouse_double_clicked_fn = kwargs.get("mouse_double_clicked_fn", None)
        self._selection_changed_fn = kwargs.get("selection_changed_fn", None)
        self._on_material_changed_fn = kwargs.get("material_changed_fn", None)
        self._material_changed_subscription = None
        self._card_width = 65
        self._card_height = 55
        self._style = UI_STYLES[kwargs.get("theme", "NvidiaDark")]
        self.gridView = None
        self.parent = parent
        self._widget = ui.Frame()
        self.build_ui()

    def set_on_selection_changed_fn(self, selection_changed_fn):
        self._on_selection_changed_fn = selection_changed_fn

    def set_on_material_changed_fn(self, material_changed_fn):
        self._on_material_changed_fn = material_changed_fn

    def add_material(self, material):
        self.model.add_material(material)
        with self._widget:
            self.build_ui()

    def build_ui(self):
        with self._widget:
            with ui.VStack():
                with ui.ScrollingFrame(height=250, style_type_name_override="TreeView"):
                    self._grid = ui.VGrid(
                        column_width=self._card_width,
                        row_height=self._card_height + 30,
                        # mouse_pressed_fn=partial(self._on_mouse_pressed, None),
                        mouse_double_clicked_fn=partial(self._on_mouse_double_clicked, None),
                        style_type_name_override="GridView.Grid",
                        style=self._style,
                        auto_resize=True,
                    )
                ui.Spacer(height=4)
                with ui.HStack():
                    # Metallic
                    with ui.VStack(width=50, height=75):
                        ui.Label("Roughness", height=25)
                        ui.Label("Metallic", height=25)
                        ui.Label("Emissive", height=25)
                    ui.Spacer(width=4)
                    with ui.VStack(height=75):
                        self._roughness_slider = ui.FloatDrag(min=0, max=1.0, style_type_name_override="TreeView.Edit")
                        self._roughness_slider.model.add_value_changed_fn(
                            lambda a, b=self: set_roughness(
                                b.selections[0].model if b.selections else None, a.get_value_as_float()
                            )
                        )
                        self._metallic_slider = ui.FloatDrag(min=0, max=1.0, style_type_name_override="TreeView.Edit")
                        self._metallic_slider.model.add_value_changed_fn(
                            lambda a, b=self: set_metallic(
                                b.selections[0].model if b.selections else None, a.get_value_as_float()
                            )
                        )
                        with ui.HStack():
                            self.emmissive_model = ui.ColorWidget(height=25, width=25).model
                            ui.Spacer(width=4)
                            for i, it in enumerate(self.emmissive_model.get_item_children()):
                                if i < 3:
                                    item_model = self.emmissive_model.get_item_value_model(it)
                                    item_model.add_value_changed_fn(
                                        lambda m, b=self, i=i: set_emissive_component(
                                            b.selections[0].model if b.selections else None, i, m.get_value_as_float()
                                        )
                                    )
                            self.emmissive_intensity = ui.FloatDrag(
                                min=0, max=10000.0, style_type_name_override="TreeView.Edit"
                            )
                            self.emmissive_intensity.model.add_value_changed_fn(
                                lambda a, b=self: set_emissive_component(
                                    b.selections[0].model if b.selections else None, 3, a.get_value_as_float()
                                )
                            )
        with self._grid:
            self.build_grid()

    def build_grid(self):
        self._cards = [
            VisualMaterialCard(self.model.get_item_children()[i]) for i in range(len(self.model.get_item_children()))
        ]
        # print(item.material for item in self._cards)
        with self._grid:
            for item in self._cards:
                ui.Frame(
                    build_fn=item.build_card,
                    mouse_pressed_fn=lambda x, y, b, k, i=item: self._on_mouse_pressed(i, x, y, b, k),
                )

    @property
    def selections(self) -> [VisualMaterialItem]:
        # print("selections getter")
        return [card.material for card in self._selections]

    @selections.setter
    def selections(self, selections: [VisualMaterialItem]):
        # print("selections setter")
        cards = [item.material for item in selections if item in self._cards]
        self.clear_selections()
        self.extend_selections(cards)

    @property
    def visible(self) -> bool:
        if self._widget:
            return self._widget.visible
        return False

    @visible.setter
    def visible(self, value):
        if self._cards:
            self._widget.visible = value
        else:
            self.build_grid()
        self.clear_selections()

    def _on_material_changed(self, item: VisualMaterialItem):
        if self._on_material_changed_fn:
            self._on_material_changed_fn(item)

    def _on_mouse_pressed(self, card: VisualMaterialCard, x, y, b, key_mod):
        if not self.selections or self.selections[0] != card.material:
            if b == 0:
                self.clear_selections()
                self.add_selection(card)

                # print(card.selected, self._selections)
                self._roughness_slider.model.set_value(self.selections[0].model.material.roughness)
                self._metallic_slider.model.set_value(self.selections[0].model.material.metallic)
                emissive = self.selections[0].model.material.emissive.get_as_list()
                for i, it in enumerate(self.emmissive_model.get_item_children()):
                    if i < 3:
                        item_model = self.emmissive_model.get_item_value_model(it)
                        item_model.set_value(emissive[i])
                self.emmissive_intensity.model.set_value(emissive[3])
                if self._mouse_pressed_fn:
                    self._mouse_pressed_fn(b, key_mod, card.material.item if card else None)
                if self._selection_changed_fn:
                    self._selection_changed_fn(self.selections)

    def _on_mouse_double_clicked(self, card: VisualMaterialItem, x, y, b, key_mod):
        if self._mouse_double_clicked_fn:
            self._mouse_double_clicked_fn(b, key_mod, card.item if card else None)

    def clear_selections(self):
        for selection in self._selections:
            selection.selected = False
        self._selections.clear()

    def extend_selections(self, cards: [VisualMaterialCard]):
        for card in cards:
            self.add_selection(card)

    def add_selection(self, card: VisualMaterialCard):
        if card and card not in self._selections:
            card.selected = True
            self._selections.append(card)

    def remove_selection(self, card: VisualMaterialCard):
        if card and card in self._selections:
            card.selected = False
            self._selections.remove(card)
