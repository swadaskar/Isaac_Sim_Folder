# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import omni
import omni.ui as ui

from typing import Callable, Optional

from omni.isaac.ui.ui_utils import LABEL_WIDTH, BUTTON_WIDTH, get_style


class ListItem(ui.AbstractItem):
    def __init__(self, label="", type="", id=-1, min=0, max=1, default_val=0, on_value_changed_fn=None, tooltip=""):
        super().__init__()
        self.label = label
        self.type = type
        self.model = None
        self.id = id
        self.min = min
        self.max = max
        self.default_val = default_val
        self.on_value_changed_fn = on_value_changed_fn
        self.tooltip = tooltip

    def __repr__(self):
        return f'"{self.label}"'

    def name(self):
        return self.label


class ListItemModel(ui.AbstractItemModel):
    """
    Represents the model for lists. It's very easy to initialize it
    with any string list:
        string_list = ["Hello", "World"]
        model = ListModel(*string_list)
        ui.TreeView(model)
    """

    def __init__(self, *args):
        super().__init__()
        self._children = []
        for i in range(len(args)):
            kwargs = args[i]
            self._children.append(ListItem(**kwargs))

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

    def get_item_value(self, item, column_id=0):
        """
        Return value model.
        It's the object that tracks the specific value.
        In our case we use ui.SimpleStringModel.
        """
        return item.model.get_value_as_float()

    def set_item_value(self, id, value):
        if self._children[id].model is not None:
            self._children[id].model.set_value(value)
        else:
            # Shouldn't get here
            pass


class ListItemDelegate(ui.AbstractItemDelegate):
    """
    Delegate is the representation layer. TreeView calls the methods
    of the delegate to create custom widgets for each item.
    """

    def __init__(self, on_double_click_fn=None):
        super().__init__()
        self._on_double_click_fn = on_double_click_fn

    def build_branch(self, model, item, column_id, level, expanded):
        """Create a branch widget that opens or closes subtree"""
        pass

    def build_widget(self, model, item, column_id, level, expanded):
        """Create a widget per column per item"""
        stack = ui.ZStack(height=20, style=get_style())
        with stack:
            with ui.VStack(spacing=5, height=0):
                with ui.HStack():
                    label = ui.Label(item.label, width=LABEL_WIDTH, tooltip=item.tooltip)
                    item.model = ui.FloatDrag(
                        name="Field", width=BUTTON_WIDTH / 2, alignment=ui.Alignment.LEFT_CENTER
                    ).model
                    item.model.set_value(item.default_val)
                    ui.Spacer(width=5)
                    slider = ui.FloatSlider(
                        width=ui.Fraction(1),
                        alignment=ui.Alignment.LEFT_CENTER,
                        min=item.min,
                        max=item.max,
                        default_val=item.default_val,
                        step=0.001,
                        model=item.model,
                    )
                    ui.Spacer(width=5)
                ui.Spacer()

        if not self._on_double_click_fn:
            self._on_double_click_fn = self.on_double_click

        # Add Callback Functions
        stack.set_mouse_double_clicked_fn(lambda x, y, b, m, l=label: self._on_double_click_fn(b, m, l))
        item.model.add_value_changed_fn(lambda m, n=item.type, i=item.id: item.on_value_changed_fn(n, m, i))

    def on_double_click(self, button, model, label):
        """Called when the user double-clicked the item in TreeView"""
        if button != 0:
            return


class ComboBoxItem(ui.AbstractItem):
    def __init__(self, text):
        super().__init__()
        self.model = ui.SimpleStringModel(text)


class ComboBoxModel(ui.AbstractItemModel):
    def __init__(self, args):
        super().__init__()

        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self._items = []
        for i in range(len(args)):
            # kwargs = args[i]
            self._items.append(ComboBoxItem(args[i]))

    def get_item_children(self, item):
        return self._items

    def get_item_value_model(self, item: ui.AbstractItem = None, column_id: int = 0):
        if item is None:
            return self._current_index
        return item.model

    def set_item_value_model(self, item: ui.AbstractItem = None, column_id: int = 0):
        self._current_index = item
        self._item_changed(None)
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
