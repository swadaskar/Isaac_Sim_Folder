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


class DynamicComboBoxItem(ui.AbstractItem):
    def __init__(self, text):
        super().__init__()
        self.model = ui.SimpleStringModel(text)


class DynamicComboBoxModel(ui.AbstractItemModel):
    def __init__(self, args):
        super().__init__()

        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self._items = []
        for i in range(len(args)):
            self._items.append(DynamicComboBoxItem(args[i]))

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
