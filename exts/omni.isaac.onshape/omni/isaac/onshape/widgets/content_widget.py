# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.kit.commands
import omni.ui as ui
import weakref
from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.widgets.documents_widget import *
import threading
import asyncio


class SortByItem(ui.AbstractItem):
    def __init__(self, text, value):
        super().__init__()
        self.text = ui.SimpleStringModel(text)
        self.value = ui.SimpleStringModel(value)

        self.model = [self.text, self.value]


class SortByModel(ui.AbstractItemModel):
    def __init__(self):
        super().__init__()

        self._items = [
            SortByItem("Date Created", "createdAt"),
            SortByItem("Date Modified", "modifiedAt"),
            SortByItem("Name", "name"),
            SortByItem("Modified by", "modifiedBy"),
            SortByItem("Date Promoted", "promotedAt"),
        ]

        self.current_index = ui.SimpleIntModel()
        self.current_index.add_value_changed_fn(lambda a: self._item_changed(None))

    def get_item_children(self, item):
        return self._items

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self.current_index
        return item.model[column_id]

    def get_selected_value(self):
        return self._items[self.current_index.get_value_as_int()].value.get_value_as_string()


class OnshapeContentWidget:
    def __init__(self, **kwargs):

        theme = kwargs.get("theme", "NvidiaDark")
        self._filter_unsupported = kwargs.get("filter_unsupported", True)
        self._style = UI_STYLES[theme]
        self._filter_option = 0
        self.orders = ["asc", "desc"]
        self.order_icons = ["arrow_down", "arrow_up"]
        self.order = 1
        self._frame = ui.Frame(**kwargs)
        self.searchbar = None
        self._hysteresis = False
        self.build_ui()
        self._double_clicked_fn = None

    def set_on_mouse_double_clicked(self, double_clicked_fn):
        self._double_clicked_fn = double_clicked_fn

    @staticmethod
    def _set_widget_visible(widget: ui.Widget, visible):
        """Utility for using in lambdas"""
        widget.visible = visible

    def query_changed(self, q=None, b=None):
        q = self.queryField.model.get_value_as_string().replace(" ", "*")
        self.docs_list.model.list_all_docs(
            query=q,
            filter_type=self._filter_option,
            sortOrder=self.orders[self.order],
            sortColumn=self.sort_box.model.get_selected_value(),
        )
        self.scroll.scroll_y = 0
        self._hysteresis = False

    def scroll_changed(self, y):
        if y + self.scroll.computed_height >= self.docs_list.computed_height - 300:
            if self._hysteresis:
                self.docs_list.model.get_next_page()
                self._hysteresis = False
        else:
            self._hysteresis = True
        # self.docs_list.dirty_widgets()

    def _on_options_reset(self):
        item = [f for f in self._filters if f.checked == True]
        if item:
            for i in item:  # There should be only one, but loop anyway
                i.checked = False

    def _on_filter_changed(self, value, i):
        if value:
            self._filter_option = i
            for n, f in enumerate(self._filters):
                if f.checked and n != i:
                    f.checked = False
            self.query_changed()
        elif not [f for f in self._filters if f.checked == True]:
            self._on_filter_changed(True, -1)

    def _change_sort_order(self):
        self.order = not self.order
        self.btn_order_up.visible = self.order
        self.btn_order_down.visible = not self.order
        self.query_changed()

    def _on_item_double_clicked(self, item):
        if self._double_clicked_fn:
            self._double_clicked_fn(item)

    def build_ui(self):
        self._docs_model = DocumentListModel(self._filter_unsupported)
        self._docs_delegate = DocumentListDelegate(self._style)
        self._docs_delegate.set_on_mouse_double_clicked(self._on_item_double_clicked)
        self._filters = []
        with self._frame:
            self._options_menu = ui.Menu("Options")
            with self._options_menu:
                ui.MenuItem("Search Filter", enabled=False)
                ui.Separator()
                self._filters.append(ui.MenuItem("My Docs", checkable=True, checked=True))
                self._filters.append(ui.MenuItem("Created by Me", checkable=True))
                self._filters.append(ui.MenuItem("Shared with Me", checkable=True))
                self._filters.append(ui.MenuItem("Trash", checkable=True))
                self._filters.append(ui.MenuItem("Public documents", checkable=True))
                self._filters.append(ui.MenuItem("Recent", checkable=True))
                self._filters.append(ui.MenuItem("Owner", checkable=True))
                self._filters.append(ui.MenuItem("Company", checkable=True))
                self._filters.append(ui.MenuItem("Team", checkable=True))
                # ui.Separator()
                # ui.MenuItem("Reset", triggered_fn=self._on_options_reset)
                for i, f in enumerate(self._filters):
                    f.set_checked_changed_fn(lambda value, idx=i: self._on_filter_changed(value, idx))
            with ui.VStack():
                self.searchbar = ui.HStack(style=self._style, height=ui.Pixel(20))
                with self.searchbar:
                    with ui.ZStack(height=ui.Pixel(20)):
                        self.queryField = ui.StringField(height=ui.Pixel(20))
                        self.search_label = ui.Label("Search", name="search", style=self._style, height=ui.Pixel(20))
                        self._begin_filter_subscription = self.queryField.model.subscribe_begin_edit_fn(
                            lambda _: OnshapeContentWidget._set_widget_visible(self.search_label, False)
                        )
                    self._end_filter_subscription = self.queryField.model.subscribe_end_edit_fn(
                        lambda m: OnshapeContentWidget._set_widget_visible(self.search_label, not m.as_string)
                    )

                    ui.Spacer(width=8)
                    ui.Label("Sort by: ", width=0)
                    ui.Spacer(width=2)
                    self.sort_box = ui.ComboBox(SortByModel(), style=self._style, width=ui.Pixel(120))
                    self.item_changed_fn_subs = self.sort_box.model.subscribe_item_changed_fn(
                        weakref.proxy(self).query_changed
                    )
                    ui.Spacer(width=3)
                    self.btn_order_up = ui.Button(
                        # omni.kit.ui.get_custom_glyph_code("${glyphs}/arrow_up.svg"),
                        name="arrow_up",
                        tooltip="Sort by Descending",
                        width=22,
                        height=22,
                        clicked_fn=lambda: self._change_sort_order(),
                    )
                    self.btn_order_down = ui.Button(
                        # omni.kit.ui.get_custom_glyph_code("${glyphs}/arrow_down.svg"),
                        name="arrow_down",
                        tooltip="Sort by Ascending",
                        width=22,
                        height=22,
                        clicked_fn=lambda: self._change_sort_order(),
                    )
                    self.btn_order_down.visible = False
                    ui.Button(
                        # omni.kit.ui.get_custom_glyph_code("${glyphs}/filter.svg"),
                        name="filter",
                        width=22,
                        height=22,
                        clicked_fn=lambda: self._options_menu.show(),
                        style=self._style,
                    )
                    self.queryField.model.add_value_changed_fn(lambda m: self.query_changed(m.get_value_as_string()))
                # ui.Spacer(height=ui.Pixel(3))
                # with ui.HStack(style = _style, height = ui.Pixel(20)):
                #     ui.Label("User name:")
                #     ui.Label("Company name:")
                #     ui.Label("Team Name:")

                ui.Spacer(height=ui.Pixel(3))
                self.scroll = ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    # vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style_type_name_override="TreeView.ScrollingFrame",
                    style=self._style,
                    width=ui.Percent(100),
                )
                self.scroll.set_scroll_y_changed_fn(lambda y: self.scroll_changed(y))
                with self.scroll:
                    self.docs_list = ui.TreeView(
                        self._docs_model,
                        delegate=self._docs_delegate,
                        style=self._style,
                        height=ui.Percent(100),
                        column_widths=[ui.Percent(100)],
                    )
        self._frame.visible = True
