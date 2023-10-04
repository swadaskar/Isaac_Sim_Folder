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
from pxr import UsdGeom
import io
from functools import partial
from PIL import Image, ImageChops
import numpy as np
import asyncio
import threading
import time
import signal

from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.client import OnshapeClient


supported_elements = ["Assembly"]  # , "Part", "Part Studio"]


def make_image(element_type, style):
    name = {
        "Assembly": "assembly",
        "Part Studio": "part_studio",
        "Part": "part",
        "Application": "blob",
        "Bill Of Materials": "bom",
    }
    ui.Image(name=name.get(element_type, "blob"), width=20, height=20, style=style)


def make_type_tooltip(type):
    with ui.VStack():
        if type not in supported_elements:
            ui.Spacer(height=15)
        ui.Label(type, style_type_name_override="Tooltip")


class ElementItem(ui.Widget):
    def __init__(self, document, el_idx, **kwargs):
        self.document = document
        self.element_idx = el_idx
        self.element = document.get_elements()[el_idx]
        self._selected = False
        self.elid = self.element["id"]
        self.element_meta = None
        self.__thumb_img = None
        self._byte_img_provider = ui.ByteImageProvider()
        self._widget = None
        self._label = None
        self._mouse_pressed_fn = kwargs.get("mouse_pressed_fn", None)
        # self._mouse_double_clicked_fn = kwargs.get("mouse_double_clicked_fn", None)
        self._theme = "NvidiaDark"
        self._style = UI_STYLES[self._theme]
        self._mouse_double_clicked_fn = None

        # self.__get_thumb()

    def get_id(self):
        return self.element["id"]

    def get_meta(self):
        def get_meta_async():
            self.element_meta = OnshapeClient.get().metadata_api.get_element_metadata()

        self.get_meta_task = threading.Thread(target=get_meta_async)
        self.get_meta_task.start()

    def get_thumb(self):
        if self.__thumb_img is None:
            self.__get_thumb()
        return self._byte_img_provider

    def get_thumb_img(self):
        if not self.__thumb_img:
            self.__get_thumb()
        return self.__thumb_img

    @property
    def selected(self) -> bool:
        # print(self._selected)
        return self._selected

    @selected.setter
    def selected(self, value: bool):
        if self._widget:
            self._widget.checked = value
            self._rect.checked = value
            self._label.checked = value
            self._selected = value

    def build_widget(self):
        def on_mouse_pressed(card, *args):
            if self._mouse_pressed_fn:
                self._mouse_pressed_fn(card, *args)

        def on_mouse_double_clicked(card, *args):
            # print("element double clicked", self.element)
            if self._mouse_double_clicked_fn:
                self._mouse_double_clicked_fn(card, *args)

        def mouse_hovered_fn(hovered: bool):
            self._label.selected = hovered

        with ui.ZStack(width=0, height=0):
            self._widget = ui.Frame(
                tooltip="Element type Not currently supported" if self.element["type"] not in supported_elements else ""
            )
            with self._widget:
                ui.Rectangle(
                    mouse_pressed_fn=partial(on_mouse_pressed, self),
                    mouse_double_clicked_fn=partial(on_mouse_double_clicked, self)
                    if self.element["type"] in supported_elements
                    else None,
                    style={
                        "margin_width": 8,
                        "background_color": 0x00FFFFFF,
                        "border_radius": 10,
                        ":checked": {
                            "background_color": 0x44FFFFFF if self.element["type"] in supported_elements else 0x443333FF
                        },
                    },
                )
            self._widget.set_mouse_hovered_fn(mouse_hovered_fn)
            with ui.VStack(style_type_name_override="Card", style=self._style):
                with ui.ZStack(width=100):
                    self._rect = ui.Rectangle(
                        height=100,
                        width=100,
                        style={
                            "margin": 15,
                            "background_color": 0x44FFFFFF
                            if self.element["type"] in supported_elements
                            else 0x443333FF,
                            "border_color": 0xFF222222,
                            "border_width": 0.5,
                            "border_radius": 10,
                            ":checked": {
                                "background_color": 0x88000000
                                if self.element["type"] in supported_elements
                                else 0x88000033
                            },
                        },
                    )
                    ui.ImageWithProvider(
                        self.get_thumb(), height=100, width=100, style={"border_radius": 10, "margin": 18}
                    )
                    with ui.VStack(
                        width=ui.Percent(100), height=ui.Percent(100), style={"alignment": ui.Alignment.LEFT_BOTTOM}
                    ):
                        ui.Spacer(height=60)
                        with ui.HStack(tooltip_fn=lambda: make_type_tooltip(self.element["type"])):
                            ui.Spacer(width=20)
                            make_image(self.element["type"], self._style)

                with ui.HStack(height=30):
                    # Note: This Placer nudges the label closer to the image. The inherent
                    # margin would otherwise create a less pleasing separation.
                    with ui.Placer(stable_size=True, offset_x=0, offset_y=-8):
                        self._label = ui.Label(
                            self.get_name(),
                            style_type_name_override="Card.Label",
                            word_wrap=True,
                            elided_text=True if self._theme == "NvidiaDark" else False,
                        )
                ui.Spacer()
        self.selected = False

    def get_name(self):
        return self.element["name"]

    def __get_thumb(self):

        # Download largest available thumb size
        # r = OnshapeClient.get().thumbnails_api.get_document_thumbnail_with_size(self.document_id, self.get_workspace(), thumb_sizes["sizes"][-1]["size"], _preload_content=False)
        def trim(im):
            bg = Image.new(im.mode, im.size, im.getpixel((0, 0)))
            diff = ImageChops.difference(im, bg)
            diff = ImageChops.add(diff, diff, 2.0, -100)
            bbox = diff.getbbox()
            if bbox:
                return im.crop(bbox)
            return im

        def get_thumb():

            try:
                thumb_sizes = OnshapeClient.get().thumbnails_api.get_element_thumbnail(
                    self.document.document_id, self.document.get_wdid(), self.document.get_workspace(), self.elid
                )
                sizes = [
                    int("".join(filter(str.isdigit, thumb_sizes["sizes"][i]["size"])))
                    for i in range(len(thumb_sizes["sizes"]))
                ]
                idx = sorted(range(len(sizes)), key=lambda k: sizes[k])
                r = OnshapeClient.get().thumbnails_api.get_element_thumbnail_with_size(
                    self.document.document_id,
                    self.document.get_default_workspace(),
                    self.elid,
                    thumb_sizes["sizes"][idx[-1]]["size"],
                    _preload_content=False,
                )
                stream = io.BytesIO(r.data)
                pil_img = trim(Image.open(stream))
                size = pil_img.size
                scale = 100.0 / size[1]
                self.__thumb_img = np.array(
                    pil_img.resize((int(size[0] * scale), int(size[1] * scale)), resample=Image.LANCZOS)
                )
                self._byte_img_provider.set_bytes_data(
                    self.__thumb_img.flatten().tolist(), [self.__thumb_img.shape[1], self.__thumb_img.shape[0]]
                )
            except:
                pass

        task = threading.Thread(target=get_thumb)
        task.start()
        # print(self._byte_img_provider)


class ElementGridView:
    def __init__(self, theme: str, parent, **kwargs):
        self._cards = {}
        self._selections = []
        self._style = UI_STYLES[theme]
        self._mouse_pressed_fn = kwargs.get("mouse_pressed_fn", None)
        self._mouse_double_clicked_fn = kwargs.get("mouse_double_clicked_fn", None)
        self._selection_changed_fn = kwargs.get("selection_changed_fn", None)
        self._card_width = 105
        self._card_height = 105
        self._parent = parent
        self._scale = 1.0

        self._widget = ui.VGrid(
            column_width=self._scale * self._card_width,
            row_height=self._scale * self._card_height + 30,
            # mouse_pressed_fn=partial(self._on_mouse_pressed, None),
            mouse_double_clicked_fn=partial(self._on_mouse_double_clicked, None),
            style_type_name_override="GridView.Grid",
            style=self._style,
            auto_resize=True,
        )
        with self._widget:
            self.build_grid()
        self._widget.visible = False

    @property
    def selections(self) -> [ElementItem]:
        # print("selections getter")
        return [card.element for card in self._selections]

    @selections.setter
    def selections(self, selections: [ElementItem]):
        # print("selections setter")
        cards = [self._cards[item] for item in selections if item in self._cards]
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
            for card in self._cards.values():
                if card:
                    card.selected = False
        else:
            self.build_grid()
        self.clear_selections()

    def build_grid(self):
        self._widget.clear()
        with self._widget:
            self._cards.clear()
            self._cards = {
                i: ElementItem(
                    self._parent,
                    i,
                    mouse_pressed_fn=self._on_mouse_pressed,
                    mouse_double_clicked_fn=self._on_mouse_double_clicked,
                )
                for i in range(len(self._parent.get_elements()))
            }
            for item in self._cards.values():
                ui.Frame(build_fn=lambda i=item: self.build_widget(i))

        self._widget.visible = True

    def build_widget(self, item: ElementItem):
        """Create a widget per item"""
        if not item:
            return
        item.build_widget()

    def _on_mouse_pressed(self, card: ElementItem, x, y, b, key_mod):
        # print("mouse pressed")
        if b == 0:
            self.clear_selections()
            self.add_selection(card)

        if self._mouse_pressed_fn:
            self._mouse_pressed_fn(b, key_mod, card.item if card else None)

        if self._selection_changed_fn:
            self._selection_changed_fn(self.selections)
        # print("end mouse pressed")

    def _on_mouse_double_clicked(self, card: ElementItem, x, y, b, key_mod):
        # print("mouse double clicked")
        if self._mouse_double_clicked_fn:
            self._mouse_double_clicked_fn(b, key_mod, card.item if card else None)
        # print("end double clicked")

    def clear_selections(self):
        for selection in self._selections:
            selection.selected = False
        self._selections.clear()

    def extend_selections(self, cards: [ElementItem]):
        for card in cards:
            self.add_selection(card)

    def add_selection(self, card: ElementItem):
        if card and card not in self._selections:
            card.selected = True
            self._selections.append(card)

    def remove_selection(self, card: ElementItem):
        if card and card in self._selections:
            card.selected = False
            self._selections.remove(card)
