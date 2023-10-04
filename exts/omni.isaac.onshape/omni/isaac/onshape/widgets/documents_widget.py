# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from urllib.parse import urlparse
import carb, omni.ext, omni.kit.commands, omni.ui as ui, os, asyncio
from enum import Enum
from pxr import UsdGeom
import io
from PIL import Image, ImageChops
import numpy as np
import asyncio
import threading
import time
import signal

from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.client import OnshapeClient
from omni.isaac.onshape.widgets.elements_widget import ElementGridView, supported_elements


def _list_all_docs(doc_w, rr):

    # print(self.filter)
    query = doc_w.query
    rr.wait()
    if rr.successful():
        r = rr.get()
        with doc_w.lock:
            # If query text didn't change since the documents were fetched, otherwise drop update as there's another one oncoming
            if query == doc_w.query:
                if r.next:
                    doc_w.current_offset = doc_w._step
                    doc_w.next = True
                else:
                    doc_w.next = False
                doc_w._children = [
                    DocumentItem(doc["id"], filter_unsupported=doc_w._filter_unsupported) for doc in r["items"]
                ]
                doc_w._item_changed(None)


class DocumentItem(ui.AbstractItem):
    def __init__(self, document_id, workspace_id="", version="", element="", filter_unsupported=False):
        super().__init__()
        self.document_id = document_id
        self.workspace = workspace_id
        self.version = version
        self._filtered_elements = []
        self._element = element
        self.document = None
        self.__thumb_img = None
        self.populate_document()
        self.populate_doc_type()
        self._byte_img_provider = ui.ByteImageProvider()
        self.__get_thumb()
        self.document_type = None
        self.elements = []
        self._children = []
        self._selected = False
        self._element_grid_view = None
        self._filter_unsupported = filter_unsupported

    def populate_document(self):
        def get_doc():
            self.document = OnshapeClient.get().documents_api.get_document(self.document_id)

        self._doc_task = threading.Thread(target=get_doc)
        self._doc_task.start()

    def get_document(self):
        self._doc_task.join()
        return self.document

    def on_click(self):
        if self._widget:
            if self._element_grid_view:
                # print("selected")
                for card in self._element_grid_view._cards:
                    card.selected = False

    def populate_doc_type(self):
        def get_doc_type():
            self.elements = [
                i
                for i in OnshapeClient.get().documents_api.get_elements_in_document(
                    self.document_id, self.get_wdid(), self.get_workspace()
                )
            ]

        self._doc_type_task = threading.Thread(target=get_doc_type)
        self._doc_type_task.start()

    def update_doc_type(self):
        if len(self.get_elements()) == 1:
            # if self.elements[0]["type"] == "Part Studio":
            #     self.parts = OnshapeClient.get().parts_api.get_parts_wmve(self.document_id,'w',self.get_workspace(), self.elements[0]['id'])
            #     if len(self.parts) == 1:
            #         self.document_type = "Part"
            #     else:
            #         self.document_type = "Part Studio"
            # else:
            self.document_type = self.get_elements()[0]["type"]
        else:
            self.document_type = "Document"

    def get_document_type(self):
        self._doc_type_task.join()
        self.update_doc_type()
        return self.document_type

    def get_elements(self):
        self._doc_type_task.join()
        if not self._filtered_elements:
            if self._element:
                self._filtered_elements = [i for i in self.elements if i["id"] == self._element]
            else:
                self._filtered_elements = [
                    i for i in self.elements if not self._filter_unsupported or i["type"] in supported_elements
                ]
        return self._filtered_elements

    def update_elements_visibility(self):
        if self._element_grid_view:
            self._element_grid_view.build_grid()

    def get_workspace(self):
        if self.workspace:
            return self.workspace
        elif self.version:
            return self.version
        else:
            return self.get_document()["default_workspace"]["id"]

    def get_default_workspace(self):
        return self.get_document()["default_workspace"]["id"]

    def get_wdid(self):
        if self.version:
            return "v"
        return "w"

    def get_name(self):
        return self.get_document()["name"]

    def get_thumb(self):
        # self.img_task.join()
        return self._byte_img_provider

    def get_thumb_img(self):
        return self.__thumb_img

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

        def get_thumb(req):
            req.wait()
            if req.successful():
                r = req.get()
                stream = io.BytesIO(r.data)
                pil_img = trim(Image.open(stream))
                size = pil_img.size
                scale = 100.0 / size[1]
                self.__thumb_img = np.array(
                    pil_img.resize((int(size[0] * scale), int(size[1] * scale)), resample=Image.LANCZOS)
                )
                # print(size,self.__thumb_img.shape)
                self._byte_img_provider.set_bytes_data(
                    self.__thumb_img.flatten().tolist(), [self.__thumb_img.shape[1], self.__thumb_img.shape[0]]
                )

        def get_thumb_size(req):
            try:
                if len(self.get_elements()) == 1:
                    thumb_sizes = OnshapeClient.get().thumbnails_api.get_element_thumbnail(
                        self.document_id, self.get_wdid(), self.get_workspace(), self.get_elements()[0]["id"]
                    )
                    sizes = [
                        int("".join(filter(str.isdigit, thumb_sizes["sizes"][i]["size"])))
                        for i in range(len(thumb_sizes["sizes"]))
                    ]
                    idx = sorted(range(len(sizes)), key=lambda k: sizes[k])
                    r = OnshapeClient.get().thumbnails_api.get_element_thumbnail_with_size(
                        self.document_id,
                        self.get_default_workspace(),
                        self.get_elements()[0]["id"],
                        thumb_sizes["sizes"][idx[-1]]["size"],
                        _preload_content=False,
                        async_req=True,
                    )
                else:

                    thumb_sizes = OnshapeClient.get().thumbnails_api.get_document_thumbnail(
                        self.document_id, self.get_workspace()
                    )
                    sizes = [
                        int("".join(filter(str.isdigit, thumb_sizes["sizes"][i]["size"])))
                        for i in range(len(thumb_sizes["sizes"]))
                    ]
                    idx = sorted(range(len(sizes)), key=lambda k: sizes[k])
                    # print(idx)
                    r = OnshapeClient.get().thumbnails_api.get_document_thumbnail_with_size(
                        self.document_id,
                        self.get_workspace(),
                        thumb_sizes["sizes"][idx[-1]]["size"],
                        _preload_content=False,
                        async_req=True,
                    )
                self.thumb_task = threading.Thread(target=get_thumb, args=[r])
                self.thumb_task.start()

            except:
                pass

        self.img_task_size = threading.Thread(target=get_thumb_size, args=[None])
        self.img_task_size.start()
        # print(self._byte_img_provider)

    def toggle_elements_visible(self):
        if self._element_grid_view:
            self._element_grid_view.visible = not self._element_grid_view.visible
            self._element_frame.visible = self._element_grid_view.visible
            return self._element_grid_view.visible
        return False

    def clicked(self):
        pass
        # if self._element_grid_view:
        #     for card in self._element_grid_view._cards.values():
        #         card.selected = False

    def get_selected_element(self):
        if len(self.get_elements()) == 1:
            return self.get_elements()
        else:
            return self._element_grid_view.selections

    def build_element_grid_view(self, double_clicked_fn=None):
        self._element_frame = ui.VStack(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            auto_resize=True,
        )
        with self._element_frame:
            with ui.VStack():
                with ui.ZStack():
                    ui.Rectangle(style={"background_color": 0x88000000, "margin": 8, "border_radius": 10})
                    with ui.VStack():
                        ui.Spacer(height=10)
                        self._element_grid_view = ElementGridView(
                            "NvidiaDark", self, mouse_double_clicked_fn=double_clicked_fn
                        )
                        ui.Spacer(height=5)
                ui.Spacer(height=2)
        self._element_frame.visible = False
        self._element_grid_view.visible = False


class DocumentListModel(ui.AbstractItemModel):
    def __init__(self, filter_unsupported):
        super().__init__()
        # self._children = []
        self.current_offset = 0
        self._step = 20
        self.query = ""
        self.filter = -1
        self.ownerType = 1
        self.owner_q = ""
        self.sortColumn = "createdAt"
        self.sortOrder = "desc"
        self.current_offset = 0
        self._children = []
        self.lock = threading.Semaphore(1)
        self._element_grid_view = None
        self._filter_unsupported = filter_unsupported
        self.url_search = False
        self.list_all_docs()
        self.next = True
        # self._item_changed(None)

    def on_update_filter_unsupported(self, value):
        self._filter_unsupported = value
        for i in range(len(self._children)):
            self._children[i]._filter_unsupported = value
            self._children[i].update_elements_visibility()
            self._item_changed(self._children[i])

    def list_all_docs(self, query="", filter_type=-1, owner="", ownerType=1, sortColumn="createdAt", sortOrder="desc"):
        if query:
            self.query = query + "*"
        else:
            self.query = ""
        self.url_search = query.startswith("https")

        if self.url_search:
            url = urlparse(query)
            components = url.path[1:].split("/")
            document = ""
            workspace = ""
            version = ""
            element = ""
            for i, c in enumerate(components):
                if c == "documents":
                    document = components[i + 1]
                if c == "w":
                    workspace = components[i + 1]
                if c == "v":
                    version = components[i + 1]
                if c == "e":
                    element = components[i + 1]

            doc = OnshapeClient.get().documents_api.get_document(did=document)
            if doc:
                self._children = [
                    DocumentItem(
                        doc["id"],
                        workspace_id=workspace,
                        version=version,
                        element=element,
                        filter_unsupported=self._filter_unsupported,
                    )
                ]
                self._item_changed(None)

        else:
            self.filter = filter_type
            self.ownerType = ownerType
            self.sortColumn = sortColumn
            self.sortOrder = sortOrder
            self.current_offset = 0

            if (
                self.filter >= 0
            ):  # for some reason adding the filter option when none is selected makes it block to local docs only.
                request = OnshapeClient.get().documents_api.get_documents(
                    limit=self._step,
                    offset=0,
                    q=self.query,
                    filter=self.filter,
                    owner_type=self.ownerType,
                    sort_column=self.sortColumn,
                    sort_order=self.sortOrder,
                    async_req=True,
                )
            else:
                request = OnshapeClient.get().documents_api.get_documents(
                    limit=self._step,
                    offset=0,
                    q=self.query,
                    owner_type=self.ownerType,
                    sort_column=self.sortColumn,
                    sort_order=self.sortOrder,
                    async_req=True,
                )
            self.task = threading.Thread(target=_list_all_docs, args=[self, request])
            self.task.start()

    def get_next_page(self):
        def get_next():
            with self.lock:
                if self.next:
                    if self.filter >= 0:
                        r = OnshapeClient.get().documents_api.get_documents(
                            limit=self._step,
                            offset=self.current_offset,
                            q=self.query,
                            filter=self.filter,
                            owner_type=self.ownerType,
                            sort_column=self.sortColumn,
                            sort_order=self.sortOrder,
                        )
                    else:
                        r = OnshapeClient.get().documents_api.get_documents(
                            limit=self._step,
                            offset=self.current_offset,
                            q=self.query,
                            owner_type=self.ownerType,
                            sort_column=self.sortColumn,
                            sort_order=self.sortOrder,
                        )

                    if r.next:
                        self.current_offset += self._step
                    else:
                        self.next = False
                    self._children = self._children + [DocumentItem(doc["id"]) for doc in r["items"]]
                    self._item_changed(None)

        task = threading.Thread(target=get_next)
        task.start()

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 1

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        """
        if item and isinstance(item, DocumentItem):
            if column_id == 1:
                return item.get_name()
            elif column_id == 0:
                return item.get_thumb()

    def get_item_children(self, item):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            return self._children


class DocumentListDelegate(ui.AbstractItemDelegate):
    def __init__(self, style):
        super().__init__()
        self.num_columns = 1
        self._style = style
        self._on_mouse_double_clicked = None

    def set_on_mouse_double_clicked(self, function):
        self._on_mouse_double_clicked = function

    def on_mouse_double_clicked(self, item):
        if self._on_mouse_double_clicked:
            self._on_mouse_double_clicked(item)

    def build_branch(self, model, item, column_id, level, expanded):
        pass

    def add_List_view(self, listView):
        self.listView = listView

    def build_widget(self, model, item, column_id, level, expanded):
        if item:
            type = item.get_document_type()
            if len(item.get_elements()) < 1:
                return
            with ui.VStack():

                ui.Spacer(height=6)
                with ui.ZStack(height=ui.Pixel(20)):
                    with ui.HStack(width=ui.Percent(100)):
                        # else:
                        ui.Rectangle(
                            height=20,
                            style_type_name_override="Button",
                            style={
                                "Button": {
                                    "background_color": 0xFF444444,
                                    "border_radius": 3,
                                    "margin_width": 3,
                                    "margin_height": 0,
                                },
                                "Button:hovered": {"background_color": 0xFF666666},
                                "Button:pressed": {"background_color": 0xFF888888},
                            },
                        )
                    with ui.HStack():
                        if len(item.get_elements()) > 1:
                            ui.Spacer(width=26)
                        else:
                            ui.Spacer(width=9)
                        ui.Label(
                            item.get_name(),
                            style={
                                "aligmnent": ui.Alignment.LEFT_CENTER,
                                "margin": ui.Pixel(3),
                                "color": 0xFFDDDDDD if len(item.get_elements()) > 1 else 0xFF777777,
                            },
                            height=20,
                        )
                    if len(item.get_elements()) > 1:
                        with ui.HStack(style={"alignment": ui.Alignment.LEFT_CENTER}):

                            def toggle(button, button2, item):
                                value = item.toggle_elements_visible()
                                button2.visible = value
                                button.visible = not value

                            stl = {
                                "Button": {"background_color": 0x0, "margin": 0},
                                "Button.Image": {"margin": 2.5, "alignment": ui.Alignment.LEFT_CENTER},
                                "Button:hovered": {"background_color": 0x0},
                                "Button:pressed": {"background_color": 0x0},
                            }
                            ui.Spacer(width=3)
                            down = ui.Button(name="arrow_right", style=stl, width=ui.Fraction(1), height=ui.Pixel(20))
                            up = ui.Button(
                                name="arrow_down", visible=False, style=stl, width=ui.Fraction(1), height=ui.Pixel(20)
                            )
                            down.set_clicked_fn(lambda a=down, b=up, c=item: toggle(a, b, c))
                            up.set_clicked_fn(lambda a=down, b=up, c=item: toggle(a, b, c))
                            up.visible = False
                        ui.Spacer(width=6)
                ui.Spacer(height=3)
                with ui.HStack(
                    height=0,
                    width=ui.Percent(100),
                    style=self._style,
                    style_type_name_override="TreeView",
                    mouse_double_clicked_fn=(lambda x, y, b, _: self.on_mouse_double_clicked(item)),
                    mouse_pressed_fn=(lambda x, y, b, _, i=item: item.clicked()),
                    auto_resize=True,
                ):
                    with ui.ZStack(width=80):
                        ui.Rectangle(
                            height=85,
                            width=85,
                            style={
                                "margin": 5,
                                "background_color": 0xFF444444,
                                "border_color": 0xFF222222,
                                "border_width": 0.5,
                                "border_radius": 10,
                            },
                        )
                        ui.ImageWithProvider(
                            item.get_thumb(), height=85, width=85, style={"border_radius": 10, "margin": 10}
                        )

                        # ui.Spacer()
                        # ui.Spacer(height=3)
                    # ui.Spacer(width=1)
                    with ui.ZStack():
                        ui.Rectangle(
                            style={
                                "background_color": 0xFF333333,
                                # "border_color": 0xFF000000,
                                # "border_width": 0.5,
                                "border_radius": 5,
                                "margin": 5,
                            }
                        )
                        with ui.VStack(style={"margin_height": 13, "margin_width": 8}):
                            # ui.Spacer(height=13)
                            with ui.HStack(
                                height=0, style={"margin_height": 0, "margin_width": 0}
                            ):  # style_type_name_override="TreeView"):
                                ui.Spacer(width=8)
                                ui.Label("Date created (modified):", width=ui.Percent(0), style={"color": 0xFF999999})
                                ui.Spacer(width=3)
                                ui.Label(
                                    item.get_document()["default_workspace"]["created_at"].strftime("%Y-%m-%d %H:%M"),
                                    style={"color": 0xFF777777},
                                    width=ui.Percent(0),
                                )
                                ui.Spacer(width=3)
                                ui.Label(
                                    "({})".format(
                                        item.get_document()["default_workspace"]["modified_at"].strftime(
                                            "%Y-%m-%d %H:%M"
                                        )
                                    ),
                                    style={"color": 0xFF777777},
                                    width=ui.Percent(0),
                                )
                            with ui.HStack(height=0, style={"margin_height": 0, "margin_width": 0}):
                                ui.Spacer(width=8)
                                ui.Label("Author:", width=ui.Percent(0), style={"color": 0xFF999999})
                                ui.Spacer(width=3)
                                ui.Label(
                                    item.get_document()["default_workspace"]["creator"]["name"],
                                    style={"color": 0xFF777777},
                                )
                            with ui.HStack(height=0, style={"margin_height": 0, "margin_width": 0}):
                                ui.Spacer(width=8)
                                ui.Label("Owner:", width=ui.Percent(0), style={"color": 0xFF999999})
                                ui.Spacer(width=3)
                                ui.Label(item.get_document()["owner"]["name"], style={"color": 0xFF777777})

                            with ui.HStack(height=0, style={"margin_height": 0, "margin_width": 0}):
                                with ui.ZStack():
                                    with ui.HStack():
                                        ui.Spacer(width=8)
                                        ui.Label("Type:", width=ui.Percent(0), style={"color": 0xFF999999})
                                        ui.Spacer(width=3)
                                        ui.Label(item.get_document_type(), style={"color": 0xFF777777}, width=0)

                if len(item.get_elements()) > 1:
                    item.build_element_grid_view(lambda x, y, b, item=item: self.on_mouse_double_clicked(item))
