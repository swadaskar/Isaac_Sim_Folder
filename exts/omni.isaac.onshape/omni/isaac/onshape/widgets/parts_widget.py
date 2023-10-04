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
import weakref

import base64

from urllib.parse import quote_plus

from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.client import OnshapeClient
from omni.isaac.onshape.widgets.tesselation_properties_widget import *
from omni.isaac.onshape.widgets.physics_materials_widget import *
from omni.isaac.onshape.mesh import Mesh
from omni.isaac.onshape import get_import_physics
from concurrent.futures import ThreadPoolExecutor


class MassProperties:
    def __init__(self, com, inertia, mass, density):
        self.com = com
        self.inertia = inertia
        self.unit_inertia = inertia / mass
        self.mass = mass
        self.density = density

    def get_parallel_inertia(self, transform):
        """
        Transform should already account for global transform of center of mass
        """
        # Todo, sort out the Transform matrix type and adapt code below

        translate = transform.p
        rotate = transform.r

        return rotate * (self.inertia + self.mass + norm(translate))


class OnshapePart(ui.AbstractItem):
    def __init__(self, part, key, parent_model, mesh_thread_pool, mass_thread_pool, **kwargs):
        super().__init__()
        self.loop = asyncio.get_event_loop()
        self._task_physical_material = None
        self._task_mass_props = None
        self._task_mesh = None
        self.part = part
        self.key = key
        self.name_lock = threading.Lock()
        self._metadata = None
        self._physical_material = None
        self.mass_properties = None
        self._physical_props_changed = False
        tess_props = TesselationProperties()
        self.tess_props_changed = False
        self.parent_model = parent_model
        self.mesh_thread_pool = mesh_thread_pool
        self.mass_thread_pool = mass_thread_pool
        self.parts_usds = set()
        self._on_mesh_imported_fn = kwargs.get("mesh_imported_fn", None)
        self._on_mass_props_changed = None
        self._error_msgs = []
        self._warn_msgs = []

        self.material_lib = kwargs.get("mat_lib", OnshapeClient.get_default_material_libraries())
        self.modelCols = [
            ui.SimpleBoolModel(),
            ui.SimpleStringModel(),  # Name
            None,  # ui.SimpleStringModel(),  # Material Properties
            ui.SimpleStringModel(),  # Mass Properties
            TesselationPropertiesModel(tess_props),
        ]

        self.stage = None
        self._mesh = None
        self._name = None
        self.get_name_async()
        self._children = []

        self.start_workers()

    def make_error_tooltip(self):
        out = ""
        for i, error in enumerate(self._error_msgs):
            out += "[{}] Error: {}\n".format(i, error)
        for i, error in enumerate(self._warn_msgs):
            out += "[{}] Warning: {}\n".format(i, error)
        return out

    def set_on_mass_props_changed(self, fn):
        self._on_mass_props_changed = fn

    def get_key(self):
        return self.get_item("key")

    def add_usd_path(self, path):
        self.parts_usds.add(path)
        # print(self.parts_usds)

    def remove_usd_path(self, path):
        if path in self.parts_usds:
            self.parts_usds.remove(path)

    def start_workers(self):
        # Start Workers
        # self._get_physical_material_properties()
        self._get_mesh()
        # self._get_mass_properties()

    @property
    def downloading(self):
        return (
            (self._task_physical_material and not self._task_physical_material.done())
            or (self._task_mass_props and not self._task_mass_props.done())
            or (self._task_mesh and not self._task_mesh.done())
        )

    def set_on_mesh_imported_fn(self, fn):
        self._on_mesh_imported_fn = fn

    def on_mesh_imported(self):
        if self._on_mesh_imported_fn:
            self._on_mesh_imported_fn(self)
        self.parent_model._item_changed(self)

    def _on_mesh_imported(self, task):
        if self._mesh:
            self.on_mesh_imported()
        else:
            carb.log_error("error importing mesh {}({})".format(self.get_name(), self.get_encoded_part_id()))
        self.parent_model._item_changed(self)

    def _on_get_physical_material_props(self, task):
        # print("received materials", self.get_name())
        self.parent_model._item_changed(self)

    def _on_get_mass_props(self, task):
        if self.mass_properties:
            if self._on_mass_props_changed:
                self._on_mass_props_changed(self.mass_properties)

        self.parent_model._item_changed(self)

    def has_item(self, key):
        return self.part.has_item(key)

    def get_item(self, key):
        return self.part.get_item(key)

    def get_encoded_part_id(self):
        return quote_plus(self.get_item("partId"))

    def set_item(self, key, value):
        self.part.set_item(key, value)

    def _get_metadata(self):
        m = None
        # print("getting metadata {}".format(self._name["value"]))
        if get_import_physics():
            req = OnshapeClient.get().metadata_api.get_wmvep_metadata(
                self.get_item("documentId"),
                "w",
                self.get_item("workspaceId"),
                self.get_item("elementId"),
                self.get_encoded_part_id(),
                link_document_id=self.get_item("documentId"),
                _preload_content=False,
                async_req=True,
            )
            req.wait()
            if req.successful():
                m = req.get()

        if m:
            self._metadata = json.loads(m.data)

        self.get_name_sync()
        self._physical_material = [a for a in self._metadata["properties"] if a["name"] == "Material"][0]
        if not self.modelCols[2]:
            self.modelCols[2] = PhysicsMaterialsModel(self.material_lib, self._physical_material["value"])
        self._on_get_physical_material_props(None)

    def _get_physical_material_properties(self):
        # print("get metadata {}".format(self._name["value"]))

        # if get_import_physics():
        self._task_physical_material = self.mass_thread_pool.submit(
            self._get_metadata
        )  # threading.Thread(target=_get_metadata)
        self._task_physical_material.add_done_callback(self._on_get_physical_material_props)
        # self._task_physical_material.start()

    def get_physical_material(self):
        if not self._task_physical_material:
            self._get_physical_material_properties()
        elif self._task_physical_material.done():
            # else:
            #     self._task_physical_material.result()
            if self._physical_material:
                return self._physical_material["value"]

    def get_physical_material_sync(self):
        if not self._task_physical_material:
            self._get_physical_material_properties()
        self._task_physical_material.result()
        if self._physical_material:
            return self._physical_material["value"]

    def get_physical_material_name(self):
        if self.modelCols[2]:
            if self.modelCols[2].selection:
                return self.modelCols[2].selection.get_name()
        return ""

    def get_mass(self):
        if self.mass_properties and "mass" in self.mass_properties:
            if len(self.mass_properties["mass"]) > 0:
                return self.mass_properties["mass"][0]
        return 0

    def is_similar(self, item):
        vol_dif = 1.0
        if "volume" in self.get_mass_properties():
            volume = self.get_mass_properties()["volume"][0]
            if "volume" in item.get_mass_properties():
                v2 = item.get_mass_properties()["volume"][0]
                vol_dif = abs((v2 - volume)) / max(v2, volume)
        extent_dif = np.linalg.norm(self.get_mesh().get_extent() - item.get_mesh().get_extent()) / np.linalg.norm(
            self.get_mesh().get_extent()
        )
        return vol_dif * 0.9 + extent_dif * 0.1 < 0.01

    def _set_physical_material_properties(self, physics_material):
        def set_material():
            # print("set material props")
            try:
                body = self._metadata
                body["properties"] = [self._physical_material]
                mat = physics_material.get_metadata_dict()
                body["properties"][0]["value"] = mat
                body = str(json.dumps(body))
                r = OnshapeClient.update_metadata(
                    self.get_item("documentId"),
                    "w",
                    self.part.get_item("workspaceId"),
                    self.get_item("elementId"),
                    self.get_encoded_part_id(),
                    body,
                )
                self._physical_props_changed = True
                self._get_mass_properties()
            except Exception as e:
                self._error_msgs.append("Error setting material: {}".format((str(e))))
                carb.log_warn(str(e))

        self._task_physical_material = self.mass_thread_pool.submit(set_material)
        self._task_physical_material.add_done_callback(self._on_get_physical_material_props)
        self.parent_model._item_changed(self)
        # task = threading.Thread(target=set_material)
        # task.start()

    def _get_mass_properties(self):
        def get_mass_properties(req):
            req.wait()
            if not req.successful():
                req = OnshapeClient.get().parts_api.get_mass_properties(
                    self.get_item("documentId"),
                    "m",
                    self.get_item("documentMicroversion"),
                    self.get_item("elementId"),
                    self.get_encoded_part_id(),
                    _preload_content=False,
                    link_document_id=self.get_item("documentId"),
                    async_req=True,
                )
                req.wait()
            if req.successful():
                r = req.get()
                self.mass_properties = json.loads(r.data)
                # print(self.mass_properties)
                if self.mass_properties["bodies"]:
                    self.mass_properties = self.mass_properties["bodies"][self.get_encoded_part_id()]
                # else:
                # print(self.get_name(), self.mass_properties)
                self.mass_changed = True

            else:
                self._warn_msgs.append("Error getting part Mass Properties ({}): {}".format(self.get_name(), str(e)))
                carb.log_warn(self._warn_msgs[-1])

        if get_import_physics():
            req = OnshapeClient.get().parts_api.get_mass_properties(
                self.get_item("documentId"),
                "w",
                self.get_item("workspaceId"),
                self.get_item("elementId"),
                self.get_encoded_part_id(),
                _preload_content=False,
                link_document_id=self.get_item("documentId"),
                async_req=True,
            )
            self._task_mass_props = self.mass_thread_pool.submit(
                get_mass_properties, req
            )  # threading.Thread(target=get_mass_properties)
            self._task_mass_props.add_done_callback(self._on_get_mass_props)
        else:
            self._on_get_mass_props()
        # self._task_mass_props.start()

    def get_mass_properties_async(self):
        if get_import_physics():
            if not self._task_mass_props:
                self._get_mass_properties()
            elif self._task_mass_props.done():
                self._on_get_mass_props(None)

    def get_mass_properties(self, wait=False):
        if get_import_physics():
            # print("get Mass")
            if not self._task_mass_props:
                self._get_mass_properties()
            elif self._task_mass_props.done():
                return self.mass_properties
            if self._task_mass_props and wait:
                self._task_mass_props.result()
                # print("mass props:", self.mass_properties)

        return self.mass_properties

    def get_mesh(self):
        if self._task_mesh and self._task_mesh.done():
            return self._mesh

    def _get_mesh(self):
        tess_props = self.modelCols[4].get_value()
        self._error_msgs = []
        self._warn_msgs = []
        self._get_physical_material_properties()

        def __get_mesh():
            # print("1 - getting mesh", self._name["value"])
            self._task_physical_material.result()
            # print("2 - getting mesh", self._name["value"])
            # while not self.has_item("workspaceId"):
            #     time.sleep(0.5)
            # self._task_physical_material.result()
            # dtype = "w"
            # wmvid = self.get_item("workspaceId")
            # if self.has_item("documentMicroversion"):
            # print(self.get_item("configuration"))
            req = OnshapeClient.get().parts_api.get_faces1(
                self.get_item("documentId"),
                dtype,
                wmvid,
                self.get_item("elementId"),
                # self.get_encoded_part_id(),
                self.get_item("partId"),
                configuration=self.get_item("configuration"),
                angle_tolerance=tess_props.angle_tolerance,
                chord_tolerance=tess_props.chord_tolerance,
                max_facet_width=tess_props.max_chord,
                output_vertex_normals=True,
                output_texture_coordinates=True,
                output_face_appearances=True,
                output_index_table=True,
                _preload_content=False,
                link_document_id=self.get_item("documentId"),
                async_req=True,
            )
            req.wait()
            if req.successful():
                response = req.get()

                # print(response)
                r = json.loads(response.data)
                # print(r)
                self._mesh = Mesh.GetFromPartSpec(r)
                # print(self._mesh)
            else:
                self._error_msgs.append("error getting mesh {}: {}".format(self.get_name(), str(dir(req))))
                carb.log_error(self._error_msgs[-1])
                self._mesh = None

        if self._task_mesh and not self._task_mesh.done():
            self._task_mesh.cancel()
        dtype = "m"
        wmvid = self.get_item("documentMicroversion")
        # loop = asyncio.get_event_loop()
        # self._task_mesh = loop.create_task(__get_mesh())
        self._task_mesh = self.mesh_thread_pool.submit(__get_mesh)
        self._task_mesh.add_done_callback(self._on_mesh_imported)
        self.tess_props_changed = False
        self.parent_model._item_changed(self)
        self._on_get_physical_material_props(None)

    def get_name_async(self):
        self.name_req = OnshapeClient.get().metadata_api.get_wmvep_metadata(
            self.get_item("documentId"),
            "m",
            self.get_item("documentMicroversion"),
            self.get_item("elementId"),
            self.get_encoded_part_id(),
            _preload_content=False,
            async_req=True,
            link_document_id=self.get_item("documentId"),
        )

    def get_name_sync(self):
        with self.name_lock:
            if not self._name:
                self.name_req.wait()
                if self.name_req.successful():
                    m = self.name_req.get()
                    self._metadata = json.loads(m.data)
                    self._name = [a for a in self._metadata["properties"] if a["name"] == "Name"][0]
                else:
                    carb.log_error("FAilure getting Metadata for {}".format(self._name["value"]))

        return self._name["value"]

    def get_name(self):
        if self._name:
            return self._name["value"]
        else:
            return self.part.get_item("name")

    def _set_name(self, name):
        body = self._metadata
        body["properties"] = [self.get_name()]
        body["properties"][0]["value"] = name

        OnshapeClient.get().metadata_api.update_wmvep_metadata(
            self.get_item("documentId"),
            "w",
            self.get_item("workspaceId"),
            self.get_item("elementId"),
            # self.get_item("partId"),
            self.get_encoded_part_id(),
            "",
            body,
        )

    def get_processing(self):
        return "processing" if self.modelCols[0].get_value_as_bool() else ""

    def set_processing(self, value):
        self.modelCols[0].set_value(value)

    def get_value(self):
        return self.part

    def get_item_value(self, item=None, column_id=0):
        if item == self:
            return item.part
        return None

    def get_value_model(self, column_id):
        if column_id in [1, 2]:
            if not self._task_physical_material:
                self._get_physical_material_properties()
            # self._task_physical_material.result()
        return self.modelCols[column_id]

    def get_item_value_model(self, item=None, column_id=0):
        if item == self:
            if column_id == 2:
                if not self._task_physical_material:
                    self._get_physical_material_properties()
                # self._task_physical_material.result()
            return item.modelCols(column_id=column_id)
        return None

    def get_item_value_model_count(self, item=None):
        return 5  # Processing, Name, Physical Material, Mass, Tesselation Options

    def add_mesh_to_stage(self):
        pass


class OnshapePartListModel(ui.AbstractItemModel):
    def __init__(self, parts_list, **kwargs):
        super().__init__()
        self.mass_executor = kwargs.get(
            "thread_pool", ThreadPoolExecutor(max_workers=20, thread_name_prefix="onshape_parts_executor_pool")
        )
        self.mesh_executor = kwargs.get(
            "thread_pool", ThreadPoolExecutor(max_workers=20, thread_name_prefix="onshape_parts_executor_pool")
        )
        self.sort_column = 0
        self.usd_gen = kwargs.get("usd_gen", None)
        self.reverse_order = False
        # print(len(parts_list))
        c = [
            OnshapePart(parts_list[part], part, self, self.mesh_executor, self.mass_executor, **kwargs)
            for part in parts_list
        ]
        self._children = sorted(c, key=OnshapePart.get_name)
        self.sortKeys = [None, OnshapePart.get_name, OnshapePart.get_physical_material_name, OnshapePart.get_mass, None]

    def __del__(self):
        self._children = None

    def sort_children(self, column_id):
        if self.sortKeys[column_id]:
            if self.sort_column == column_id:
                self.reverse_order = not self.reverse_order
            else:
                self.reverse_order = False
            self.sort_column = column_id
            self._children = sorted(self._children, reverse=self.reverse_order, key=self.sortKeys[column_id])
            self._item_changed(None)

    def get_parts(self):
        return [item.get_value() for item in self._children]

    def add_part(self, part, key, **kwargs):
        self._children.append(OnshapePart(part, key, self, self.mass_executor, self.mass_executor, **kwargs))
        self._item_changed(None)

    def get_item_children(self, item=None):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            return self._children

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 5

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        """
        if item:
            if isinstance(item, OnshapePart):
                return item.get_value_model(column_id)

    def get_num_pending_meshes(self):
        pending_meshes = len(
            [i for i in self._children if i._task_mesh and not i._task_mesh.done() or i.modelCols[4].changed]
        )

        pending_process = 0
        if self.usd_gen:
            pending_process = len(self.usd_gen.pending_payloads) + len(
                [a for a in self.usd_gen.finalizing_meshes if not a.done()]
            )
            self.usd_gen.clear_done_tasks()
        return pending_meshes + pending_process

    def import_meshes(self, import_all=False):

        for c in self._children:
            if import_all or c.modelCols[4].changed:
                c.modelCols[4].changed = False
                # print("getting mesh", c)
                c._get_mesh()
                self._item_changed(c)


class OnshapePartsListDelegate(ui.AbstractItemDelegate):
    def __init__(self, model, **args):
        super().__init__()
        self.header = ["", "Name", "Material", "Mass", "Tesselation"]
        self._default_material_library = OnshapeClient.get_default_material_libraries()
        self.on_click_fn = args.get("click_fn", None)
        self.sort_fn = args.get("sort_fn", None)
        self._on_tess_props_changed_fn = args.get("tess_props_changed_fn", None)
        self._on_material_props_changed_fn = args.get("material_props_changed_fn", None)
        self.model = model
        self.theme = args.get("theme", "NvidiaDark")
        self._style = args.get("style", UI_STYLES[self.theme])
        # self._widget = None

    def __del__(self):
        pass
        # self._widget = None

    def build_branch(self, model, item, column_id, level, expanded):
        pass

    def on_click(self, item, col, *args):
        if self.on_click_fn:
            self.on_click_fn(item, col, *args)

    def build_header(self, column_id):
        with ui.HStack():
            if self.model.sort_column == column_id:
                ui.Image(
                    name="arrow_up" if self.model.reverse_order else "arrow_down",
                    style=self._style,
                    width=18,
                    height=18,
                )
            label = ui.Label(self.header[column_id])
        if self.sort_fn:
            label.set_mouse_pressed_fn(lambda a, b, c, d: self.sort_fn(column_id))

    def build_widget(self, model, item, column_id, level, expanded):
        # print(type(item))
        value_model = model.get_item_value_model(item, column_id)

        def get_style(item):
            if item.downloading:
                return "processing"
            if item.tess_props_changed:
                return "changed"
            if item._error_msgs:
                return "error"
            if item._warn_msgs:
                return "warning"
            return ""

        with ui.HStack(mouse_pressed_fn=lambda a, b, c, d, i=item, col=column_id: self.on_click(i, col, a, b, c, d)):
            ui.Spacer(width=1.5)

            if item:
                if column_id == 0:  # Processing
                    # ui.Label(str(type(model)))
                    ui.Image(
                        name=get_style(item), width=18, height=18, tooltip=item.make_error_tooltip()
                    )  # "processing","hidden"
                    # ui.Label(get_style(item), name=get_style(item))
                if column_id == 1:  # Name
                    # ui.Label(str(item))
                    ui.Label(
                        item.get_name(),
                        name=get_style(item),
                        tooltip_fn=lambda i=item: ui.Label(i.get_name(), style_type_name_override="Tooltip"),
                        elided_text=True,
                    )
                elif column_id == 2:  # Material
                    if value_model:
                        widget = PhysicsMaterialsWidget(
                            model=value_model,
                            selection=item.get_physical_material(),
                            height=22,
                            selection_changed_fn=lambda mat, model=model, item=item: self.material_changed(
                                model, item, mat
                            ),
                        )
                    else:
                        ui.Label("(downloading)", style={"color": 0xFF333333})
                elif column_id == 3:  # Mass Properties
                    mass = item.get_mass_properties()
                    mass_label = ui.Label("")

                    def make_tooltip(mass, has_material):
                        if mass and ("mass" in mass) and has_material:
                            # print(mass)
                            with ui.VStack():
                                ui.Label(
                                    "Volume: {:.2e} m^3".format(mass["volume"][0]), style_type_name_override="Tooltip"
                                )
                                inertia = mass["principalInertia"]
                                ui.Label(
                                    "Inertia: [{:.2e} {:.2e} {:.2e}]".format(inertia[0], inertia[1], inertia[2]),
                                    style_type_name_override="Tooltip",
                                )
                                com = mass["centroid"]
                                ui.Label(
                                    "Center of Mass: [{:0.2} {:0.2} {:0.2}]".format(com[0], com[1], com[2]),
                                    style_type_name_override="Tooltip",
                                )
                        else:
                            if not has_material:
                                ui.Label(
                                    "Must select a material (If one was selected, there was an error retrieving the data)",
                                    style_type_name_override="Tooltip",
                                )
                            else:
                                ui.Label("Part is not a solid", style_type_name_override="Tooltip")

                    if mass and item.get_physical_material():
                        if "mass" in mass:
                            mass_label.text = "{:.2e} kg".format(mass["mass"][0])
                            mass_label.name = ""
                    else:
                        mass_label.text = "--"
                        mass_label.name = "error"
                    mass_label.set_tooltip_fn(lambda a=mass: make_tooltip(mass, item.get_physical_material()))

                elif column_id == 4:  # Tesselation Properties
                    # pass
                    TesselationPropertiesItemWidget(
                        model=value_model,
                        end_edit_fn=lambda mat, model=model, item=item: self.tesselation_props_changed(
                            model, item, mat
                        ),
                    )
                ui.Spacer(width=1.5)

    def material_changed(self, model, item, material):
        # print("material changed")
        item._set_physical_material_properties(material)
        if self._on_material_props_changed_fn:
            self._on_material_props_changed_fn(item)
        model._item_changed(item)

    def tesselation_props_changed(self, model, item, tess):
        item.tess_props_changed = True
        if self._on_tess_props_changed_fn:
            self._on_tess_props_changed_fn(item)
        model._item_changed(item)


class OnshapePartsWidget(ui.Widget):
    def __init__(self, assembly_model, **kwargs):
        self.assembly = assembly_model
        parts = self.assembly.get_parts()
        self.total_imports = 0.0
        self._on_mesh_imported_fn = kwargs.get("mesh_imported_fn", None)
        self.progress_stack = kwargs.get("progress_stack", None)
        self.usd_gen = kwargs.get("usd_gen", None)
        self.usd_gen.mesh_imported_fn = self._on_mesh_imported
        kwargs["mesh_imported_fn"] = self._on_mesh_imported
        kwargs["tess_props_changed_fn"] = self._on_tesselation_props_changed
        kwargs["material_props_changed_fn"] = self._on_material_props_changed
        self.total_imports = len(parts)
        self.model = OnshapePartListModel(parts, **kwargs)  # TesselationPropertiesListModel([TesselationProperties()])
        self.delegate = OnshapePartsListDelegate(
            self.model, click_fn=self.show_context_menu, sort_fn=self.model.sort_children, **kwargs
        )
        # self.import_meshes(force_all=True)
        self.theme = kwargs.get("theme", "NvidiaDark")
        self._style = kwargs.get("style", UI_STYLES[self.theme])
        self.context_menu_item = None

        self._widget = ui.Frame(style=self._style, height=ui.Fraction(1))
        self._selection = omni.usd.get_context().get_selection()
        self._events = omni.usd.get_context().get_stage_event_stream()
        self._stage_event_subscription = None
        self._stage_event_subscription = self._events.create_subscription_to_pop(
            self._on_stage_event, name="Onshape Parts widget stage Watch"
        )
        self.build_ui()
        self.changed_from_stage = False

    def _on_material_props_changed(self, item):
        self.context_menu_item = item
        self.apply_material_to_selected()

    def _on_tesselation_props_changed(self, item):
        self.context_menu_item = item
        self.apply_tesselation_to_selected()

    def shutdown(self):
        # print(self, "shutting down parts widget")
        self._widget = None
        self.model = None
        self.delegate = None
        self._selection = None
        self._events = None
        self._stage_event_subscription = None

    def _on_stage_event(self, event):
        """Called with omni.usd.context when stage event"""

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            # print(self, "selection changed")
            asyncio.ensure_future(self._on_kit_selection_changed())

    async def _on_kit_selection_changed(self):
        selection = []
        for sel in self._selection.get_selected_prim_paths():
            for part in [a for a in self.model._children if a not in selection]:
                in_part = [Sdf.Path(i) in [Sdf.Path(sel), Sdf.Path(os.path.dirname(sel))] for i in part.parts_usds]
                if np.any(in_part):
                    selection.append(part)
                    break
        if selection != self._mesh_list.selection:
            self.changed_from_stage = True
            self._mesh_list.selection = selection

    def _on_parts_selection_changed(self, source):
        # print(self, "parts selection changed")
        if self.changed_from_stage:
            self.changed_from_stage = False
        else:
            selection = []  # self._selection.get_selected_prim_paths()
            for item in self._mesh_list.selection:
                selection = selection + [i for i in item.parts_usds if i not in selection]
            # print(selection)
            self._selection.set_selected_prim_paths(
                list(set([i for i in selection if omni.usd.get_context().get_stage().GetPrimAtPath(i)])), False
            )

    def find_similar_parts(self):
        item = self.context_menu_item
        self._mesh_list.selection = [i for i in self.model._children if item.is_similar(i)]

    def apply_material_to_selected(self, *args):
        material = self.context_menu_item.modelCols[2].selection
        for item in self._mesh_list.selection:
            if item.modelCols[2].selection != material:
                item.modelCols[2].selection = material
                item._set_physical_material_properties(material)
                self.model._item_changed(item)
            # print(item.get_name(), material.get_name(), item.modelCols[2].selection.get_name())

    def apply_tesselation_to_selected(self, *args):
        props = self.context_menu_item.modelCols[4].get_value()
        self.total_imports = len(self._mesh_list.selection)
        # print(self.total_imports)
        if self.total_imports > 0 and self.progress_stack:
            self.progress.model.set_value(0)
            self.progress_stack.visible = True
        for item in self._mesh_list.selection:
            item.modelCols[4].set_value(props)
            item.tess_props_changed = item.modelCols[4].changed
            item._get_mesh()
            # self.delegate.tesselation_props_changed(self.model, item, props)
            self.model._item_changed(item)

        # self.import_meshes()

    def replace_selecetd_parts(self, *args):
        carb.log_warn("Replace selected parts still not implemented", self.context_menu_item.get_name())

    def show_context_menu(self, item, column, *args):
        if args[2] == 1:
            self.context_menu_item = item
            # self.context_replace_selected_parts.visible = column == 1
            # self.context_replace_selected_parts.enabled = len(self._mesh_list.selection) > 1 or (
            #     len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            # )
            self.context_find_similar.visible = False  # column == 1
            self.context_material.visible = column == 2
            self.context_material.enabled = len(self._mesh_list.selection) > 1 or (
                len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            )
            self.context_tesselation.visible = column == 4
            self.context_tesselation.enabled = len(self._mesh_list.selection) > 1 or (
                len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            )
            # self.context_menu.position_x = args[0]
            # self.context_menu.position_y = args[1]
            if (
                self.context_find_similar.visible
                # or self.context_replace_selected_parts.visible
                or self.context_material.visible
                or self.context_tesselation.visible
            ):
                self.context_menu.show()

    def import_meshes(self, force_all=False):
        context = omni.usd.get_context()
        stage = context.get_stage()
        if stage:
            edit_target = Usd.EditTarget(stage.GetRootLayer())
            stage.SetEditTarget(edit_target)
        if force_all:
            self.total_imports = len(self.model._children)
        else:
            self.total_imports = self.model.get_num_pending_meshes()
        # print(self.total_imports)
        if self.total_imports > 0 and self.progress_stack:
            self.progress.model.set_value(0)
            self.progress_stack.visible = True
        self.model.import_meshes(force_all)

    def _on_mesh_imported(self, item):
        if self.progress_stack:
            remaining = self.model.get_num_pending_meshes()
            v = float(self.total_imports - remaining) / self.total_imports
            # print(remaining, v)
            if v > self.progress.model.get_value_as_float():
                # with self.ui_lock:
                self.progress.model.set_value(v)
                self.progress_stack.visible = remaining != 0

        # Update progress bar
        if self._on_mesh_imported_fn:
            self._on_mesh_imported_fn(item, remaining == 0)

    def shutdown(self):
        # print(self, "shutting down parts widget")
        self._widget = None
        self.model = None
        self.delegate = None
        self._selection = None
        self._events = None
        self._stage_event_subscription = None

    def _on_stage_event(self, event):
        """Called with omni.usd.context when stage event"""

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            # print(self, "selection changed")
            asyncio.ensure_future(self._on_kit_selection_changed())

    async def _on_kit_selection_changed(self):
        selection = []
        for sel in self._selection.get_selected_prim_paths():
            for part in [a for a in self.model._children if a not in selection]:
                in_part = [Sdf.Path(i) in [Sdf.Path(sel), Sdf.Path(os.path.dirname(sel))] for i in part.parts_usds]
                if np.any(in_part):
                    selection.append(part)
                    break
        if selection != self._mesh_list.selection:
            self.changed_from_stage = True
            self._mesh_list.selection = selection

    def _on_parts_selection_changed(self, source):
        # print(self, "parts selection changed")
        if self.changed_from_stage:
            self.changed_from_stage = False
        else:
            selection = []  # self._selection.get_selected_prim_paths()
            for item in self._mesh_list.selection:
                selection = selection + [i for i in item.parts_usds if i not in selection]
            # print(selection)
            self._selection.set_selected_prim_paths(
                list(set([i for i in selection if omni.usd.get_context().get_stage().GetPrimAtPath(i)])), False
            )

    def find_similar_parts(self):
        item = self.context_menu_item
        self._mesh_list.selection = [i for i in self.model._children if item.is_similar(i)]

    def apply_material_to_selected(self, *args):
        material = self.context_menu_item.modelCols[2].selection
        for item in self._mesh_list.selection:
            if item.modelCols[2].selection != material:
                item.modelCols[2].selection = material
                item._set_physical_material_properties(material)
                self.model._item_changed(item)
            # print(item.get_name(), material.get_name(), item.modelCols[2].selection.get_name())

    def apply_tesselation_to_selected(self, *args):
        props = self.context_menu_item.modelCols[4].get_value()
        # print(props.angle_tolerance)
        for item in self._mesh_list.selection:
            item.modelCols[4].set_value(props)
            # self.delegate.tesselation_props_changed(self.model, item, props)
            # print(item.modelCols[4].get_value().angle_tolerance)
            self.model._item_changed(item)
        self.import_meshes()

    def replace_selecetd_parts(self, *args):
        carb.log_warn("Replace selected parts still not implemented", self.context_menu_item.get_name())

    def show_context_menu(self, item, column, *args):
        if args[2] == 1:
            self.context_menu_item = item
            # self.context_replace_selected_parts.visible = column == 1
            # self.context_replace_selected_parts.enabled = len(self._mesh_list.selection) > 1 or (
            #     len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            # )
            self.context_find_similar.visible = False  # column == 1
            self.context_material.visible = column == 2
            self.context_material.enabled = len(self._mesh_list.selection) > 1 or (
                len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            )
            self.context_tesselation.visible = column == 4
            self.context_tesselation.enabled = len(self._mesh_list.selection) > 1 or (
                len(self._mesh_list.selection) == 1 and self._mesh_list.selection[0] != item
            )
            # self.context_menu.position_x = args[0]
            # self.context_menu.position_y = args[1]
            if (
                self.context_find_similar.visible
                # or self.context_replace_selected_parts.visible
                or self.context_material.visible
                or self.context_tesselation.visible
            ):
                self.context_menu.show()

    def import_meshes(self, force_all=False):
        context = omni.usd.get_context()
        stage = context.get_stage()
        if stage:
            edit_target = Usd.EditTarget(stage.GetRootLayer())
            stage.SetEditTarget(edit_target)
        if force_all:
            self.total_imports = len(self.model._children)
        else:
            self.total_imports = self.model.get_num_pending_meshes()
        # print(self.total_imports)
        if self.total_imports > 0 and self.progress_stack:
            self.progress.model.set_value(0)
            self.progress_stack.visible = True
        self.model.import_meshes(force_all)

    def _on_mesh_imported(self, item):
        if self.progress_stack:
            remaining = self.model.get_num_pending_meshes()
            v = float(self.total_imports - remaining) / self.total_imports
            # print(remaining, v)
            if v > self.progress.model.get_value_as_float():
                # with self.ui_lock:
                self.progress.model.set_value(v)
                self.progress_stack.visible = remaining != 0

        # Update progress bar
        if self._on_mesh_imported_fn:
            self._on_mesh_imported_fn(item, not self.progress_stack.visible)

    def build_ui(self):
        with self._widget:
            self.context_menu = ui.Menu("parts_context_menu")
            with self.context_menu:
                self.context_material = ui.MenuItem(
                    "Apply this Material To selected parts",
                    checkable=False,
                    triggered_fn=self.apply_material_to_selected,
                )
                self.context_material.set_tooltip("Applies the Physical material of this part to all selected parts")

                self.context_tesselation = ui.MenuItem(
                    "Apply This Tesselation To selected parts",
                    checkable=False,
                    triggered_fn=self.apply_tesselation_to_selected,
                )
                self.context_tesselation.set_tooltip(
                    "Applies the tesselation properties of this part to all selected parts"
                )
                self.context_replace_selected_parts = ui.MenuItem(
                    "Replace selected parts with this", checkable=False, triggered_fn=self.replace_selecetd_parts
                )
                self.context_replace_selected_parts.set_tooltip_fn(
                    lambda: ui.Label("Replace all Selected parts with this part", style_type_name_override="Tooltip")
                )
                self.context_find_similar = ui.MenuItem(
                    "Find similar parts", checkable=False, triggered_fn=self.find_similar_parts
                )
                self.context_replace_selected_parts.visible = False
            with ui.VStack():
                if not self.progress_stack:
                    self.progress_stack = ui.HStack(height=25, style={"margin": 3})
                with self.progress_stack:
                    ui.Label("Importing Parts", width=0)
                    ui.Spacer(width=6)
                    self.progress = ui.ProgressBar(height=22)
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    # vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style_type_name_override="TreeView",
                    style=self._style,
                    height=ui.Fraction(1),
                    # width=ui.Percent(100)
                ):
                    self._mesh_list = ui.TreeView(
                        self.model,
                        delegate=self.delegate,
                        style=self._style,
                        header_visible=True,
                        alignment=ui.Alignment.CENTER_TOP,
                        column_widths=[25, ui.Fraction(1), ui.Pixel(280), ui.Pixel(80), ui.Pixel(140)],
                        columns_resizable=True,
                    )
                    self._mesh_list.set_selection_changed_fn(self._on_parts_selection_changed)
