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

from omni.isaac.onshape.scripts.style import UI_STYLES
from omni.isaac.onshape.client import OnshapeClient
from omni.isaac.onshape.widgets.parts_widget import OnshapePartsWidget, OnshapePart


from concurrent.futures import ThreadPoolExecutor


def make_part_id(part):
    string_id = "".join(
        [part["documentId"], part["documentMicroversion"], part["elementId"], part["partId"], part["configuration"]]
    )
    return hash(string_id)


class MateGroup(object):
    def __init__(self, group):
        self.name = group["name"]
        self.occurrences = [m["occurrence"][0] for m in relation["mates"] if m["occurrence"]]


class MateRelation(object):
    def __init__(self, relation):
        self.name = relation["name"]
        self.occurrences = [m["featureId"] for m in relation["mates"]]


distance_unit = {"cm": 0.01, "mm": 0.001, "m": 1.00, "yd": 0.9144, "ft": 0.3048, "in": 0.0254}


def nop(a):
    return a


rotation_unit = {"deg": lambda a: nop(a), "rad": lambda a: degrees(a)}


class Mate(object):
    def is_locked(self):
        if self.type in ["SLIDER", "REVOLUTE"]:
            return self.limits[0] == self.limits[1] and self.limits[0] is not None
        elif self.type == "CYLINDRICAL":
            return (self.limits_linear[0] == self.limits_linear[1] and self.limits_linear[0] is not None) and (
                self.limits_radial[0] == self.limits_radial[1] and self.limits_radial[0] is not None
            )
        elif self.type == "BALL":
            return False  # It is always at least a single revolute joint
        carb.log_warn("Mate type {} unsupported".format(self.type))
        return False

    def __init__(self, mate, details):
        # print(mate)
        self.name = mate["featureData"]["name"]
        # print(self.name)
        self.type = mate["featureData"]["mateType"]
        # print(mate)
        self.limits = [None, None]
        self.occurrences = [m["matedOccurrence"] for m in mate["featureData"]["matedEntities"]]
        self.positions = [
            Gf.Matrix4d(
                m["matedCS"]["xAxis"] + [0],
                m["matedCS"]["yAxis"] + [0],
                m["matedCS"]["zAxis"] + [0],
                [i for i in m["matedCS"]["origin"]] + [1],
            )
            for m in mate["featureData"]["matedEntities"]
        ]
        # Axis is actually alwayz Z
        # print(details)
        if self.type == "REVOLUTE":
            self.axis = (
                "Z"
            )  # [ d["message"]["value"] for d in details["parameters"] if d["message"]["parameterId"] == "rotationType"][0]
            value = [
                d["message"]["value"]
                for d in details["message"]["parameters"]
                if d["message"]["parameterId"] == "rotation"
            ]
            self.value = value[0] if value else 0
            self.limits = [
                [
                    rotation_unit[d["message"]["expression"].split(" ")[1]](
                        float(d["message"]["expression"].split(" ")[0])
                    )
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitAxialZMin"
                ][0],
                [
                    rotation_unit[d["message"]["expression"].split(" ")[1]](
                        float(d["message"]["expression"].split(" ")[0])
                    )
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitAxialZMax"
                ][0],
            ]

        # Axis is actually alwayz Z
        elif self.type == "SLIDER":
            self.axis = (
                "Z"
            )  # [ d["message"]["value"] for d in details["parameters"] if d["message"]["parameterId"] == "rotationType"][0]
            value = [
                d["message"]["value"]
                for d in details["message"]["parameters"]
                if d["message"]["parameterId"] == "rotation"
            ]
            self.value = value[0] if value else 0
            self.limits = [
                [
                    float(d["message"]["expression"].split(" ")[0])
                    * distance_unit[d["message"]["expression"].split(" ")[1]]
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitZMin"
                ][0],
                [
                    float(d["message"]["expression"].split(" ")[0])
                    * distance_unit[d["message"]["expression"].split(" ")[1]]
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitZMax"
                ][0],
            ]
        elif self.type == "CYLINDRICAL":
            self.axis = "Z"
            self.limits_linear = [
                [
                    float(d["message"]["expression"].split(" ")[0])
                    * distance_unit[d["message"]["expression"].split(" ")[1]]
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitZMin"
                ][0],
                [
                    float(d["message"]["expression"].split(" ")[0])
                    * distance_unit[d["message"]["expression"].split(" ")[1]]
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitZMax"
                ][0],
            ]
            self.limits_radial = [
                [
                    rotation_unit[d["message"]["expression"].split(" ")[1]](
                        float(d["message"]["expression"].split(" ")[0])
                    )
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitAxialZMin"
                ][0],
                [
                    rotation_unit[d["message"]["expression"].split(" ")[1]](
                        float(d["message"]["expression"].split(" ")[0])
                    )
                    if not d["message"]["nullValue"]
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitAxialZMax"
                ][0],
            ]
        elif self.type == "BALL":
            self.axis = "Z"
            self.axis_cone = "Y"
            self.limit_cone = 180
            has_limits = [
                d["message"]["value"]
                for d in details["message"]["parameters"]
                if d["message"]["parameterId"] == "limitsEnabled"
            ][0]
            if has_limits:
                self.limit_cone = [
                    rotation_unit[d["message"]["expression"].split(" ")[1]](
                        float(d["message"]["expression"].split(" ")[0])
                    )
                    if not d["message"].get("nullValue", None)
                    else None
                    for d in details["message"]["parameters"]
                    if d["message"]["parameterId"] == "limitEulerConeAngleMax"
                ][0]
            self.limits = [None, None]
            if self.limit_cone == 0:
                self.type = "REVOLUTE"
                self.value = 0

        # self.limit_min


class OnshapeAssemblyModel(ui.AbstractItemModel):
    def __init__(self, document, element, **kwargs):
        super().__init__()
        self.document = document
        self.workspace = document.get_workspace()
        self.element = element
        self.assembly = None
        self._instances_flat = {}
        self._parts_flat = {}
        self.features_lock = threading.Lock()
        self.assembly_features = {}
        self.features_details = {}
        self.features_map = {}
        self.occurrences = {}
        self._root = None
        self.assembly_definition_task = None
        self.config_changed = False
        self._children = []
        self.assembly_loaded = False
        self._assembly_features_task = []
        self.thread_pool = ThreadPoolExecutor(max_workers=100, thread_name_prefix="onshape_assembly_collection_pool")
        self._app_update_sub = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)
        )
        self.pending_notify = False
        confs = OnshapeClient.get().elements_api.get_configuration(
            self.document.document_id,
            self.document.get_wdid(),
            self.document.get_workspace(),
            self.element["id"],
            _preload_content=False,
        )
        self.main_loop = asyncio.get_event_loop()

        confs = json.loads(confs.data)
        self.conf_models = [ConfigurationListModel(c) for c in confs["configurationParameters"]]
        self._on_assembly_loaded_fn = kwargs.get("assembly_loaded_fn", None)
        for cm in self.conf_models:
            cm.add_item_changed_fn(lambda a, b: self.conf_changed(a, b))
        # self._get_assembly_definition()

    def __del__(self):
        self._app_update_sub = None

    def _on_update(self, t):
        if self.pending_notify:
            self.on_assembly_loaded(None)
            self.pending_notify = False

    def get_conf(self):
        conf = ""
        for i, a in enumerate(self.conf_models):
            conf += a.get_item_value()
            if i < len(self.conf_models) - 1:
                conf += ";"
        return conf

    def conf_changed(self, model, conf):
        self.config_changed = True
        self._get_assembly_definition()

    def _get_assembly_features(self, document_id, wtype, workspace, eid):
        def _get_features(req):
            req.wait()
            if req.successful():
                # print("get features", document_id, wtype, workspace, eid)
                response = req.get()
                # print(response.data)
                data = json.loads(response.data)
                # print(data)
                with self.features_lock:
                    for f in [f for f in data["features"] if f["message"]["nodeId"] in self.assembly_features]:
                        self.features_details[f["message"]["nodeId"]] = f
                    # print("features", self.assembly_features.keys())
                    # print("details", self.features_details.keys())
            else:
                carb.log_warn("Unable to get features".format(req))

        r = OnshapeClient.get().assemblies_api.get_features(
            document_id, wtype, workspace, eid, _preload_content=False, async_req=True
        )
        self._assembly_features_task.append(self.thread_pool.submit(_get_features, r))
        # if not self.features_details:
        #     self._assembly_features_task.add_done_callback(self.on_assembly_loaded)
        # _get_features()
        # self.on_assembly_loaded(None)

    def get_assembly_definition_sync(self):
        if not self.assembly_definition_task:
            self._get_assembly_definition()

        self.assembly_definition_task.result()

    def _get_assembly_definition(self):
        def get_def(req):
            # print(self.element)
            # configuration = ''.join([c.get_selected().get_item_value()+";" for c in self.conf_models])
            # print(
            #     self,
            #     self.document.document_id,
            #     "w",
            #     self.document.get_workspace(),
            #     self.element["id"],
            #     # configuration
            #     # self.conf_model.get_selected().get_item_value(),
            # )
            req.wait()
            if req.successful():
                response = req.get()
                self.assembly = json.loads(response.data)
                self.assembly_features = {f["id"]: f for f in self.assembly["rootAssembly"]["features"]}
                for f in self.assembly_features:
                    self.assembly_features[f]["parent"] = "root"

                self._get_assembly_features(
                    self.document.document_id,
                    self.document.get_wdid(),
                    self.document.get_workspace(),
                    self.element["id"],
                )

                # make dictionary to search for part by its partID
                self._root = OnshapeAssemblyItem(self.assembly["rootAssembly"])
                self._root.set_item("type", "assembly")
                self._parts_flat = {}
                self.features_map[self._root.uid] = [f["id"] for f in self.assembly["rootAssembly"]["features"]]
                for i, part in enumerate(self.assembly["parts"]):
                    key = make_part_id(part)
                    part["key"] = key
                    if "documentVersion" not in part:
                        part["workspaceId"] = self.document.get_workspace()
                    p = OnshapeAssemblyItem(part, self.document)
                    self._parts_flat[key] = p

                # Sort instances by path length (Make parent path be always first on list)
                self.assembly["rootAssembly"]["occurrences"] = sorted(
                    self.assembly["rootAssembly"]["occurrences"], key=lambda a: len(a["path"])
                )
                # print( json.dumps(self.assembly["rootAssembly"]["occurrences"]))
                # Add instances of root assembly in the flat instances list
                self._instances_flat = {}
                for inst in self.assembly["rootAssembly"]["instances"]:
                    # for f in self._assembly_features_task:
                    #     f.result()
                    #     self._assembly_features_task.remove(f)
                    self._instances_flat[inst["id"]] = OnshapeAssemblyItem(inst)
                    # Create Unique identifier on part instances to refer to the part dictionary
                    if inst["type"].lower() == "part":
                        # print(inst)
                        hash_id = make_part_id(inst)
                        self._instances_flat[inst["id"]]._item["hashId"] = hash_id
                        self._parts_flat[hash_id]._item["workspaceId"] = self.document.get_workspace()
                        self._parts_flat[hash_id]._item["name"] = inst["name"].strip()[:-4]
                    if inst["type"].lower() == "assembly":
                        if "documentVersion" in inst:
                            self._get_assembly_features(
                                inst["documentId"], "v", inst["documentVersion"], inst["elementId"]
                            )
                        elif "documentMicroversion" in inst:
                            self._get_assembly_features(
                                inst["documentId"], "m", inst["documentMicroversion"], inst["elementId"]
                            )

                # Add instances of sub assemblies in the flat instances list
                for sub_assm in self.assembly["subAssemblies"]:
                    self.features_map[sub_assm["documentId"] + sub_assm["elementId"]] = [
                        f["id"] for f in sub_assm["features"]
                    ]
                    for feature in sub_assm["features"]:
                        self.assembly_features[feature["id"]] = feature
                    # for f in self._assembly_features_task:
                    #     f.result()
                    #     self._assembly_features_task.remove(f)
                    if sub_assm["features"]:
                        if "documentVersion" in sub_assm:
                            self._get_assembly_features(
                                sub_assm["documentId"], "v", sub_assm["documentVersion"], sub_assm["elementId"]
                            )
                        elif "documentMicroversion" in sub_assm:
                            self._get_assembly_features(
                                sub_assm["documentId"], "m", sub_assm["documentMicroversion"], sub_assm["elementId"]
                            )
                        # else:
                        #     print(sub_assm.keys())

                    for inst in sub_assm["instances"]:
                        self._instances_flat[inst["id"]] = OnshapeAssemblyItem(inst)
                        # if inst["type"].lower() == "assembly":
                        #     if "documentVersion" in inst:
                        #         self._get_assembly_features(
                        #             inst["documentId"], "v", inst["documentVersion"], inst["elementId"]
                        #         )
                        #     elif "documentMicroversion" in inst:
                        #         self._get_assembly_features(
                        #             inst["documentId"], "m", inst["documentMicroversion"], inst["elementId"]
                        #         )
                        # Create Unique identifier on part instances to refer to the part dictionary
                        if inst["type"].lower() == "part":
                            hash_id = make_part_id(inst)
                            self._instances_flat[inst["id"]]._item["hashId"] = hash_id
                            self._parts_flat[hash_id]._item["workspaceId"] = self.document.get_workspace()
                            self._parts_flat[hash_id]._item["name"] = inst["name"].strip()[:-4]
                # print(json.dumps(self.assembly_features))
                # Parse Assembly tree
                for item in self.assembly["rootAssembly"]["occurrences"]:
                    # print(item)
                    for i, element_id in enumerate(item["path"]):
                        if i == 0:
                            # print(self._instances_flat[element_id])
                            if not self._root.has_child(element_id):
                                last_item = self._root.add_child(self._instances_flat[element_id])
                        else:
                            last_item = self._instances_flat[item["path"][i - 1]].add_child(
                                self._instances_flat[element_id]
                            )

                    last_item._item["fixed"] = item["fixed"]
                    last_item._item["hidden"] = item["hidden"]
                    # Add transform to last item
                    last_item.transform["".join(item["path"])] = item["transform"]
                    self.occurrences[item["path"][-1]] = "".join(item["path"])
                self.pending_notify = True
            else:
                carb.log_error(dir(req.get()))

        # print(self.document.document_id, self.document.get_wdid(), self.document.get_workspace(), self.element["id"])
        req = OnshapeClient.get().assemblies_api.get_assembly_definition(
            self.document.document_id,
            self.document.get_wdid(),
            self.document.get_workspace(),
            self.element["id"],
            configuration=self.get_conf(),
            _preload_content=False,
            include_mate_features=True,
            include_non_solids=True,
            async_req=True,
        )
        # get_def()
        self.assembly_definition_task = self.thread_pool.submit(get_def, req)
        # print(f.result())
        # self.thread_pool.shutdown(wait=True)
        for inst in self._instances_flat:
            if inst not in self.features_map:
                self.features_map[inst] = []

        # self._assembly_definition_task = self.thread_pool.submit(get_def)
        # if self.features_details:
        #     self._assembly_definition_task.add_done_callback(self.on_assembly_loaded)

    def assembly_features_sync(self):
        for f in self._assembly_features_task:
            f.result()
            self._assembly_features_task.remove(f)

    def on_assembly_loaded(self, task):

        self.assembly_loaded = True
        if self._on_assembly_loaded_fn:
            self._on_assembly_loaded_fn()
        # self._assembly_definition_task.result()
        self._item_changed(None)

    def get_instances_flat(self):
        return self._instances_flat

    def get_item_children(self, item):
        """Returns all the children when the widget asks it."""
        if item is None:
            item = self._root
        return item.get_children()

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 1

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        In our case we use ui.SimpleStringModel.
        """
        return item

    # def _get_assembly_features(self):
    #     def get_def():
    #         response = OnshapeClient.get().assemblies_api.get_assembly_definition(self.document.document_id, "w", self.document.get_workspace(), self.element["id"], _preload_content = False)
    #         self.assembly = json.loads(response.data)
    #     self._assembly_definition_task = threading.Thread(target=get_def)
    #     self._assembly_definition_task.start()

    def get_assembly(self):
        return self.assembly

    def get_name(self):
        return self.element["name"]

    def get_parts(self):
        return self._parts_flat

    def get_mates(self):
        return []

    def __del__(self):
        self._on_assembly_loaded_fn = None


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


class OnshapeAssemblyItem(ui.AbstractItem):
    def __init__(self, item, parent=None):
        super().__init__()
        self._item = item
        self.__children = {}
        self._children = []
        self._metadata = {}
        self._parent = parent
        self.transform = {}
        self.uid = item["documentId"] + item["elementId"]
        if "features" in item:
            self.features = [f["id"] for f in item["features"]]

    def get_type(self):
        return self._item["type"]

    def _get_assembly_metadata(self):
        pass

    def get_workspace(self):
        if self._parent:
            return self._parent.get_workspace()

    def get_children(self):
        return list(self.__children.values())

    def has_child(self, key):
        return key in self.__children.keys()

    def has_item(self, key):
        return key in self._item

    def get_item(self, key):
        if key in self._item:
            return self._item[key]
        return None

    def set_item(self, key, value):
        # print(self, key, value)
        self._item[key] = value

    def add_child(self, item):
        if item.get_item("id") not in self.__children.keys():
            self.__children[item.get_item("id")] = item
        return item

    # def print_recursive(self, level=0):
    #     for child in self.get_children():
    #         print(" " * level, " <<{}>>".format(child["type"]), child["name"])
    #         child.print_recursive(level + 1)


class ConfigurationItem(ui.AbstractItem):
    def __init__(self, config, param_id, _id):
        super().__init__()
        self.config = config
        self.model = ui.SimpleStringModel(self.config["optionName"])
        self.param_id = param_id
        self.id = _id

    def get_value_as_string(self):
        return self.config["optionName"]

    def get_value(self):
        return self.config["option"]

    def get_item_value(self):
        return "{}={}".format(self.param_id, self.config["option"])


class ConfigurationListModel(ui.AbstractItemModel):
    def __init__(self, configs):
        super().__init__()
        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(lambda a: self.item_changed(None))
        self.model = ui.SimpleIntModel()
        self.parameter_id = configs["message"]["parameterId"]
        self.node_id = configs["message"]["nodeId"]
        self.name = configs["message"]["parameterName"]
        self.type = configs["typeName"]
        self.QuantityType = "INTEGER"
        if self.type == "BTMConfigurationParameterQuantity":
            self.unit = configs["message"]["rangeAndDefault"]["message"]["units"]
            min_value = configs["message"]["rangeAndDefault"]["message"]["minValue"]
            max_value = configs["message"]["rangeAndDefault"]["message"]["maxValue"]
            self.QuantityType = configs["message"]["quantityType"]
            if self.QuantityType == "INTEGER":
                self.model = ui.SimpleIntModel()
                self.model.set_min(int(min_value))
                self.model.set_max(int(max_value))
                self.model.set_value(int(configs["message"]["rangeAndDefault"]["message"]["defaultValue"]))
            elif self.QuantityType in ["REAL", "ANGLE", "LENGTH"]:
                self.model = ui.SimpleFloatModel()
                self.model.set_min(float(min_value))
                self.model.set_max(float(max_value))
                self.model.set_value(float(configs["message"]["rangeAndDefault"]["message"]["defaultValue"]))

        elif self.type == "BTMConfigurationParameterEnum":
            self._items = {}
            if "options" in configs["message"].keys():
                self._items = {
                    c["message"]["option"]: ConfigurationItem(c["message"], self.parameter_id, i)
                    for i, c in enumerate(configs["message"]["options"])
                }
                # print(self._items[configs["message"]["defaultValue"]].id)
                self.model.set_value(self._items[configs["message"]["defaultValue"]].id)
        elif self.type == "BTMConfigurationParameterString":
            self.model = ui.SimpleStringModel()
            self.model.set_value(configs["message"]["defaultValue"])
        elif self.type == "BTMConfigurationParameterBoolean":
            self.model = ui.SimpleBoolModel()
            self.model.set_value(configs["message"]["defaultValue"])
        self.model.add_value_changed_fn(lambda a: self.item_changed(a))

    def item_changed(self, item=None):
        self._item_changed(None)

    def get_value(self):
        if self.type == "BTMConfigurationParameterQuantity":
            if self.QuantityType == "INTEGER":
                return self.model.get_value_as_int()
            else:
                return self.model.get_value_as_float()
        elif self.type == "BTMConfigurationParameterBoolean":
            return self.model.get_value_as_bool()
        elif self.type == "BTMConfigurationParameterString":
            return self.model.get_value_as_string()
        elif self.type == "BTMConfigurationParameterEnum":
            i = self._current_index.get_value_as_int()
            return self.get_item_children()[i].get_value()

    def build_widget(self):
        self.frame = ui.Frame(style={"spacing": 2})
        with self.frame:
            with ui.HStack():
                ui.Label(self.name)
                if self.type == "BTMConfigurationParameterEnum":
                    ui.ComboBox(self)
                elif self.type == "BTMConfigurationParameterQuantity":
                    if self.QuantityType == "INTEGER":
                        ui.IntSlider(self.model)
                    else:
                        ui.FloatDrag(self.model)
                elif self.type == "BTMConfigurationParameterBoolean":
                    ui.CheckBox(self.model)
                elif self.type == "BTMConfigurationParameterString":
                    ui.StringField(self.model)

    def get_item_value(self):
        extra = ""
        if self.type == "BTMConfigurationParameterQuantity":
            if self.unit:
                extra = "+{}".format(self.unit)
        return "{}={}{}".format(self.parameter_id, self.get_value(), extra)

    def get_item_children(self, parentItem=None):
        if parentItem is None:
            return list(self._items.values())
        return []

    def get_selected(self):
        return self.get_item_children(None)[self.model.get_value_as_int()]

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._current_index
        return item.model


class AssemblyDetailsWidget:
    def __init__(self, assembly_model, usd_gen, **kwargs):

        self.loop = asyncio.get_event_loop()
        self.model = assembly_model
        self.usd_gen = usd_gen
        self.conf_models = self.model.conf_models
        self.subs = self.model.subscribe_item_changed_fn(lambda a, b: weakref.proxy(self).build_ui())
        self.widget = ui.Frame(height=ui.Fraction(1))
        self.theme = kwargs.get("theme", "NvidiaDark")
        self._style = kwargs.get("style", UI_STYLES[self.theme])
        self.mesh_imported_fn = kwargs.get("mesh_imported_fn", None)
        self.delegate = OnshapeAssemblyTreeViewDelegate()
        self._parts_widget = None
        # self.args = kwargs

        # self.build_ui()

    def __del__(self):
        self.subs = None
        if self._parts_widget:
            self._parts_widget.shutdown()
        self._parts_widget = None

    def conf_changed(self):
        self.loading_image.visible = True

    def build_ui(self, *kwargs):
        self.widget.clear()
        if self.model:
            self.subs = None  # once UI is built, model doesn't change anymore.
            with self.widget:
                with ui.VStack(style=self._style, spacing=5):
                    with ui.HStack(height=32, spacing=5):
                        self.loading_image = ui.Image(name="processing", width=32, height=32)
                        ui.Label("{}".format(self.model.get_name()), style={"font_size": 32}, height=0)

                    # with ui.VStack():
                    #     for c in self.
                    ui.Spacer(height=3)
                    if self.conf_models:
                        with ui.HStack(height=0):
                            with ui.CollapsableFrame("Configuration", height=0):
                                with ui.VStack(spacing=3):
                                    for model in self.conf_models:
                                        model.build_widget()
                                        model.add_item_changed_fn(lambda a, b: weakref.proxy(self).conf_changed())
                            ui.Spacer()
                        ui.Spacer(height=3)
                    if self._parts_widget:
                        self._parts_widget.shutdown()
                        self._parts_widget = None
                    # with ui.HStack(height=ui.Fraction(1)):
                    ui.Label(
                        "Total Unique Parts: {}".format(len(self.model.get_parts())),
                        width=ui.Percent(0),
                        height=0,
                        style_type_name_override="TreeView",
                    )

                    ui.Spacer(height=3)
                    self.progress_stack = ui.HStack(height=22)
                    self._parts_widget = OnshapePartsWidget(
                        self.model,
                        style=self._style,
                        progress_stack=self.progress_stack,
                        usd_gen=self.usd_gen,
                        mesh_imported_fn=weakref.proxy(self.mesh_imported_fn),
                    )
                    # with ui.VStack(width=ui.Pixel(300)):
                    #     for c in self.conf_models:
                    #         with ui.HStack(height = ui.Pixel(22)):
                    #             ui.Label(c.name)
                    #             ui.ComboBox(c)
                # ui.TreeView(self.model, delegate=self.delegate, style=self._style)

            async def create():
                self.usd_gen.create_all_stages(self._parts_widget.model._children)
                self.usd_gen.build_assemblies()

            # print(self.usd_gen.material_stage)
            # task = threading.Thread(target=create)
            # task.start()
            self.loop.create_task(create())
            # create()


class OnshapeAssemblyTreeViewDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self.num_columns = 1
        self._highligting_enabled = None
        self._highligting_text = None

    def build_branch(self, model, item, column_id, level, expanded):
        if column_id == 0:
            with ui.HStack(width=16 * (level + 2), height=0):
                ui.Spacer()
                if model.can_item_have_children(item):
                    ui.Label("-" if expanded else "+")
                    # image_name = "Minus" if expanded else "Plus"
                    ui.Spacer(width=4)

    def on_mouse_pressed(self, button, item, expanded):
        """Called when the user press the mouse button on the item"""
        if button != 1:
            # It's for context menu only
            return
        pass

    def add_List_view(self, listView):
        self.listView = listView

    def build_widget(self, model, item, index, level, expanded):
        with ui.HStack(height=30, width=ui.Percent(100)):
            if item is None:
                ui.Label(model.get_name())
                return
            elif item.get_item("type") == "Part":
                with ui.ZStack(width=30):
                    self._rect = ui.Rectangle(
                        height=30,
                        width=30,
                        style={
                            "margin": ui.Pixel(1),
                            "background_color": 0x44FFFFFF,
                            "border_color": 0xFF222222,
                            "border_width": 0.5,
                            "border_radius": 3,
                            ":checked": {"background_color": 0x88000000},
                        },
                    )
                    # ui.ImageWithProvider(
                    #     item._byte_img_provider, height=30, width=30, style={"border_radius": 3, "margin": 3}
                    # )
                ui.Label(item.get_item("name"), width=ui.Percent(60))

            else:
                ui.Label("<<{}>> {}".format(item.get_item("type"), item.get_item("name")))
