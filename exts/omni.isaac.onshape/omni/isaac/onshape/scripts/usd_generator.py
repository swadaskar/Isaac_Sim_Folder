# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from numpy.core.einsumfunc import _greedy_path
from numpy.lib import ufunclike
import omni
import carb
from carb._carb import Float3, Float4
import pxr
from pxr import UsdShade, Sdf, Gf, Vt, UsdGeom, UsdLux, Usd, Kind, UsdPhysics, PhysxSchema
from pxr.Vt import IntArray, Vec3fArray, Vec2fArray, DoubleArray
import random
import os
import shutil
import re
import tempfile
import numpy as np
import time
import glob
import copy
import ctypes
import sys
import asyncio
import omni.client
from omni.client._omniclient import Result

# Transition between 104 and 105, deprecation of namespace omni.usd.utils
try:
    from omni.usd.utils import get_world_transform_matrix, get_local_transform_matrix
except:
    from omni.usd import get_world_transform_matrix, get_local_transform_matrix

from functools import partial

import base64
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import threading

from omni.isaac.onshape.widgets.color_name import ColorName
from omni.isaac.onshape.widgets.visual_materials_widget import VisualMaterial
from omni.isaac.onshape.widgets.assembly_widget import Mate
from omni.isaac.onshape.scripts.preferences import OnshapeImporterPreferences


import unicodedata
import re


def make_valid_filename(value):
    value = unicodedata.normalize("NFKD", str(value)).encode("ascii", "ignore").decode("ascii")
    value = re.sub(r"[^\w\s-]", "", value)
    value = re.sub(r"[-\s]+", "-", value).strip("-_")
    if not value:
        value = "missing_name"
    return pxr.Tf.MakeValidIdentifier(value)


def TraversePrim(prim, filterfn=None):
    """
    Extract all sub-childrens from a given prim, on a breadth-first search, cutting the search if a sub-prim does not
    match the filter function criteria
    """
    childrenStack = [prim]
    out = [prim] + prim.GetChildren()
    while len(childrenStack) > 0:
        prim = childrenStack.pop(0)
        if not filterfn or (filterfn and filterfn(prim)):
            children = prim.GetChildren()
            childrenStack = childrenStack + children
            out = out + children
    return out


def get_next_available(paths_dict, path):
    extension = ""

    if path not in paths_dict:
        paths_dict[path] = 1
    else:
        extension = "_{:02d}".format(paths_dict[path])
        paths_dict[path] += 1
    return path + extension


def get_next_free_path(stage, path, bool_not_used=False):
    base = path
    counter = 0
    while stage.GetPrimAtPath(path):
        counter += 1
        path = "{}_{:02}".format(base, counter)
    return path


def get_next_free_directory(base, path):
    counter = 0
    out = sanitize_slashes("{}/{}".format(base, path))
    result, _ = omni.client.list(out)

    while result != Result.ERROR_NOT_FOUND and counter < 200:
        counter += 1
        out = sanitize_slashes("{}/{}_{:02}".format(base, path, counter))
        result, _ = omni.client.list(out)
    return sanitize_slashes(out)


def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.isAlive():
        return

    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def bind_material(stage, prims, mat_path):
    material_prim = stage.GetPrimAtPath(mat_path)
    material = UsdShade.Material(material_prim)
    if type(prims) is not list:
        prims = [prims]
    for prim in prims:
        binding_api = UsdShade.MaterialBindingAPI(prim)
        binding_api.Bind(material)


def make_array(_type, a):
    if _type == "float":
        if a.shape[1] == 3:
            return Vec3fArray([Gf.Vec3f(float(a[i][0]), float(a[i][1]), float(a[i][2])) for i in range(a.shape[0])])
        if a.shape[1] == 2:
            return Vec2fArray([Gf.Vec2f(float(a[i][0]), float(a[i][1])) for i in range(a.shape[0])])
    elif _type == "int":
        al = a.flatten().tolist()
        return IntArray(al)


def createInMemoryStage(path, stage_unit):
    result, _, _ = omni.client.read_file(path)
    if result == Result.OK:
        stage = pxr.Usd.Stage.Open(path)
    else:
        stage = pxr.Usd.Stage.CreateNew(path)
        pxr.UsdGeom.SetStageUpAxis(stage, pxr.UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, stage_unit)
    return stage


class Transform:
    def __init__(self, p: Float3 = Float3(), r: Float4 = Float4()):
        self.p = p
        self.r = r


def convertColor(color_str):
    return bytearray(base64.b64decode(color_str))


def set_pose(prim, pose):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    r = Gf.Quatd(pose.r.w, pose.r.x, pose.r.y, pose.r.z)
    pos_vec = Gf.Vec3d(pose.p.x, pose.p.y, pose.p.z)
    xform_op_t.Set(pos_vec)
    xform_op_r.Set(r)


def set_pose_from_transform(prim, pose):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    t = Gf.Matrix4d(pose)
    r = t.ExtractRotationQuat()
    pos_vec = t.ExtractTranslation()
    xform_op_t.Set(pos_vec)
    xform_op_r.Set(r)


def create_joint_attributes(stage, joint, mate, limits, joint_pose, base_path, prim_path, body_0_global, body_1_global):
    rootLayer = stage.GetRootLayer()
    # with Usd.EditContext(stage, rootLayer):
    if 1:
        joint.CreateAxisAttr(mate.axis)
        # print(f.limits)
        if limits[0] is not None:
            joint.CreateLowerLimitAttr(limits[0])
        if limits[1] is not None:
            joint.CreateUpperLimitAttr(limits[1])

        joint.CreateBody0Rel().SetTargets([base_path])
        joint.CreateBody1Rel().SetTargets([prim_path])

        joint.CreateLocalPos0Attr().Set((joint_pose * body_0_global.GetInverse()).ExtractTranslation())
        joint.CreateLocalRot0Attr().Set(Gf.Quatf((joint_pose * body_0_global.GetInverse()).ExtractRotation().GetQuat()))

        joint.CreateLocalPos1Attr().Set((joint_pose * body_1_global.GetInverse()).ExtractTranslation())
        joint.CreateLocalRot1Attr().Set(Gf.Quatf((joint_pose * body_1_global.GetInverse()).ExtractRotation().GetQuat()))


def create_material(stage, _mtl_path, props):
    rootLayer = stage.GetRootLayer()
    ret = False
    with Usd.EditContext(stage, rootLayer):
        mat_prim = stage.GetPrimAtPath(Sdf.Path(_mtl_path))
        if not mat_prim:
            mat_prim = stage.DefinePrim(Sdf.Path(_mtl_path), "Material")
        material_prim = UsdShade.Material.Get(stage, mat_prim.GetPath())
        if material_prim:
            shader_path = stage.GetPrimAtPath(Sdf.Path("{}/Shader".format(_mtl_path)))
            if not shader_path:
                shader_path = stage.DefinePrim(Sdf.Path("{}/Shader".format(_mtl_path)), "Shader")
                shader_prim = UsdShade.Shader.Get(stage, shader_path.GetPath())
                if shader_prim:
                    with Sdf.ChangeBlock():
                        shader_out = shader_prim.CreateOutput("out", Sdf.ValueTypeNames.Token)
                        material_prim.CreateSurfaceOutput("mdl").ConnectToSource(shader_out)
                        material_prim.CreateVolumeOutput("mdl").ConnectToSource(shader_out)
                        material_prim.CreateDisplacementOutput("mdl").ConnectToSource(shader_out)
                        shader_prim.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
                        shader_prim.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
                        shader_prim.SetSourceAssetSubIdentifier("OmniPBR", "mdl")

                        omni.usd.create_material_input(
                            mat_prim,
                            "diffuse_color_constant",
                            Gf.Vec3f(props.color.r, props.color.g, props.color.b),
                            Sdf.ValueTypeNames.Color3f,
                        )
                        omni.usd.create_material_input(
                            mat_prim,
                            "emissive_color",
                            Gf.Vec3f(props.emissive.r, props.emissive.g, props.emissive.b),
                            Sdf.ValueTypeNames.Color3f,
                        )
                        omni.usd.create_material_input(
                            mat_prim, "metallic_constant", props.metallic, Sdf.ValueTypeNames.Float
                        )
                        omni.usd.create_material_input(
                            mat_prim, "reflection_roughness_constant", props.roughness, Sdf.ValueTypeNames.Float
                        )
                        omni.usd.create_material_input(
                            mat_prim, "enable_emission", props.emissive.a > 0, Sdf.ValueTypeNames.Bool
                        )
                        omni.usd.create_material_input(
                            mat_prim, "emissive_intensity", props.emissive.a, Sdf.ValueTypeNames.Float
                        )
                    # mat_prim.SetInstanceable(True)
            ret = mat_prim.GetPath().pathString
        else:
            carb.log_error(f"failed to create prim {mat_prim.GetPath().pathString}")
    return ret


def get_prim_and_material_binding(parent_prim, solid_name):
    return [
        (p.GetPath().pathString[len(solid_name) :], p.GetRelationship("material:binding").GetTargets())
        for p in TraversePrim(parent_prim)
        if UsdGeom.Mesh(p) or UsdGeom.Subset(p)
    ]


def get_material_path(prim):
    result = prim.GetRelationship("material:binding").GetTargets()
    return str(result[0]) if len(result) > 0 else ""


def get_all_prims_with_material(stage, material_name, prim=None):
    if prim:
        return [x for x in TraversePrim(prim) if material_name == os.path.basename(get_material_path(x))]
    return [x for x in stage.Traverse() if material_name == os.path.basename(get_material_path(x))]


DEFAULT_TEMP_FOLDER_SETTING = "/ext/omni.isaac.onshape_importer/default_temp"


class PartItem:
    def __init__(self, base_path, part, stage_unit, material_stage):
        name = make_valid_filename("{}_{}".format(part.get_name(), part.get_encoded_part_id()))
        self.path = sanitize_slashes("{}/{}".format(base_path, "{}.usd".format(name)))
        self.part = part
        self.name = make_valid_filename(part.get_name())
        result, _, _ = omni.client.read_file(self.path)
        if result == Result.OK:
            i = 1
            while result == Result.OK:
                self.path = sanitize_slashes("{}/{}".format(base_path, "{}_{}.usd".format(name, i)))
                i = i + 1
                result, _, _ = omni.client.read_file(self.path)
        self.stage = createInMemoryStage(self.path, stage_unit)
        root = UsdGeom.Xform.Define(self.stage, "/World").GetPrim()
        self.stage.SetDefaultPrim(root)
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(300)
        light_pose = Transform(r=Float4(-0.383, 0, 0, 0.924))
        set_pose(distantLight, light_pose)
        # self.stage.Save()
        # rootLayer = self.stage.GetRootLayer()
        # mat_layer = os.path.relpath(material_stage.GetRootLayer().identifier, base_path).replace(
        #     "\\", "/"
        # )
        # if mat_layer not in rootLayer.subLayerPaths:
        #     rootLayer.subLayerPaths.append(mat_layer)
        # self.stage.Save()
        self.imported = False
        self.pending_payload = []
        self.payload = False
        # print(self.path)


def sanitize_slashes(s):
    """
    Makes path/slashes uniform

    Args:
        path: path
        is_directory is path a directory, so final slash can be added

    Returns:
        path
    """
    path = os.path.normpath(s)
    path = path.replace(":/", "://")
    return path.replace("\\", "/")


class UsdGenerator:
    def __init__(self, document, assembly, stage_unit=0.01, mesh_imported_fn=None, assembly_done_fn=None):
        preferences = OnshapeImporterPreferences()
        self.assembly_stage = None
        self._assembly_edit_stage = None
        self._material_stage = None
        self.temp_stage = None
        # Dictionary of materials key: color string from the source, value: usd path in the material stage
        self._materials_dict = {}
        # Dictionary of parts and stage; key: part.get_key(), stage: stage that
        self._parts_stage_dict = {}
        self.loop = asyncio.get_event_loop()
        self.document = document
        self.assembly = assembly
        self.assembly_task = None
        self.stage_unit = stage_unit
        # define the work directory for the document usds
        self.tempdir = preferences.get_working_folder()
        self.tempdir = get_next_free_directory(self.tempdir, document.get_name())
        omni.client.create_folder(self.tempdir)
        # print(self.tempdir)
        # Materials USD, where all materials for the assembly will be stored.
        self._materials_path = sanitize_slashes("{}/{}".format(self.tempdir, "materials"))
        self.assemblies_path = {}
        self.groupMates = {}
        self.group_map = {}
        self.instance_group_map = {}
        self.override_parent = {}
        self.rigid_bodies = set()
        self.pending_payloads = []
        self.finalizing_meshes = []
        self.first_import = True
        self.mesh_imported_fn = mesh_imported_fn
        self.assembly_done_fn = assembly_done_fn
        self.rig_physics = True
        omni.client.create_folder(self._materials_path)
        # print(self._materials_path)
        self._materials_path = "{}/{}".format(self._materials_path, "materials.usd")
        self._pending_assembly_notify = False

        self._stages_dir = "{}/{}".format(self.tempdir, "parts")
        omni.client.create_folder(self._stages_dir)
        self.parts_building_pool = ThreadPoolExecutor(max_workers=40, thread_name_prefix="onshape_parts_building_pool")
        # Assemblies need parts stage to be fully built

        self.stage_path = None
        self.assembly_building_pool = ThreadPoolExecutor(
            max_workers=10, thread_name_prefix="onshape_assembly_building_pool"
        )
        self.assembly_children_stack = {}

        # self.manager = multiprocessing.Manager()
        self.assembly_lock = threading.Lock()
        self.materials_update_lock = threading.Lock()
        self.part_stage_lock = threading.Lock()
        self.stage_lock = threading.Lock()
        # self.main_thread = threading.currentThread()

        # print(self.main_thread.getName())
        self.shutdown = False
        self.delayed_make_assembly = False
        self._app_update_sub = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_ui)
        )

        self._selection = omni.usd.get_context().get_selection()
        self._events = omni.usd.get_context().get_stage_event_stream()
        # omni.usd.get_context().disable_save_to_recent_files()
        self.parts_to_process = []
        self.parts_to_process_post = []
        self.parts_pending_mass = []

    def process_part_mass(self, stage_identifier, prim_path, mass_props):
        result, _, file_content = omni.client.read_file(stage_identifier)
        if result == Result.OK:
            stage = Usd.Stage.Open(stage_identifier)
            if stage:
                # with Usd.EditContext(stage, rootLayer):
                # with self.part_stage_lock:
                mesh_prim = stage.GetPrimAtPath(prim_path)
                stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
                if mass_props:
                    com = [0.0, 0.0, 0.0]
                    if "centroid" in mass_props and mass_props["centroid"]:
                        com = [float(mass_props["centroid"][i]) / stage_unit for i in [0, 1, 2]]

                    # print(com)
                    massAPI = None
                    # print(mesh_name, "setting mass props")
                    massAPI = UsdPhysics.MassAPI.Apply(mesh_prim)
                    massAPI.CreateCenterOfMassAttr(Gf.Vec3d(com[0], com[1], com[2]))
                    if mass_props and "mass" in mass_props and mass_props["mass"]:
                        massAPI.CreateMassAttr(mass_props["mass"][0])
                        if mass_props and "inertia" in mass_props:
                            inertia_matrix = mass_props["inertia"]
                            mesh_prim.CreateAttribute("inertiaMatrix", Sdf.ValueTypeNames.DoubleArray, False).Set(
                                DoubleArray([float(i) * (1.0 / stage_unit) ** 2 for i in inertia_matrix[0:9]])
                            )
                        if mass_props and "principalInertia" in mass_props:
                            diag_inertia = mass_props["principalInertia"]
                            if not massAPI:
                                massAPI = UsdPhysics.MassAPI.Apply(mesh_prim)
                            massAPI.CreateDiagonalInertiaAttr(
                                Gf.Vec3d([float(i) * (1.0 / stage_unit) ** 2 for i in diag_inertia])
                            )
                    stage.Save()

    def is_temp_stage_open(self):
        return True

    def delete_folder(self):

        try:
            result, _ = omni.client.list(self.tempdir)
            if result == Result.OK:
                omni.client.delete(self.tempdir)
        except Exception as e:
            carb.log_error("Error trying to clean temp folder: " + str(e))

    def __del__(self):
        self.on_shutdown()

    def on_shutdown(self):
        # print("shutting down")
        self._stage_event_subscription = None
        self._app_update_sub = None
        del self._material_stage
        del self.assembly_stage
        del self._assembly_edit_stage
        del self.temp_stage
        self.assembly_stage = None
        self._assembly_edit_stage = None
        self._material_stage = None
        self.temp_stage = None
        self.assembly_task = None
        self._parts_stage_dict = {}
        if not self.shutdown:
            self.shutdown = True
            self.parts_building_pool.shutdown(wait=True)
            for t in self.parts_building_pool._threads:
                terminate_thread(t)
            # if self.materials_update_lock.locked():
            #     self.materials_update_lock.release()
            self.materials_update_lock = None
            # if self.part_stage_lock.locked():
            #     self.part_stage_lock.release()
            self.part_stage_lock = None

    def finished_meshes(self):

        for part in self._parts_stage_dict.values():
            if not part.imported:
                return False
        return True

    def get_material(self, material_key):
        """
        Get material reference according to the key
        """

        if material_key not in self._materials_dict:
            mat = VisualMaterial(convertColor(material_key))
            name = "/World/Looks/" + pxr.Tf.MakeValidIdentifier("{}".format(mat.name))
            # print ("getting material", self._materials_path, name)
            # with self.materials_update_lock:
            usd_mat = create_material(self.material_stage, name, mat)
            if usd_mat:
                self._materials_dict[material_key] = {"usd": usd_mat, "name": name}
            ###
        return self._materials_dict[material_key]

    def update_material(self, material_key, new_mat: VisualMaterial):
        name = self.get_material(material_key)
        new_name = "/World/Looks/" + pxr.Tf.MakeValidIdentifier("{}".format(new_mat.name))
        # with self.materials_update_lock:
        if new_name != name:
            move_dict = {name: new_name}
            omni.kit.commands.execute("MovePrimsCommand", paths_to_move=move_dict, on_move_fn=None)

        usd_mat = create_material(self.material_stage, name, new_mat)
        if usd_mat:
            self._materials_dict[material_key] = {"usd": usd_mat, "name": new_name}
        return self._materials_dict[material_key]

    def create_part_stage(self, part):
        # print(part.get_name(), "Done importing, starting USD conversion")
        self.loop.create_task(self.set_part_mesh(part))
        # task = self.parts_building_pool.submit(self.set_part_mesh, part)

    @property
    def material_stage(self):
        if not self._material_stage:
            with self.materials_update_lock:
                self._material_stage = createInMemoryStage(self._materials_path, self.stage_unit)
                root = UsdGeom.Xform.Define(self._material_stage, "/World").GetPrim()
                looks_prim = self._material_stage.DefinePrim(Sdf.Path("/World/Looks"), "Scope")
                self._material_stage.SetDefaultPrim(root)
                self._material_stage.Save()
        return self._material_stage

    def create_all_stages(self, parts):
        with self.part_stage_lock:
            for part in parts:
                if part.get_key() not in self._parts_stage_dict:
                    # print(part.get_name(), " part Lock")
                    self._parts_stage_dict[part.get_key()] = PartItem(
                        self._stages_dir, part, self.stage_unit, self.material_stage
                    )
        self.material_stage.GetDefaultPrim()

    async def set_part_mesh(self, part, sync=False, mesh_data=None):
        if self.shutdown:
            return
        if not sync and not self._parts_stage_dict[part.get_key()].payload:
            self.set_payload_mesh(part)
        with self.part_stage_lock:
            self.parts_to_process.append((part, []))
            self._parts_stage_dict[part.get_key()].imported = True

    def set_payload_mesh(self, part):
        stage = self._parts_stage_dict[part.get_key()].stage
        stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
        rootLayer = stage.GetRootLayer()
        Vertex = make_array("float", part.get_mesh().vertices / stage_unit)
        face_vertex_count = make_array("int", part.get_mesh().face_vertex_count)
        face_indices = make_array("int", part.get_mesh().face_indices)
        face_indices_uvs = make_array("float", part.get_mesh().vertices_UVs)
        face_indices_normals = make_array("float", part.get_mesh().vertices_normals)
        path = self._parts_stage_dict[part.get_key()].path
        # if self.assembly_stage:
        # with Sdf.ChangeBlock():
        with self.part_stage_lock:

            # return
            # Vertex = mesh_data[0]
            # face_vertex_count = mesh_data[1]
            # face_indices = mesh_data[2]
            # face_indices_uvs = mesh_data[3]
            # face_indices_normals = mesh_data[4]

            # with Sdf.ChangeBlock():
            if part.get_key() not in self._parts_stage_dict:
                carb.log_error("Mesh USD Generation error: Stage was not previously created" + part.get_name())
                return

            self._parts_stage_dict[part.get_key()].imported = False

            self._parts_stage_dict[part.get_key()].name = pxr.Tf.MakeValidIdentifier(
                make_valid_filename(part.get_name().strip())
            )
        name = self._parts_stage_dict[part.get_key()].name
        mesh_name = "/World/{}".format(name)

        try:
            rootLayer = stage.GetRootLayer()
            rootLayer.SetPermissionToEdit(True)
            # with Usd.EditContext(stage, rootLayer):
            if 1:
                UsdGeom.Xform.Define(stage, "/World").GetPrim()
                xform = UsdGeom.Xform.Define(stage, "/World/{}".format(name)).GetPrim()
            # print(part.get_name(), "creating mesh", mesh_name)
            # with Sdf.ChangeBlock():

            usdMesh = UsdGeom.Mesh.Define(stage, Sdf.Path(mesh_name))
            mesh_prim = stage.GetPrimAtPath(Sdf.Path(mesh_name))
            model_api = Usd.ModelAPI(mesh_prim)
            model_api.SetKind(Kind.Tokens.model)
            # with self.materials_update_lock:
            # print(mesh_name, "setting COM")
            if self.rig_physics:

                def set_mass(stage, mesh_prim, mass_props):
                    self.parts_pending_mass.append([stage, mesh_prim, mass_props])

                part.set_on_mass_props_changed(partial(set_mass, stage.GetRootLayer().identifier, mesh_prim.GetPath()))
                part.get_mass_properties_async()
                # TODO: Add Density based on selected material

            usdMesh.CreatePointsAttr(Vertex)
            usdMesh.CreateNormalsAttr(face_indices_normals)
            usdMesh.CreateFaceVertexCountsAttr(face_vertex_count)
            usdMesh.CreateFaceVertexIndicesAttr(face_indices)

            usdMesh.SetNormalsInterpolation(pxr.UsdGeom.Tokens.faceVarying)
            texCoord = usdMesh.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
            texCoord.Set(face_indices_uvs)
            usdMesh.CreateSubdivisionSchemeAttr("none")
            try:
                for i, material in enumerate(part.get_mesh().colors):
                    mat = VisualMaterial(convertColor(material))
                    if len(part.get_mesh().colors) > 1:
                        face_indices = part.get_mesh().facets_per_color[i]
                        subset_name = "{}/{}".format(mesh_name, pxr.Tf.MakeValidIdentifier(mat.name.split("/")[-1]))
                        geomSubset = UsdGeom.Subset.Define(stage, subset_name)
                        geomSubset.CreateElementTypeAttr("face")
            except Exception as e:
                carb.log_error(str(e))
        except Exception as e:
            carb.log_error(str(e))
            carb.log_error(str(e.with_traceback()))
        stage.Save()

    async def finalize_mesh(self, part):
        stage = self._parts_stage_dict[part.get_key()].stage
        rootLayer = stage.GetRootLayer()
        mat_layer = os.path.relpath(self.material_stage.GetRootLayer().identifier, self._stages_dir).replace("\\", "/")
        if mat_layer not in rootLayer.subLayerPaths:
            rootLayer.subLayerPaths.append(mat_layer)
        try:
            with Sdf.ChangeBlock():
                usdMesh = UsdGeom.Mesh([a for a in stage.Traverse() if UsdGeom.Mesh(a)][0])
                mesh_name = str(usdMesh.GetPrim().GetPath())
                for i, material in enumerate(part.get_mesh().colors):
                    mat = self.get_material(material)
                    if len(part.get_mesh().colors) > 1:
                        face_indices = part.get_mesh().facets_per_color[i]
                        subset_name = "{}/{}".format(mesh_name, pxr.Tf.MakeValidIdentifier(mat["name"].split("/")[-1]))
                        # print(subset_name)
                        geomSubset = UsdGeom.Subset(stage.GetPrimAtPath(subset_name))
                        # geomSubset.CreateElementTypeAttr("face")
                        geomSubset.CreateFamilyNameAttr("materialBind")
                        geomSubset.CreateIndicesAttr(IntArray(face_indices))
                        # geomSubset.GetPrim().SetInstanceable(True)
                        bind_material(stage, geomSubset, "{}".format(mat["usd"]))
                    else:
                        bind_material(stage, usdMesh, "{}".format(mat["usd"]))
                        # usdMesh.GetPrim().SetInstanceable(True)

                # materials.SetInstanceable(True)
            self._parts_stage_dict[part.get_key()].stage.Save()
        except Exception as e:
            carb.log_error("Mesh USD Generation error: " + str(e))
        # with Sdf.ChangeBlock():
        payloads = (
            os.path.relpath(self._parts_stage_dict[part.get_key()].path, self.tempdir).replace("\\", "/"),
            [p for p in self._parts_stage_dict[part.get_key()].pending_payload],
        )
        self._parts_stage_dict[part.get_key()].pending_payload = []
        with self.part_stage_lock:
            self.pending_payloads.append((payloads))
            # p.GetPayloads().AddPayload(
            #                 os.path.relpath(self._parts_stage_dict[part.get_key()].path, self.tempdir).replace("\\", "/"),
            #                 # solid_name,
            #             )
        # self.assembly_stage.Save()

    def reset_assembly(self):
        # if self.is_temp_stage_open():
        #     omni.usd.get_context().close_stage_with_callback(lambda a,b: omni.usd.get_context().new_stage_with_callback(lambda a,b: self.reset_assembly()))
        # else:
        #     if self.stage_path:
        self.stage_path = None

        # self.build_assemblies()

    def build_assemblies_sync(self):
        if not self.assembly_task:
            self.build_assemblies()
        self.assembly_task.join()
        self.assembly_task = None
        self.delayed_make_assembly = False
        self._delayed_make_assembly()
        self.assembly_notify()

    def build_assemblies(self, task=None):
        def build_assembly():
            path = "/World/{}".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(self.assembly.get_name())))
            self.groupMates = {}
            self.group_map = {}
            self.assembly_children_stack = {}
            self.assemblies_path = {}
            self.assemblies_path[""] = path
            self.assemblies_path_dict = {}
            self.assemblies_path_dict[path] = 1
            self.rigid_bodies = set()
            # print("get assembly paths")
            for a in self.assembly._root.get_children():
                self.get_assembly_paths(a, path, "")
            if self.rig_physics:
                self.assembly.assembly_features_sync()
                self.create_group_mates(self.assembly._root)
                self.process_fastened_mates(self.assembly._root)
            self.delayed_make_assembly = True

        self.assembly_task = threading.Thread(target=build_assembly)
        self.assembly_task.start()

    def get_assembly_paths(self, assembly, parent_path, parent_id):
        # if not assembly.get_item("suppressed"):
        name = assembly.get_item("name").split(" <")[0].strip()
        a_id = parent_id + assembly.get_item("id")
        path = parent_path + "/{}".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(name.strip())))
        path = get_next_available(self.assemblies_path_dict, path)
        self.assemblies_path[a_id] = path
        # print(assembly.get_item("name"), name, path, len(assembly.get_children()))
        for a in assembly.get_children():
            # if not a.get_item("suppressed"):
            self.get_assembly_paths(a, path, a_id)

    def write_assembly_xform(self, stage, assembly, parent_id, in_group=None, make_collision=False):
        # print(level, assembly   )
        # print(assembly.transform)
        stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
        name = assembly.get_item("name").strip()[:-4]
        a_id = parent_id + assembly.get_item("id")
        path = self.assemblies_path[a_id]
        if self.rig_physics:
            self.make_groups_xform(stage, assembly, path, a_id)
        # print(assembly.get_item("id"), path)
        parent_prim = stage.GetPrimAtPath(Sdf.Path(path).GetParentPath())
        if parent_prim:
            parent_global_pose = get_world_transform_matrix(parent_prim)
            if parent_prim.IsInstanceable():
                source = Sdf.Path(parent_prim.GetPath())
                basename = source.name
                temp_path = source.AppendChild(basename + "_temp")
                dest = os.path.join(source, os.path.basename(source))
                xform = UsdGeom.Xform.Define(stage, temp_path)
                new_prim = xform.GetPrim()
                xform.ClearXformOpOrder()
                # xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
                local_t = get_local_transform_matrix(parent_prim)
                set_pose_from_transform(new_prim, local_t)
                # xform_op.Set(local_t)
                edit = Sdf.BatchNamespaceEdit()
                edit.Add(Sdf.NamespaceEdit.Reparent(parent_prim.GetPath(), new_prim.GetPath(), 0))
                # edit.Add(Sdf.NamespaceEdit.Rename(new_prim.GetPath(), basename))
                stage.GetRootLayer().Apply(edit)
                edit = Sdf.BatchNamespaceEdit()
                # edit.Add(Sdf.NamespaceEdit.Reparent(parent_prim.GetPath(),new_prim.GetPath(),0))
                edit.Add(Sdf.NamespaceEdit.Rename(new_prim.GetPath().pathString, basename))
                xform = UsdGeom.Xformable(stage.GetPrimAtPath(source.AppendChild(basename)))
                xform.ClearXformOpOrder()
            t = np.array(assembly.transform[a_id]).reshape((4, 4))
            t = np.transpose(t)
            gf_m = Gf.Matrix4d(*t.reshape(16).tolist())
            gf_m.SetTranslateOnly(gf_m.ExtractTranslation() / stage_unit)
            local_t = gf_m * parent_global_pose.GetInverse()
        else:
            child = (assembly, parent_id, in_group)
            p = Sdf.Path(path)
            parent_path = p.GetParentPath().pathString
            if parent_path in self.assembly_children_stack:
                self.assembly_children_stack[parent_path].append(child)
            else:
                self.assembly_children_stack[parent_path] = [child]
            return

        prim = stage.GetPrimAtPath(path)
        if assembly.get_item("suppressed") and prim:
            stage.RemovePrim(path)
            return

        try:
            if prim and assembly.get_item("type") == "Part":
                if prim.GetChildren():
                    # print("{} moved to {}".format(path, path + "/{}".format(make_valid_filename(name))))
                    path = get_next_free_path(stage, path + "/{}".format(make_valid_filename(name)), False)
                    self.assemblies_path[a_id] = path
                    prim = stage.GetPrimAtPath(path)
            if not prim:
                if assembly.get_item("type") == "Part":
                    part_id = assembly.get_item("hashId")
                    if part_id in self._parts_stage_dict:
                        UsdGeom.Xform.Define(stage, path)
                        prim = stage.OverridePrim(path)
                        if self._parts_stage_dict[part_id].imported:
                            prim.GetPayloads().AddPayload(
                                os.path.relpath(self._parts_stage_dict[part_id].path, self.tempdir).replace("\\", "/"),
                                # solid_name,
                            )
                        else:
                            self._parts_stage_dict[part_id].pending_payload.append(prim.GetPath())
                        prim.SetInstanceable(True)

                        self._parts_stage_dict[part_id].part.add_usd_path(path)
                    else:
                        carb.log_error(
                            "Part Not Found, Please re-import the assembly to load it: {} ({})".format(name, path)
                        )
                        return
                else:
                    prim = UsdGeom.Xform.Define(stage, path).GetPrim()

            # print(prim)
            # print(level*" ",prim.GetPath())
            child_prims = TraversePrim(prim)
            child_transforms = [get_world_transform_matrix(c) for c in child_prims]

            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            # xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
            # xform_op.Set(local_t)
            set_pose_from_transform(prim, local_t)

            if assembly.get_item("hidden"):
                UsdGeom.Imageable(prim).MakeInvisible()
        except Exception as e:
            carb.log_error("{}: {}".format(str(e), path))

        # print(assembly.get_item("name"), path)
        # for a in assembly.get_children():
        # print ("   ", a.get_item("name"))

        if path in self.assembly_children_stack:
            for a, b, c in self.assembly_children_stack[path]:
                self.write_assembly_xform(stage, a, b, c)
            self.assembly_children_stack.pop(path)
        for a in assembly.get_children():
            if not a.get_item("suppressed"):
                c_id = a.get_item("id")
                p_path = path
                p_in_group = in_group
                if a_id + c_id in self.group_map:
                    p_in_group = self.group_map[a_id + c_id]
                    p_path = self.groupMates[p_in_group]["prim"]
                self.write_assembly_xform(stage, a, a_id, in_group=p_in_group)

    def make_groups_xform(self, stage, assembly, path, a_id=""):
        if self.rig_physics and not assembly.get_item("suppressed") and assembly.uid in self.assembly.features_map:
            # a_id = a_id + (assembly.get_item("id") or '')
            for f_id in self.assembly.features_map[assembly.uid]:
                feature = self.assembly.assembly_features[f_id]
                f_id = feature["id"]
                if feature["featureType"] == "mateGroup" and not feature["suppressed"]:
                    if (a_id, f_id) in self.groupMates:
                        # Create Prim for group
                        group = self.groupMates[(a_id, f_id)]
                        group_path = group["prim"]
                        # print("Before", group_path)
                        if not group_path:
                            group_path = path + "/{}".format(
                                pxr.Tf.MakeValidIdentifier(make_valid_filename(feature["featureData"]["name"].strip()))
                            )
                            group_path = get_next_free_path(stage, group_path, False)
                            self.groupMates[(a_id, f_id)]["prim"] = group_path
                        g_prim = stage.GetPrimAtPath(group_path)
                        if not g_prim:
                            sdf_path = Sdf.Path(group_path)
                            pending_parents = []
                            parent = sdf_path.GetParentPath()
                            while not (stage.GetPrimAtPath(parent)):
                                pending_parents.append(parent)
                                parent = parent.GetParentPath()
                            for i in range(len(pending_parents) - 1, -1, -1):
                                UsdGeom.Xform.Define(stage, pending_parents[i])
                            g_prim = UsdGeom.Xform.Define(stage, group_path).GetPrim()
                            xform = UsdGeom.Xformable(g_prim)
                            xform.ClearXformOpOrder()
                            set_pose_from_transform(g_prim, Gf.Matrix4d())
                            # UsdPhysics.RigidBodyAPI.Apply(g_prim)
                    else:
                        carb.log_error("group mate not found {}".format((a_id, f_id)))

    def get_sub_joints(self, assembly):
        features = []
        if assembly.uid in self.assembly.features_map:
            features = [
                f
                for f in self.assembly.features_map[assembly.uid]
                if f in self.assembly.assembly_features
                and f in self.assembly.features_details
                and self.assembly.assembly_features[f]["featureType"] == "mate"
                and self.assembly.assembly_features[f]["suppressed"] == False
                and (
                    self.assembly.assembly_features[f]["featureData"]["mateType"]
                    in ["REVOLUTE", "SLIDER", "CYLINDRICAL", "BALL"]
                    and not Mate(self.assembly.assembly_features[f], self.assembly.features_details[f]).is_locked()
                )
            ]
            for f in features:
                mate = Mate(self.assembly.assembly_features[f], self.assembly.features_details[f])
                if mate.limits[0] == mate.limits[1] and mate.limits[0] is not None:
                    features.remove(f)
        for a in assembly.get_children():
            features += self.get_sub_joints(a)
        return features

    def create_group_mates(self, assembly, a_id="", in_group=None):
        if self.rig_physics:
            a_id = a_id + str(assembly.get_item("id") or "")
            self.instance_group_map[a_id] = in_group
            if (
                not assembly.get_item("suppressed")
                and assembly.get_item("type").lower() == "assembly"
                and assembly.uid in self.assembly.features_map
            ):
                gms = [
                    f
                    for f in self.assembly.features_map[assembly.uid]
                    if self.assembly.assembly_features[f]["featureType"] == "mateGroup"
                    and not self.assembly.assembly_features[f]["suppressed"]
                ]
                # if gms and a_id in self.assemblies_path:

                for f_id in gms:
                    feature = self.assembly.assembly_features[f_id]
                    f_id = feature["id"]

                    instances = [o["occurrence"][0] for o in feature["featureData"]["occurrences"]]
                    if (a_id, f_id) not in self.groupMates:
                        prim = self.assemblies_path[a_id] + "/{}".format(
                            pxr.Tf.MakeValidIdentifier(make_valid_filename(feature["featureData"]["name"].strip()))
                        )
                        prim = get_next_available(self.assemblies_path_dict, prim)
                        groupMate = {"name": feature["featureData"]["name"], "instances": instances, "prim": prim}
                        # print((a_id, f_id), feature["featureData"]["name"])
                        # print("  ", groupMate["prim"])
                        for i in instances:
                            self.group_map[a_id + i] = (a_id, f_id)
                        # print("   ", assembly.get_item("name"), groupMate["name"], groupMate["prim"])
                        self.groupMates[(a_id, f_id)] = groupMate
            # Find Instances of assembly that are part of each mate
            for a in assembly.get_children():
                p_in_group = in_group
                if a_id + a.get_item("id") in self.group_map and not p_in_group:
                    p_in_group = self.group_map[a_id + a.get_item("id")]
                    self.get_assembly_paths(a, self.groupMates[p_in_group]["prim"], a_id)
                self.create_group_mates(a, a_id, p_in_group)

    def process_fastened_mates(self, assembly, a_id="", in_group=None):
        if self.rig_physics:
            _a_id = a_id
            a_id = a_id + str(assembly.get_item("id") or "")
            if (
                not assembly.get_item("suppressed")
                and assembly.get_item("type").lower() == "assembly"
                and assembly.uid in self.assembly.features_map
            ):
                for f_id in [
                    f
                    for f in self.assembly.features_map[assembly.uid]
                    if f in self.assembly.assembly_features
                    and f in self.assembly.features_details
                    and self.assembly.assembly_features[f]["featureType"] == "mate"
                    and self.assembly.assembly_features[f]["suppressed"] == False
                    and (
                        self.assembly.assembly_features[f]["featureData"]["mateType"] == "FASTENED"
                        or Mate(self.assembly.assembly_features[f], self.assembly.features_details[f]).is_locked()
                    )
                ]:
                    feature = self.assembly.assembly_features[f_id]
                    # print(assembly.get_item("name"), feature["featureData"]["name"], feature["suppressed"])
                    f_id = feature["id"]
                    # f = Mate(self.assembly.assembly_features[f_id], self.assembly.features_details[f_id])
                    mateds = [m["matedOccurrence"] for m in feature["featureData"]["matedEntities"]]
                    base = None
                    p = None
                    if len(mateds) >= 2:
                        base, p = mateds
                    if base and p:
                        if self.instance_group_map[a_id + "".join(base)]:
                            base_path = self.groupMates[self.instance_group_map[a_id + "".join(base)]]["prim"]
                        else:
                            base_id = max(0, len(base) - 2)  # start one before last
                            # print("    Find base parent assembly")
                            inst = self.assembly._instances_flat[base[base_id]]
                            while not self.get_sub_joints(
                                inst
                            ):  # inst.uid not in self.assembly.features_map or not self.assembly.features_map[inst.uid]:
                                # print("    ", self.assemblies_path[a_id+''.join(base[:base_id])])
                                base_id -= 1
                                if base_id < 0:
                                    break
                                inst = self.assembly._instances_flat[base[base_id]]
                            # print(base_id)
                            base_id = min(len(base) - 1, base_id + 1)
                            b_id = "".join(base[: base_id + 1])
                            # print(f.name, base_id, a_id, b_id)
                            base_path = self.assemblies_path[a_id + b_id]
                            if (
                                self.assembly._instances_flat[base[base_id]].get_item("type").lower() == "part"
                            ):  # selected the part itself
                                self.assemblies_path[a_id + b_id] = base_path + "/{}".format(
                                    os.path.basename(base_path)
                                )
                                # base_path = os.path.dirname(base_path)
                            # print("  ", len(base), base_id+1)
                        # print("  ", feature["featureData"]["name"], base_path)
                        rb_path = [i for i in self.rigid_bodies if Sdf.Path(base_path).HasPrefix(i)]
                        if rb_path:
                            a = [i for i in self.rigid_bodies if Sdf.Path(i).HasPrefix(base_path) and i != base_path]
                            for _p in a:
                                self.rigid_bodies.remove(_p)
                            rb_path = "".join(rb_path)
                        else:
                            self.rigid_bodies.add(base_path)
                            rb_path = base_path
                        base_path = rb_path
                        prim_path = self.assemblies_path[a_id + "".join(p)]
                        # print("   ",rb_path, prim_path)
                        if rb_path not in prim_path:
                            # print(self.instance_group_map[a_id + "".join(p)])
                            if self.instance_group_map[a_id + "".join(p)]:

                                prim_path = self.groupMates[self.instance_group_map[a_id + "".join(p)]]["prim"]
                                # print(self.instance_group_map[a_id + "".join(base)]
                                # != self.instance_group_map[a_id + "".join(p)],  self.instance_group_map[a_id + "".join(base)],
                                # self.instance_group_map[a_id + "".join(p)])
                                if (
                                    self.instance_group_map[a_id + "".join(base)]
                                    != self.instance_group_map[a_id + "".join(p)]
                                ):
                                    prim_path = get_next_available(
                                        self.assemblies_path_dict, base_path + "/{}".format(os.path.basename(prim_path))
                                    )
                                    # print("updated prim_path", prim_path)
                                    self.groupMates[self.instance_group_map[a_id + "".join(p)]]["prim"] = prim_path
                                else:
                                    prim_path = base_path
                                # for a in assembly.get_children():
                                #     if self.instance_group_map[a_id + a.get_item("id")] == self.instance_group_map[a_id+''.join(p)]:
                                #         self.get_assembly_paths(a,prim_path,a_id)

                            else:

                                p_id = max(0, len(p) - 2)  # start one before last

                                # print("    Find child parent assembly")
                                inst = self.assembly._instances_flat[p[p_id]]
                                while not self.get_sub_joints(
                                    inst
                                ):  # inst.uid not in self.assembly.features_map or not self.assembly.features_map[inst.uid]:
                                    # print("    ", self.assemblies_path[a_id+''.join(p[:p_id])])
                                    p_id -= 1
                                    if p_id < 0:
                                        break
                                    inst = self.assembly._instances_flat[p[p_id]]
                                # print(p_id)
                                p_id = min(len(p) - 1, p_id + 1)
                                _p_id = "".join(p[: p_id + 1])
                                # print("   ", self.assemblies_path[a_id + "".join(_p_id)])
                                prim_path = get_next_available(
                                    self.assemblies_path_dict,
                                    base_path + "/{}".format(os.path.basename(self.assemblies_path[a_id + _p_id])),
                                )
                                # print("  ", prim_path)
                                if self.instance_group_map[a_id + "".join(base)]:
                                    self.instance_group_map[a_id + "".join(p)] = self.instance_group_map[
                                        a_id + "".join(base)
                                    ]
                                self.assemblies_path[a_id + _p_id] = prim_path
                                for a in self.assembly._instances_flat[p[p_id]].get_children():
                                    self.get_assembly_paths(a, prim_path, a_id + _p_id)
                        if not [i for i in self.rigid_bodies if Sdf.Path(base_path).HasPrefix(i)]:
                            a = [i for i in self.rigid_bodies if Sdf.Path(i).HasPrefix(base_path) and i != base_path]
                            for p in a:
                                self.rigid_bodies.remove(p)
                            self.rigid_bodies.add(
                                base_path
                            )  # Add this body to a set to later add the RBAPI, and set the collisions for the underlying meshes
                        # print("  ", feature["featureData"]["name"], prim_path)
            ## Regenerate all assemblies paths if any group mate was created
            # Find Instances of assembly that are part of each mate
            # print("Done", a_id)
            # print("redoing group mates", _a_id)
            self.create_group_mates(assembly, _a_id, in_group)
            for a in assembly.get_children():
                p_in_group = in_group
                if a_id + a.get_item("id") in self.group_map:
                    p_in_group = self.group_map[a_id + a.get_item("id")]
                self.process_fastened_mates(a, a_id, p_in_group)

    def get_rigid_body_path(self, a_id, occurence):
        if self.instance_group_map[a_id + "".join(occurence)]:
            path = self.groupMates[self.instance_group_map[a_id + "".join(occurence)]]["prim"]
            # print("in group:", path)
            # print(self.rigid_bodies)
        else:
            _id = max(0, len(occurence) - 1)
            inst = self.assembly._instances_flat[occurence[_id - 1]]
            while not self.get_sub_joints(
                inst
            ):  # inst.uid not in self.assembly.features_map or not self.assembly.features_map[inst.uid]:
                _id -= 1
                if _id < 0:
                    break
                inst = self.assembly._instances_flat[occurence[_id - 1]]

            _id += 2
            _id = min(_id, len(occurence) + 1)
            o_id = "".join(occurence[:_id])
            path = self.assemblies_path[a_id + o_id]
        if not [i for i in self.rigid_bodies if Sdf.Path(path).HasPrefix(i)]:
            a = [i for i in self.rigid_bodies if Sdf.Path(i).HasPrefix(path) and i != path]
            for p in a:
                self.rigid_bodies.remove(p)
            self.rigid_bodies.add(path)
        path = sorted([i for i in self.rigid_bodies if Sdf.Path(path).HasPrefix(i)])[0]  # There should be only one
        return path

    def process_joints(self, stage, assembly, parent_id, in_group=None):
        a_id = parent_id + str(assembly.get_item("id") or "")
        stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
        rootLayer = stage.GetRootLayer()
        # with Usd.EditContext(stage, rootLayer):
        if 1:
            if (
                not assembly.get_item("suppressed")
                and assembly.get_item("type").lower() == "assembly"
                and assembly.uid in self.assembly.features_map
            ):
                mates = [
                    f
                    for f in self.assembly.features_map[assembly.uid]
                    if f in self.assembly.assembly_features
                    and f in self.assembly.features_details
                    and self.assembly.assembly_features[f]["featureType"] == "mate"
                    and self.assembly.assembly_features[f]["suppressed"] == False
                ]
                for f_id in [
                    f
                    for f in mates
                    if f in self.assembly.assembly_features
                    and self.assembly.assembly_features[f]["featureData"]["mateType"]
                    in ["REVOLUTE", "SLIDER", "CYLINDRICAL", "BALL"]
                    and not Mate(self.assembly.assembly_features[f], self.assembly.features_details[f]).is_locked()
                ]:

                    feature = self.assembly.assembly_features[f_id]
                    f_id = feature["id"]
                    # print(assembly.get_item("name"), feature["featureData"]["name"])
                    # print("feature", feature['featureData']["name"])
                    # f = Mate(self.assembly.assembly_features[f_id], self.assembly.features_details[f_id])
                    base, p = [m["matedOccurrence"] for m in feature["featureData"]["matedEntities"]]
                    base_path = self.get_rigid_body_path(a_id, base)
                    prim_path = self.get_rigid_body_path(a_id, p)

                    if base_path == prim_path:
                        carb.log_warn(
                            "Joint {} attempted connecting {} to itself".format(
                                feature["featureData"]["name"], base_path
                            )
                            + str(self.assembly._instances_flat[base[-1]].get_item("name"))
                            + ", "
                            + str(self.assembly._instances_flat[p[-1]].get_item("name"))
                        )
                        continue
                    # print(" ",self.assembly._instances_flat[base[-1]].get_item("name"))
                    # print(" ",self.assembly._instances_flat[p[-1]].get_item("name"))
                    # print(" ",prim_path)
                    mate = Mate(self.assembly.assembly_features[f_id], self.assembly.features_details[f_id])
                    if mate.limits[0] == mate.limits[1] and mate.limits[0] is not None:
                        continue
                    body_0 = UsdGeom.Xform.Define(stage, base_path).GetPrim()
                    body_1 = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
                    # body_1 = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).GetPrim()

                    UsdPhysics.RigidBodyAPI.Apply(body_0)
                    UsdPhysics.RigidBodyAPI.Apply(body_1)

                    for p in [a for a in TraversePrim(body_0) + TraversePrim(body_1) if a.IsInstanceable()]:
                        UsdPhysics.CollisionAPI.Apply(p)
                        collisionAPI = UsdPhysics.MeshCollisionAPI.Apply(p)
                        collisionAPI.CreateApproximationAttr().Set("convexHull")

                    body_0_global = get_world_transform_matrix(body_0)
                    body_1_global = get_world_transform_matrix(body_1)

                    joint_parent_assembly = mate.positions[0]
                    joint_parent_assembly.SetTranslateOnly(joint_parent_assembly.ExtractTranslation() / stage_unit)
                    a0 = stage.GetPrimAtPath(self.assemblies_path[a_id + "".join(base)])
                    p_a = get_world_transform_matrix(a0)
                    joint_global_pose = joint_parent_assembly * p_a  #
                    # t0.SetTranslateOnly(t0.ExtractTranslation() + t0.ExtractRotation().TransformDir(joint_parent_assembly.ExtractTranslation()))
                    # t0.SetRotateOnly(joint_parent_assembly.ExtractRotation()*t0.ExtractRotation())
                    # * joint_parent_assembly#.GetInverse()
                    root = self.assemblies_path[""]
                    p = "{}/{}".format(root, pxr.Tf.MakeValidIdentifier(make_valid_filename(mate.name)))
                    p = get_next_free_path(stage, p, False)
                    # print(p)
                    if mate.type in ["SLIDER", "REVOLUTE"]:
                        if mate.type == "SLIDER":
                            joint = UsdPhysics.PrismaticJoint.Define(stage, p)
                            PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "linear")
                            for i in range(len(mate.limits)):
                                if mate.limits[i]:
                                    mate.limits[i] /= stage_unit
                        if mate.type == "REVOLUTE":
                            joint = UsdPhysics.RevoluteJoint.Define(stage, p)
                            PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
                        create_joint_attributes(
                            stage,
                            joint,
                            mate,
                            mate.limits,
                            joint_global_pose,
                            base_path,
                            prim_path,
                            body_0_global,
                            body_1_global,
                        )
                    if mate.type == "CYLINDRICAL":
                        # Create proxy rigid body
                        proxy_path = (
                            Sdf.Path(base_path)
                            .GetParentPath()
                            .AppendChild("{}_proxy".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(mate.name))))
                        )
                        g_prim = UsdGeom.Xform.Define(stage, proxy_path).GetPrim()
                        UsdPhysics.RigidBodyAPI.Apply(g_prim)
                        mass_api = UsdPhysics.MassAPI.Apply(g_prim)
                        mass_api.CreateMassAttr(0.001)  # Add a non-zero, negligible mass
                        local_t = get_local_transform_matrix(stage.GetPrimAtPath(base_path))
                        set_pose_from_transform(g_prim, local_t)
                        p_l = get_next_free_path(stage, "{}_linear".format(p))
                        joint_slide = UsdPhysics.PrismaticJoint.Define(stage, p_l)
                        joint_slide.CreateAxisAttr(mate.axis)
                        PhysxSchema.JointStateAPI.Apply(joint_slide.GetPrim(), "linear")
                        for i in range(len(mate.limits_linear)):
                            mate.limits_linear[i] /= stage_unit
                        create_joint_attributes(
                            stage,
                            joint_slide,
                            mate,
                            mate.limits_linear,
                            joint_global_pose,
                            base_path,
                            proxy_path,
                            body_0_global,
                            body_0_global,
                        )
                        p_r = get_next_free_path(stage, "{}_rotate".format(p))
                        joint_rotate = UsdPhysics.RevoluteJoint.Define(stage, p_r)
                        PhysxSchema.JointStateAPI.Apply(joint_rotate.GetPrim(), "angular")
                        create_joint_attributes(
                            stage,
                            joint_rotate,
                            mate,
                            mate.limits_radial,
                            joint_global_pose,
                            proxy_path,
                            prim_path,
                            body_0_global,
                            body_1_global,
                        )
                    if mate.type == "BALL":
                        # Create proxy rigid body
                        proxy_path = (
                            Sdf.Path(base_path)
                            .GetParentPath()
                            .AppendChild("{}_proxy".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(mate.name))))
                        )
                        g_prim = UsdGeom.Xform.Define(stage, proxy_path).GetPrim()
                        UsdPhysics.RigidBodyAPI.Apply(g_prim)
                        mass_api = UsdPhysics.MassAPI.Apply(g_prim)
                        mass_api.CreateMassAttr(0.001)  # Add a non-zero, negligible mass
                        local_t = get_local_transform_matrix(stage.GetPrimAtPath(base_path))
                        set_pose_from_transform(g_prim, local_t)
                        proxy2_path = (
                            Sdf.Path(base_path)
                            .GetParentPath()
                            .AppendChild("{}_proxy2".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(mate.name))))
                        )
                        g2_prim = UsdGeom.Xform.Define(stage, proxy2_path).GetPrim()
                        UsdPhysics.RigidBodyAPI.Apply(g2_prim)
                        mass_api = UsdPhysics.MassAPI.Apply(g2_prim)
                        mass_api.CreateMassAttr(0.001)  # Add a non-zero, negligible mass
                        local_t = get_local_transform_matrix(stage.GetPrimAtPath(base_path))
                        set_pose_from_transform(g2_prim, local_t)
                        p_l = get_next_free_path(stage, "{}_revolute1".format(p))
                        joint_slide = UsdPhysics.RevoluteJoint.Define(stage, p_l)
                        joint_slide.CreateAxisAttr(mate.axis)
                        PhysxSchema.JointStateAPI.Apply(joint_slide.GetPrim(), "angular")
                        create_joint_attributes(
                            stage,
                            joint_slide,
                            mate,
                            mate.limits,
                            joint_global_pose,
                            base_path,
                            proxy_path,
                            body_0_global,
                            body_0_global,
                        )
                        p_r = get_next_free_path(stage, "{}_cone".format(p))
                        joint_rotate = UsdPhysics.RevoluteJoint.Define(stage, p_r)
                        PhysxSchema.JointStateAPI.Apply(joint_rotate.GetPrim(), "angular")
                        create_joint_attributes(
                            stage,
                            joint_rotate,
                            mate,
                            [0, mate.limit_cone],
                            joint_global_pose,
                            proxy_path,
                            proxy2_path,
                            body_0_global,
                            body_0_global,
                        )
                        joint_rotate.CreateAxisAttr(mate.axis_cone)
                        p_l = get_next_free_path(stage, "{}_revolute2".format(p))
                        joint_slide = UsdPhysics.RevoluteJoint.Define(stage, p_l)
                        joint_slide.CreateAxisAttr(mate.axis)
                        PhysxSchema.JointStateAPI.Apply(joint_slide.GetPrim(), "angular")
                        create_joint_attributes(
                            stage,
                            joint_slide,
                            mate,
                            mate.limits,
                            joint_global_pose,
                            proxy2_path,
                            prim_path,
                            body_0_global,
                            body_1_global,
                        )

        for a in assembly.get_children():
            if not a.get_item("suppressed"):
                p_in_group = in_group
                if a_id + a.get_item("id") in self.group_map:
                    p_in_group = self.group_map[a_id + a.get_item("id")]

                self.process_joints(stage, a, a_id, p_in_group)
        # self.create_group_mates(assembly, a_id, in_group)

    def _delayed_make_assembly(self):
        with self.assembly_lock:
            self.delayed_make_assembly = False

            # self.main_thread.join()
            # def do_task():
            # print("Building assembly")

            self.stage_path = "{}/{}_base.usd".format(self.tempdir, make_valid_filename(self.assembly.get_name()))
            self.edit_layer_path = "{}/{}_edit.usd".format(self.tempdir, make_valid_filename(self.assembly.get_name()))
            self.temp_stage_path = "{}/{}_temp.usd".format(self.tempdir, make_valid_filename(self.assembly.get_name()))
            # print(self.stage_path)
            if not self.assembly_stage:
                self.assembly_stage = createInMemoryStage(self.stage_path, self.stage_unit)
                self._assembly_edit_stage = createInMemoryStage(self.edit_layer_path, self.stage_unit)
                root = UsdGeom.Xform.Define(self._assembly_edit_stage, "/World").GetPrim()
                self._assembly_edit_stage.SetDefaultPrim(root)
                root_layer = self._assembly_edit_stage.GetRootLayer()
                root_layer.SetPermissionToEdit(True)
                root_layer.subLayerPaths.append(self.assembly_stage.GetRootLayer().identifier)
                self._assembly_edit_stage.Save()
                stage = self.assembly_stage

            self.temp_stage = createInMemoryStage(self.temp_stage_path, self.stage_unit)
            stage = self.temp_stage
            rootLayer = stage.GetRootLayer()
            rootLayer.SetPermissionToEdit(True)
            # with Usd.EditContext(self.assembly_stage, rootLayer):
            # with self.stage_lock:
            # asyncio.wait(omni.kit.app.get_app().next_update_async())

            # print(self.assembly_stage)
            # print(self.assembly._root._item)

            # self.assembly_stage.Save()
            path = "/World/{}".format(pxr.Tf.MakeValidIdentifier(make_valid_filename(self.assembly.get_name())))
            self.root = path
            _root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
            UsdGeom.Xform.Define(stage, self.root).GetPrim()
            stage.SetDefaultPrim(_root)
            mat_layer = os.path.relpath(self._materials_path, self.tempdir).replace("\\", "/")
            if mat_layer not in rootLayer.subLayerPaths:
                # with self.materials_update_lock:
                rootLayer.subLayerPaths.append(mat_layer)
            # print("wrote mat layer")
            if self.rig_physics:
                UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(path))
                root_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
                root_api.CreateEnabledSelfCollisionsAttr().Set(False)

            if self.rig_physics:
                self.make_groups_xform(stage, self.assembly._root, path)
            # print("write assembly xforms")
            for a in self.assembly._root.get_children():
                if not a.get_item("suppressed"):
                    c_id = a.get_item("id")
                    in_group = None
                    if c_id in self.group_map:
                        in_group = self.group_map[c_id]
                    self.write_assembly_xform(stage, a, "", in_group=in_group)
            while self.assembly_children_stack:
                keys = list(self.assembly_children_stack.keys())
                for path in keys:
                    xforms_to_define = [path]
                    parent = Sdf.Path(path).GetParentPath().pathString
                    while not stage.GetPrimAtPath(parent):
                        xforms_to_define.append(parent)
                        parent = Sdf.Path(parent).GetParentPath().pathString
                    for i in range(len(xforms_to_define) - 1, -1, -1):
                        UsdGeom.Xform.Define(stage, xforms_to_define[i])
                    for a, b, c in self.assembly_children_stack[path]:
                        self.write_assembly_xform(stage, a, b, c)
                    self.assembly_children_stack.pop(path)
            if self.rig_physics:
                self.process_joints(stage, self.assembly._root, "")
            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(300)
            light_pose = Transform(r=Float4(-0.383, 0, 0, 0.924))
            set_pose(distantLight, light_pose)
            if self.rig_physics:
                scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
                physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())

            stage.Save()

            self._pending_assembly_notify = True

    def assembly_notify(self):
        self._pending_assembly_notify = False
        omni.kit.commands.execute(
            "RemovePrimSpec", layer_identifier=self.assembly_stage.GetRootLayer().identifier, prim_spec_path=["/World"]
        )
        omni.kit.commands.execute(
            "MovePrimSpecsToLayer",
            dst_layer_identifier=self.assembly_stage.GetRootLayer().identifier,
            src_layer_identifier=self.temp_stage.GetRootLayer().identifier,
            prim_spec_path=str("/World"),
            dst_stronger_than_src=False,
        )
        self.assembly_stage.Save()
        del self.temp_stage
        self.temp_stage = None
        omni.client.delete(self.temp_stage_path)
        # while self.unloaded_prims:
        #     p = self.unloaded_prims.pop()
        #     p.Load()
        if self.assembly_done_fn:
            self.assembly_done_fn(self.edit_layer_path)

    def _on_update_ui(self, time):
        # if self.delayed_root_layer:
        # self._selection.clear_selected_prim_paths()
        # self.set_root_authoring_layer()
        # asyncio.ensure_future(omni.kit.app.get_app().next_update_async())
        # self.delayed_root_layer = False

        if self.delayed_make_assembly:
            self.delayed_make_assembly = False

            self.assembly_building_pool.submit(self._delayed_make_assembly)

            # self._delayed_make_assembly()
            # if not self.is_temp_stage_open():
            #     omni.usd.get_context().open_stage(self.stage_path)
        if self._pending_assembly_notify:
            self.assembly_notify()

        if self.parts_to_process:
            # with Sdf.ChangeBlock():
            # while self.parts_to_process:
            with self.materials_update_lock:
                colors = []
                while self.parts_to_process:
                    p = self.parts_to_process.pop()
                    self.parts_to_process_post.append(p)
                    part = p[0]
                    for material in part.get_mesh().colors:
                        colors.append(material)
                colors = set(colors)
                for color in colors:
                    self.get_material(color)
            self.material_stage.Save()
        while self.parts_to_process_post:
            part = self.parts_to_process_post.pop()
            if self._parts_stage_dict[part[0].get_key()].payload:
                self.set_payload_mesh(part[0])

            loop = asyncio.get_event_loop()
            self.finalizing_meshes.append(loop.create_task(self.finalize_mesh(part[0])))
            self._parts_stage_dict[part[0].get_key()].payload = True
            if self.mesh_imported_fn:
                self.mesh_imported_fn(None)
            # self.parts_building_pool.submit(self.finalize_mesh,part[0])

        if self.pending_payloads:
            with self.assembly_lock:
                with Sdf.ChangeBlock():
                    while self.pending_payloads:
                        with self.part_stage_lock:
                            pl = self.pending_payloads.pop()
                        pl_path = pl[0]
                        for p in set(pl[1]):
                            prim = self.assembly_stage.GetPrimAtPath(p)
                            if prim:
                                prim.GetPayloads().AddPayload(pl_path)
                    if self.mesh_imported_fn:
                        self.mesh_imported_fn(None)
                self.assembly_stage.Save()

        if self.parts_pending_mass:
            while self.parts_pending_mass:
                # with self.part_stage_lock:
                params = self.parts_pending_mass.pop()
                self.process_part_mass(*params)
            # do_task()
            # asyncio.run(do_task())

        # self.parts_building_pool.submit(do_task)

    def clear_done_tasks(self):
        for a in self.finalizing_meshes:
            if a.done():
                self.finalizing_meshes.remove(a)

    def open_stage(self):
        if self.stage_path:
            omni.usd.get_context().open_stage(self.stage_path)
