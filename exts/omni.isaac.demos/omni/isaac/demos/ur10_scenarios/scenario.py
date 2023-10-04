# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema
import omni.usd
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import set_stage_up_axis
import numpy as np
import gc


def set_translate(prim, new_loc):
    properties = prim.GetPropertyNames()
    if "xformOp:translate" in properties:
        translate_attr = prim.GetAttribute("xformOp:translate")

        translate_attr.Set(new_loc)
    elif "xformOp:translation" in properties:
        translation_attr = prim.GetAttribute("xformOp:translate")
        translation_attr.Set(new_loc)
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        matrix.SetTranslateOnly(new_loc)
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(Gf.Matrix4d().SetTranslate(new_loc))


def set_rotate(prim, rot_mat):
    properties = prim.GetPropertyNames()
    if "xformOp:rotate" in properties:
        rotate_attr = prim.GetAttribute("xformOp:rotate")
        rotate_attr.Set(rot_mat)
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        matrix.SetRotateOnly(rot_mat.ExtractRotation())
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(Gf.Matrix4d().SetRotate(rot_mat))


def create_ur10(stage, env_path, UR10_stage, location):
    envPrim = stage.DefinePrim(env_path, "Xform")
    envPrim.GetReferences().AddReference(UR10_stage)
    set_translate(envPrim, location)


def create_objects(stage, asset_paths, env_paths, translations, rotations=None):
    if rotations is None:
        rotations = [None for i in range(len(asset_paths))]
    if (
        (len(asset_paths) != len(env_paths))
        and len(asset_paths) != len(translations)
        and (len(asset_paths) != len(rotations))
    ):
        print("Error: asset paths, env paths and poses must be same length")
        return
    for (asset, path, translation, rotation) in zip(*[asset_paths, env_paths, translations, rotations]):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(asset)
        set_translate(prim, translation)
        if rotation is not None:
            set_rotate(prim, rotation)


def create_rubiks_cube(stage, asset_path, prim_path, location):
    obstaclePrim = stage.DefinePrim(prim_path, "Xform")
    obstaclePrim.GetReferences().AddReference(asset_path)
    set_translate(obstaclePrim, location)


def create_background(stage, background_stage, pos, rot):
    background_path = "/background"
    if not stage.GetPrimAtPath(background_path):
        backPrim = stage.DefinePrim(background_path, "Xform")
        backPrim.GetReferences().AddReference(background_stage)
        set_translate(backPrim, Gf.Vec3d(pos[0], pos[1], pos[2]))
        set_rotate(backPrim, Gf.Matrix3d(rot))


def setup_physics(stage):
    metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)
    gravityScale = 9.81 / metersPerUnit
    scene = UsdPhysics.Scene.Define(stage, "/physics/scene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(gravityScale)

    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physics/scene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physics/scene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")


class Scenario:
    """ Defines a block stacking scenario

    Scenarios define the life cycle within kit and handle init, startup, shutdown etc.
    """

    def __init__(self, dc, mp):
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = omni.usd.get_context().get_stage()
        self._dc = dc
        self._mp = mp
        self._domains = []  # Contains instances of environment
        self._obstacles = []  # Containts references to any obstacles in the scenario
        self._executor = None
        self._created = False
        self._add_bin_enabled = True
        self.asset_path = None
        self.small_bin_scale = np.array([0.19, 0.296, 0.08])
        self._paused = True

    def __del__(self):
        self.robot_created = False
        if self._executor:
            self._executor.shutdown(True)
            self._executor = None
        self._domains = []
        gc.collect()

    def reset_blocks(self, *args):
        pass

    def stop_tasks(self, *args):
        pass

    def pause_tasks(self, *args):
        return True

    def step(self, step):
        pass

    def open_gripper(self):
        pass

    def add_bin(self, *args):
        pass

    def create_UR10(self, *args):
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.ur10_table_usd = self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_srt.usd"
        self.small_klt_usd = self.assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        self.background_usd = self.assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        self.rubiks_cube_usd = self.assets_root_path + "/Isaac/Props/Rubiks_Cube/rubiks_cube.usd"

        self._created = True
        self._stage = omni.usd.get_context().get_stage()
        set_stage_up_axis("z")
        self.stop_tasks()
        pass

    def register_assets(self, *args):
        pass

    def task(self, domain):
        pass

    def perform_tasks(self, *args):
        return False

    def is_created(self):
        return self._created
