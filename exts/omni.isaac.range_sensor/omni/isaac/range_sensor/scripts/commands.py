# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.commands
import omni.kit.utils
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import carb
from pxr import Gf, UsdGeom
from omni.isaac.core.utils.stage import get_next_free_path


def setup_base_prim(prim, enabled, draw_points, draw_lines, min_range, max_range):
    prim.CreateEnabledAttr(enabled)
    prim.CreateDrawPointsAttr(draw_points)
    prim.CreateDrawLinesAttr(draw_lines)
    prim.CreateMinRangeAttr(min_range)
    prim.CreateMaxRangeAttr(max_range)


# this command is used to create each REB prim, it also handles undo so that each individual prim command doesn't have to
class RangeSensorCreatePrim(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "",
        parent: str = "",
        scehma_type=RangeSensorSchema.RangeSensor,
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim_path = None
        pass

    def do(self):
        self._stage = omni.usd.get_context().get_stage()
        # make prim path unique
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = self._scehma_type.Define(self._stage, self._prim_path)
        setup_base_prim(self._prim, True, self._draw_points, self._draw_lines, self._min_range, self._max_range)

        xform = UsdGeom.Xformable(self._prim)
        xform_trans = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
        xform_rot = xform.AddXformOp(UsdGeom.XformOp.TypeRotateXYZ, UsdGeom.XformOp.PrecisionDouble, "")

        # rotate sensor to align correctly if stage is y up
        if UsdGeom.GetStageUpAxis(self._stage) == UsdGeom.Tokens.y:
            xform_rot.Set(Gf.Vec3d(270, 0, 0))
        return self._prim

    def undo(self):
        if self._prim_path is not None:
            return self._stage.RemovePrim(self._prim_path)


class RangeSensorCreateLidar(omni.kit.commands.Command):
    """Commands class to create a lidar sensor.

        Typical usage example:

        .. code-block:: python

            result, prim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/Lidar",
                parent=None,
                min_range=0.4,
                max_range=100.0,
                draw_points=False,
                draw_lines=False,
                horizontal_fov=360.0,
                vertical_fov=30.0,
                horizontal_resolution=0.4,
                vertical_resolution=4.0,
                rotation_rate=20.0,
                high_lod=False,
                yaw_offset=0.0,
                enable_semantics=False,
            )
    """

    def __init__(
        self,
        path: str = "/Lidar",
        parent=None,
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        horizontal_fov: float = 360.0,
        vertical_fov: float = 30.0,
        horizontal_resolution: float = 0.4,
        vertical_resolution: float = 4.0,
        rotation_rate: float = 20.0,
        high_lod: bool = False,
        yaw_offset: float = 0.0,
        enable_semantics: bool = False,
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        success, self._prim = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            scehma_type=RangeSensorSchema.Lidar,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )
        if success and self._prim:
            self._prim.CreateHorizontalFovAttr().Set(self._horizontal_fov)
            self._prim.CreateVerticalFovAttr().Set(self._vertical_fov)
            self._prim.CreateRotationRateAttr().Set(self._rotation_rate)
            self._prim.CreateHorizontalResolutionAttr().Set(self._horizontal_resolution)
            self._prim.CreateVerticalResolutionAttr().Set(self._vertical_resolution)
            self._prim.CreateHighLodAttr().Set(self._high_lod)
            self._prim.CreateYawOffsetAttr().Set(self._yaw_offset)
            self._prim.CreateEnableSemanticsAttr().Set(self._enable_semantics)
        else:
            carb.log.error("Could not create lidar prim")
        return self._prim

    def undo(self):
        # undo must be defined even if empty
        pass


class RangeSensorCreateUltrasonicArray(omni.kit.commands.Command):
    """Commands class to create an ultrasonic array.

        Typical usage example:

        .. code-block:: python

            result, prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicArray",
                path="/UltrasonicArray",
                parent=None,
                min_range=0.4,
                max_range=3.0,
                draw_points=False,
                draw_lines=False,
                horizontal_fov=15.0,
                vertical_fov=10.0,
                horizontal_resolution=0.5,
                vertical_resolution=0.5,
                num_bins=224,
                use_brdf: bool = False,
                use_uss_materials: bool = False,
                emitter_prims=[],
                firing_group_prims=[],
            )
    """

    def __init__(
        self,
        path: str = "/UltrasonicArray",
        parent=None,
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        horizontal_fov: float = 360.0,
        vertical_fov: float = 30.0,
        rotation_rate: float = 20.0,
        horizontal_resolution: float = 0.4,
        vertical_resolution: float = 4.0,
        num_bins: int = 224,
        use_brdf: bool = False,
        use_uss_materials: bool = False,
        emitter_prims: [] = [],
        firing_group_prims: [] = [],
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        success, self._prim = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            scehma_type=RangeSensorSchema.UltrasonicArray,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )
        if success and self._prim:
            self._prim.CreateHorizontalFovAttr().Set(self._horizontal_fov)
            self._prim.CreateVerticalFovAttr().Set(self._vertical_fov)
            self._prim.CreateHorizontalResolutionAttr().Set(self._horizontal_resolution)
            self._prim.CreateVerticalResolutionAttr().Set(self._vertical_resolution)
            self._prim.CreateNumBinsAttr().Set(self._num_bins)
            self._prim.CreateUseBRDFAttr().Set(self._use_brdf)
            self._prim.CreateUseUSSMaterialsForBRDFAttr().Set(self._use_uss_materials)

            rel_paths = self._prim.CreateEmitterPrimsRel()
            for p in self._emitter_prims:
                rel_paths.AddTarget(p)

            rel_paths = self._prim.CreateFiringGroupsRel()
            for p in self._firing_group_prims:
                rel_paths.AddTarget(p)
        return self._prim

    def undo(self):
        # undo must be defined even if empty
        pass


class RangeSensorCreateUltrasonicEmitter(omni.kit.commands.Command):
    """Commands class to create an ultrasonic emitter.

        Typical usage example:

        .. code-block:: python

            result, prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicEmitter",
                path="/UltrasonicEmitter",
                parent=None,
                per_ray_intensity=1.0,
                yaw_offset=0.0,
                adjacency_list=[],
            )
    """

    def __init__(
        self,
        path: str = "/UltrasonicEmitter",
        parent=None,
        per_ray_intensity: float = 1.0,
        yaw_offset: float = 0.0,
        adjacency_list: [] = [],
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        self._stage = omni.usd.get_context().get_stage()
        # make prim path unique
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = RangeSensorSchema.UltrasonicEmitter.Define(self._stage, self._prim_path)

        xform = UsdGeom.Xformable(self._prim)
        xform_trans = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
        xform_rot = xform.AddXformOp(UsdGeom.XformOp.TypeRotateXYZ, UsdGeom.XformOp.PrecisionDouble, "")

        # rotate sensor to align correctly if stage is y up
        if UsdGeom.GetStageUpAxis(self._stage) == UsdGeom.Tokens.y:
            xform_rot.Set(Gf.Vec3d(270, 0, 0))
        if self._prim:
            self._prim.CreatePerRayIntensityAttr().Set(self._per_ray_intensity)
            self._prim.CreateYawOffsetAttr().Set(self._yaw_offset)
            self._prim.CreateAdjacencyListAttr().Set(self._adjacency_list)
        return self._prim

    def undo(self):
        if self._prim_path is not None:
            return self._stage.RemovePrim(self._prim_path)
        pass


class RangeSensorCreateUltrasonicFiringGroup(omni.kit.commands.Command):
    """Commands class to create an ultrasonic firing group.

        Typical usage example:

        .. code-block:: python

            result, prim = omni.kit.commands.execute(
                "RangeSensorCreateUltrasonicFiringGroup",
                path="/UltrasonicFiringGroup",
                parent=None,
                emitter_modes=[],
                receiver_modes=[],
            )
    """

    def __init__(
        self, path: str = "/UltrasonicFiringGroup", parent=None, emitter_modes: [] = [], receiver_modes: [] = []
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        self._stage = omni.usd.get_context().get_stage()
        # make prim path unique
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = RangeSensorSchema.UltrasonicFiringGroup.Define(self._stage, self._prim_path)

        if self._prim:
            self._prim.CreateEmitterModesAttr().Set(self._emitter_modes)
            self._prim.CreateReceiverModesAttr().Set(self._receiver_modes)
        return self._prim

    def undo(self):
        if self._prim_path is not None:
            return self._stage.RemovePrim(self._prim_path)
        pass


class RangeSensorCreateGeneric(omni.kit.commands.Command):
    """Commands class to create a generic range sensor.

        Typical usage example:

        .. code-block:: python

            result, prim = omni.kit.commands.execute(
                "RangeSensorCreateGeneric",
                path="/GenericSensor",
                parent=None,
                min_range=0.4,
                max_range=100.0,
                draw_points=False,
                draw_lines=False,
                sampling_rate=60,
            )
    """

    def __init__(
        self,
        path: str = "/GenericSensor",
        parent=None,
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        sampling_rate: int = 60,
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        success, self._prim = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            scehma_type=RangeSensorSchema.Generic,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )
        if success and self._prim:
            self._prim.CreateSamplingRateAttr().Set(self._sampling_rate)
        else:
            carb.log.error("Could not create generic sensor prim")
        return self._prim

    def undo(self):
        if self._prim_path is not None:
            return self._stage.RemovePrim(self._prim_path)
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
