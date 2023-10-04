# Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import re
import omni.kit.commands
import omni.kit.utils
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
from omni.isaac.core.utils.stage import get_next_free_path
from omni.isaac.core.utils.rotations import gf_quat_to_np_array
from omni.isaac.core.utils.xforms import reset_and_set_xform_ops
from omni.isaac.core.utils.prims import delete_prim
from pxr import Gf, UsdGeom, Sdf
import omni.usd
import carb
import sys


class IsaacSensorCreatePrim(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "",
        parent: str = "",
        visualize: bool = False,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        schema_type=IsaacSensorSchema.IsaacBaseSensor,
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim_path = None

    def do(self):
        self._stage = omni.usd.get_context().get_stage()

        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = self._schema_type.Define(self._stage, self._prim_path)
        self._prim.CreateEnabledAttr(True)
        self._prim.CreateVisualizeAttr(self._visualize)
        reset_and_set_xform_ops(self._prim.GetPrim(), self._translation, self._orientation)

        return self._prim

    def undo(self):
        if self._prim_path is not None:
            return delete_prim(self._prim_path)


class IsaacSensorCreateContactSensor(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "/Contact_Sensor",
        parent: str = None,
        visualize: bool = False,
        min_threshold: float = 0,
        max_threshold: float = 100000,
        color: Gf.Vec4f = Gf.Vec4f(1, 1, 1, 1),
        radius: float = -1,
        sensor_period: float = -1,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        success, self._prim = omni.kit.commands.execute(
            "IsaacSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=IsaacSensorSchema.IsaacContactSensor,
            translation=self._translation,
            visualize=self._visualize,
        )

        if success and self._prim:
            self._prim.CreateThresholdAttr().Set((self._min_threshold, self._max_threshold))
            self._prim.CreateColorAttr().Set(self._color)
            self._prim.CreateSensorPeriodAttr().Set(self._sensor_period)
            self._prim.CreateRadiusAttr().Set(self._radius)
            return self._prim

        else:
            carb.log_error("Could not create contact sensor prim")
            return None

    def undo(self):
        # undo must be defined even if empty
        pass


class IsaacSensorCreateImuSensor(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "/Imu_Sensor",
        parent: str = None,
        visualize: bool = False,
        sensor_period: float = -1,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        success, self._prim = omni.kit.commands.execute(
            "IsaacSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=IsaacSensorSchema.IsaacImuSensor,
            translation=self._translation,
            orientation=self._orientation,
            visualize=self._visualize,
        )

        if success and self._prim:
            self._prim.CreateSensorPeriodAttr().Set(self._sensor_period)
            return self._prim
        else:
            carb.log_error("Could not create Imu sensor prim")
            return None

    def undo(self):
        # undo must be defined even if empty
        pass


class IsaacSensorCreateRtxLidar(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "/RtxLidar",
        parent: str = None,
        config: str = "Example_Rotary",
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        visibility: bool = False,
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        if sys.platform == "win32":
            carb.log_error("RTX Lidar not supported on windows currently")
            return False, None
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = UsdGeom.Camera.Define(self._stage, Sdf.Path(self._prim_path)).GetPrim()
        IsaacSensorSchema.IsaacRtxLidarSensorAPI.Apply(self._prim)
        camSensorTypeAttr = self._prim.CreateAttribute("cameraSensorType", Sdf.ValueTypeNames.Token, False)
        camSensorTypeAttr.Set("lidar")
        tokens = camSensorTypeAttr.GetMetadata("allowedTokens")
        if not tokens:
            camSensorTypeAttr.SetMetadata("allowedTokens", ["camera", "radar", "lidar"])
        self._prim.CreateAttribute("sensorModelPluginName", Sdf.ValueTypeNames.String, False).Set(
            "omni.sensors.nv.lidar.lidar_core.plugin"
        )
        self._prim.CreateAttribute("sensorModelConfig", Sdf.ValueTypeNames.String, False).Set(self._config)
        if self._visibility is False:
            UsdGeom.Imageable(self._prim).MakeInvisible()
        reset_and_set_xform_ops(self._prim.GetPrim(), self._translation, self._orientation)

        if self._prim:
            return self._prim
        else:
            carb.log_error("Could not create RTX Lidar Prim")
            return None

    def undo(self):
        # undo must be defined even if empty
        pass


class IsaacSensorCreateRtxRadar(omni.kit.commands.Command):
    def __init__(
        self,
        path: str = "/RtxRadar",
        parent: str = None,
        config: str = "Example",
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
    ):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        pass

    def do(self):
        if sys.platform == "win32":
            carb.log_error("RTX Radar not supported on windows currently")
            return False, None
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = UsdGeom.Camera.Define(self._stage, Sdf.Path(self._prim_path)).GetPrim()
        IsaacSensorSchema.IsaacRtxRadarSensorAPI.Apply(self._prim)
        camSensorTypeAttr = self._prim.CreateAttribute("cameraSensorType", Sdf.ValueTypeNames.Token, False)
        camSensorTypeAttr.Set("radar")
        tokens = camSensorTypeAttr.GetMetadata("allowedTokens")
        if not tokens:
            camSensorTypeAttr.SetMetadata("allowedTokens", ["camera", "radar", "lidar"])
        self._prim.CreateAttribute("sensorModelPluginName", Sdf.ValueTypeNames.String, False).Set(
            "omni.sensors.nv.radar.dmatapprox.plugin"
        )
        self._prim.CreateAttribute("sensorModelConfig", Sdf.ValueTypeNames.String, False).Set(self._config)
        reset_and_set_xform_ops(self._prim.GetPrim(), self._translation, self._orientation)

        if self._prim:
            return self._prim
        else:
            carb.log_error("Could not create RTX Radar Prim")
            return None

    def undo(self):
        # undo must be defined even if empty
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
