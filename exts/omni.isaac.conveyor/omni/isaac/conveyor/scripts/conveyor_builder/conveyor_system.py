from math import asin, degrees
from typing import List

import omni
from pxr import Gf, UsdGeom
from pxr.Usd import Stage
from .conveyor_track import Angle, ConveyorTrack, Curvature, Ramp, Style, Type


def set_pose_from_transform(prim, pose, scale=Gf.Vec3d(1, 1, 1)):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_s = xform.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
    scale_mat = Gf.Matrix4d().SetScale(scale)
    t = Gf.Matrix4d(pose)
    # t = Gf.Matrix4d(scale_mat*pose)
    r = t.ExtractRotationQuat().GetNormalized()
    pos_vec = t.ExtractTranslation()
    xform_op_t.Set(pos_vec)
    xform_op_r.Set(r)
    xform_op_s.Set(scale)


def remove_scale_from_matrix(matrix):
    out = Gf.Matrix4d()
    x = Gf.Vec3d(0)
    y = Gf.Vec3d(0)
    z = Gf.Vec3d(0)
    for i in range(3):
        x[i] = matrix[0][i]
        y[i] = matrix[1][i]
        z[i] = matrix[2][i]
    l_x = x.GetLength()
    l_y = y.GetLength()
    l_z = z.GetLength()
    # print(l_x, l_y, l_z)
    for i in range(3):
        out[0][i] = matrix[0][i] / l_x
        out[1][i] = matrix[1][i] / l_y
        out[2][i] = matrix[2][i] / l_z
    for i in range(4):
        out[i][3] = matrix[i][3]
    out[3] = matrix[3]
    return out


class ConveyorFilter:
    def __init__(
        self,
        styles: List[Style] = [],
        angles: List[Angle] = [],
        curvatures: List[Curvature] = [],
        ramps: List[Ramp] = [],
        types: List[Type] = [],
    ):
        self.style = styles
        self.angle = angles
        self.curvature = curvatures
        self.ramp = ramps
        self.type = types

    def clear_filter(self):
        self.style = []
        self.angle = []
        self.curvature = []
        self.ramp = []
        self.type = []


class ConveyorSelector:
    def __init__(self, config_file, **kwargs):
        self.asset_path = config_file["source"]
        self.tracks = {}
        for track in config_file["assets"]:
            base_usd = "{}.usd".format(track)
            self.tracks[base_usd] = ConveyorTrack(
                base_usd="{}/{}".format(self.asset_path, base_usd),
                **config_file["assets"][track],
                thumb_loaded_callback=self.on_thumb_loaded,
            )
        self._thumb_loaded_callback = kwargs.get("thumb_loaded_callback", None)

    def on_thumb_loaded(self, track):
        if self._thumb_loaded_callback:
            self._thumb_loaded_callback(track)

    def list_tracks(self, config: ConveyorFilter):
        v = list(self.tracks.values())
        tracks = [
            a
            for a in self.tracks
            if (not config.style or self.tracks[a].style in config.style)
            and (not config.angle or self.tracks[a].angle in config.angle)
            and (not config.curvature or self.tracks[a].curvature in config.curvature)
            and (not config.ramp or self.tracks[a].ramp in config.ramp)
            and (not config.type or self.tracks[a].type in config.type)
        ]

        return tracks


class ConveyorBuilder:
    def __init__(self, stage, conveyor_selector=None):
        self.stage = stage
        if conveyor_selector:
            self._conveyor_selector = conveyor_selector
        self._tracks = {}  # usd path, ConveyorTrack
        self._anchor_connections = {}  # usd path, dict(anchor ID,usd_path)
        self._parent_anchor = {}  # usd path, anchor ID

    def clear_system(self, stage):
        self.__init__(stage)

    def get_available_connections(self, track_path):
        if track_path in self._tracks:
            available_connections = [
                a
                for a in self._tracks[track_path].get_anchors()
                if a not in self._anchor_connections[track_path].keys()
            ]
            return available_connections

    def is_track(self, prim_path):
        sel = self.stage.GetPrimAtPath(prim_path)
        if sel.GetReferences():
            refs = []
            refs += omni.usd.get_composed_references_from_prim(sel, False)
            if sel.GetPayloads():
                refs += omni.usd.get_composed_payloads_from_prim(sel, False)
            for a in refs:
                if self._conveyor_selector.asset_path in a[0].assetPath:
                    return True
        return False

    def add_track(self, track, track_anchor="", x_direction=1, y_direction=1, parent=None, parent_anchor=""):
        # print("adding", track_anchor, parent_anchor)

        next_pose = self.get_next_pose(track, track_anchor, x_direction, y_direction, parent, parent_anchor)
        parent_pose = Gf.Matrix4d()
        if parent in self._tracks:
            world_prim = self.stage.GetPrimAtPath(parent).GetParent()
            _prim_path = omni.usd.get_stage_next_free_path(
                self.stage, str(world_prim.GetPath()) + "/ConveyorTrack", False
            )
            parent_pose = omni.usd.utils.get_world_transform_matrix(world_prim)
        elif self.is_track(parent):
            parent_path = self.stage.GetPrimAtPath(parent).GetParent().GetPath()
            _prim_path = omni.usd.get_stage_next_free_path(self.stage, str(parent_path) + "/ConveyorTrack", False)
        else:
            parent_path = ""
            if parent and parent != "/" and self.stage.GetPrimAtPath(parent):
                parent_path = parent
            else:
                parent_path = self.stage.GetDefaultPrim().GetPath()
            _prim_path = omni.usd.get_stage_next_free_path(self.stage, str(parent_path) + "/ConveyorTrack", False)
        prim = self.stage.DefinePrim(_prim_path, "Xform")
        prim.GetReferences().AddReference(track.base_usd)
        part_stage = Stage.Open(track.base_usd)
        scale = Gf.Vec3d(x_direction, y_direction, 1)
        a = UsdGeom.GetStageMetersPerUnit(self.stage)
        b = UsdGeom.GetStageMetersPerUnit(part_stage)
        scale_unit = b / a
        # pose = Gf.Matrix4d().SetScale(scale) * next_pose
        set_pose_from_transform(prim, parent_pose.GetInverse() * next_pose, scale * scale_unit)

        new_track = str(prim.GetPath())
        self._tracks[new_track] = track
        self._anchor_connections[new_track] = {}
        self._parent_anchor[new_track] = track_anchor
        if parent in self._tracks:
            self._anchor_connections[new_track][track_anchor] = parent
            self._anchor_connections[parent][parent_anchor] = new_track
        for node in track.conveyor_nodes.keys():
            _, conveyor_prim = omni.kit.commands.execute(
                "CreateConveyorBelt", conveyor_prim=self.stage.GetPrimAtPath(prim.GetPath().AppendChild(node))
            )
            if conveyor_prim:
                conveyor_prim.GetAttribute("inputs:animateDirection").Set(
                    Gf.Vec2f(*track.conveyor_nodes[node]["animate_direction"])
                )
                forward_direction = 1
                if track_anchor != "":
                    if "1" not in node and "1" not in track_anchor:
                        forward_direction = -1
                    if "1" in node and "1" in track_anchor:
                        forward_direction = -1
                if track.angle != Angle.NONE:
                    forward_direction *= y_direction

                conveyor_prim.GetAttribute("inputs:animateScale").Set(track.conveyor_nodes[node]["animate_scale"])
                direction = forward_direction * Gf.Vec3f(*track.conveyor_nodes[node]["direction"])
                conveyor_prim.GetAttribute("inputs:direction").Set(direction)
                conveyor_prim.GetAttribute("inputs:curved").Set(track.conveyor_nodes[node]["curved"])

        return new_track

    def remove_track(self, usd_path):
        previous_track = ""
        parent_anchor = self._parent_anchor[usd_path]
        if parent_anchor in self._anchor_connections[usd_path]:
            previous_track = self._anchor_connections[usd_path][parent_anchor]
        for anchor in self._anchor_connections[usd_path]:
            parent = self._anchor_connections[usd_path][anchor]
            if parent in self._anchor_connections:
                for cn in list(self._anchor_connections[parent].keys()):
                    if self._anchor_connections[parent][cn] == usd_path:
                        self._anchor_connections[parent].pop(cn)
                        break
        self._anchor_connections.pop(usd_path)
        self._parent_anchor.pop(usd_path)
        self._tracks.pop(usd_path)
        return previous_track

    def get_direction(self, usd_path):
        if usd_path in self._tracks:
            if self._tracks[usd_path].ramp == Ramp.FLAT:
                return 1
            conns = self.get_available_connections(usd_path)
            if conns and conns[0] == "":
                return -1
        return 1

    def get_next_pose(self, track, track_anchor="", x_direction=1, y_direction=1, parent=None, parent_anchor=""):
        base_pose = Gf.Matrix4d()
        x_scale = 1
        y_scale = 1
        track_stage = Stage.Open(track.base_usd)
        a = UsdGeom.GetStageMetersPerUnit(self.stage)
        b = UsdGeom.GetStageMetersPerUnit(track_stage)
        scale_unit = b / a
        # print("Get base pose", parent, [parent_anchor], [track_anchor])
        if parent:
            anchor_prim = self.stage.GetPrimAtPath("{}{}".format(parent, parent_anchor))
            base_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(parent))
            base_pose = omni.usd.utils.get_world_transform_matrix(base_xform.GetPrim())

            scale_ops = [o for o in base_xform.GetOrderedXformOps() if o.GetOpType() in [UsdGeom.XformOp.TypeScale]]
            anchor_pose = Gf.Matrix4d()
            if parent_anchor:
                anchor_pose = omni.usd.utils.get_local_transform_matrix(anchor_prim)
            if scale_ops:
                # first, get the global pose of the base

                # Revert the scaling
                scale = Gf.Matrix4d().SetScale(scale_ops[0].Get())
                base_pose = scale.GetInverse() * base_pose
                base_pose.SetTranslateOnly(base_pose.ExtractTranslation() / scale_unit)

                # Rotate 180 degrees on the Z axis

                x_scale = scale_ops[0].Get()[0]
                y_scale = scale_ops[0].Get()[1]
            if y_scale < 0:
                t = anchor_pose.ExtractTranslation()
                t[1] = -t[1]
                anchor_pose.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = anchor_pose.ExtractRotation().TransformDir((Gf.Vec3d(1, 0, 0)))
                prev_angle = anchor_pose.ExtractRotation().GetAngle()
                dot = direction * Gf.Vec3d(1, 0, 0)
                anchor_pose.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * anchor_pose.ExtractRotation()
                )
                if prev_angle < 89.0 and prev_angle > 1.0:
                    anchor_pose.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180) * anchor_pose.ExtractRotation())
            if x_scale < 0:
                t = anchor_pose.ExtractTranslation()
                t[0] = -t[0]
                anchor_pose.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = anchor_pose.ExtractRotation().TransformDir((Gf.Vec3d(1, 0, 0)))

                dot = direction * Gf.Vec3d(1, 0, 0)
                anchor_pose.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * anchor_pose.ExtractRotation()
                )
            if not parent_anchor:
                anchor_pose.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))

            base_pose = anchor_pose * base_pose

        slope_and_direction = Gf.Vec3f(x_direction, y_direction, 1)
        # if track.angle == Angle.HALF and y_direction == 1 and track_anchor:
        #     y_direction = -1
        m = Gf.Matrix4d()
        m.SetRotateOnly(base_pose.ExtractRotation())
        m.SetTranslateOnly(base_pose.ExtractTranslation())
        # print(track, track_anchor)
        if track_anchor:
            anchor_prim = track_stage.GetPrimAtPath(str(track_stage.GetDefaultPrim().GetPath()) + track_anchor)
            mat = omni.usd.utils.get_local_transform_matrix(anchor_prim)
            if y_direction < 0:
                t = mat.ExtractTranslation()
                t[1] = -t[1]
                mat.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                prev_angle = mat.ExtractRotation().GetAngle()
                direction = mat.ExtractRotation().TransformDir((Gf.Vec3d(1, 0, 0)))

                dot = direction * Gf.Vec3d(1, 0, 0)
                mat.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * mat.ExtractRotation()
                )
                if prev_angle < 89.0 and prev_angle > 1.0:
                    mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180) * mat.ExtractRotation())
            if x_direction < 0:
                t = mat.ExtractTranslation()
                t[0] = -t[0]
                mat.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = mat.ExtractRotation().TransformDir((Gf.Vec3d(1, 0, 0)))

                dot = direction * Gf.Vec3d(1, 0, 0)
                mat.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * mat.ExtractRotation()
                )
            pivot_angle = mat.ExtractRotationQuat().GetNormalized()
            m.SetRotateOnly(
                m.ExtractRotationQuat()
                * (mat.ExtractRotationQuat().GetInverse() * Gf.Quatd(0, Gf.Vec3d(0, 0, -1))).GetNormalized()
            )
            t = Gf.Transform()
            t.SetRotation(Gf.Rotation(pivot_angle))
            t.SetTranslation(-mat.ExtractTranslation())
            m.SetTranslateOnly((t.GetMatrix().GetInverse() * base_pose).ExtractTranslation())
        if slope_and_direction[0] < 0:
            m.SetRotateOnly(m.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
        scale = Gf.Matrix4d().SetScale(scale_unit)
        # print(scale)
        # print(m)
        m.SetTranslateOnly((Gf.Matrix4d().SetTranslate(m.ExtractTranslation()) * scale).ExtractTranslation())
        # print(m)
        # print()
        return m
