# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
from omni.isaac.core.utils.rotations import lookat_to_quatf
from pxr import Gf, UsdGeom, Usd


class SpringDamperFollower:
    def __init__(
        self, mass, stiffness, damping, current=Gf.Vec3d(0, 0, 0), target=Gf.Vec3d(0, 0, 0), vel=Gf.Vec3d(0, 0, 0)
    ):
        self.m = mass
        self.k = stiffness
        self.c = damping
        self.current = current
        self.target = target
        self.v = vel

    def update(self, step):
        d = self.target - self.current
        a = (self.k * d - self.c * self.v) / self.m
        self.v = self.v + a * step
        self.current = self.current + self.v * step


class DynamicCamera:
    def __init__(self, stage, base_path, camera_name, focal_length=24, f_stop=5, focus_distance=0):
        self._stage = stage
        self.target_follower = SpringDamperFollower(mass=5, stiffness=5, damping=10)
        self.position_follower = SpringDamperFollower(mass=20, stiffness=5, damping=20)
        self.focus_follower = SpringDamperFollower(mass=1, stiffness=10, damping=10, current=10000, target=10000, vel=0)
        self._base_path = base_path
        self._camera_path = base_path + "/" + camera_name
        self.thresh = 0.1

        self.proxy = self._stage.DefinePrim(self._base_path + "/" + camera_name + "_proxy", "Xform")
        xform = UsdGeom.Xformable(self.proxy)
        xform.ClearXformOpOrder()
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        self._timeline = omni.timeline.get_timeline_interface()

        self.prim = self._stage.DefinePrim(base_path + "/" + camera_name + "_proxy/" + camera_name, "Camera")
        self.prim.GetAttribute("focalLength").Set(focal_length)
        self.prim.GetAttribute("fStop").Set(float(f_stop))
        self.prim.GetAttribute("focusDistance").Set(float(focus_distance))
        self.focus = False

    def reset(self):
        self.position_follower.current = self.position_follower.target
        self.target_follower.current = self.target_follower.target
        self.update(1.0 / 60.0)

    def update(self, step, timecode=Usd.TimeCode.Default()):
        self.target_follower.update(step)
        self.position_follower.update(step)

        pos = self.position_follower.current
        target = self.target_follower.current

        orient = lookat_to_quatf(target, pos, Gf.Vec3d(0, 0, 1))
        mat = Gf.Matrix4d().SetRotateOnly(orient).SetTranslateOnly(pos)
        # mat_1 = Gf.Matrix4d().SetLookAt(self.position_follower.current, self.target_follower.current, Gf.Vec3d(0, 0, 1))
        # trans = mat_1.ExtractTranslation()
        # mat_1.SetTranslateOnly(Gf.Vec3d(trans[2], trans[0], -trans[1]))
        self.proxy.GetAttribute("xformOp:transform").Set(mat, timecode)

        if self.focus:
            self.focus_follower.target = float((target - pos).GetLength())
        else:
            self.focus_follower.target = 10000
        self.focus_follower.update(step)
        # print("focal", self.focus_follower.current)
        self.prim.GetAttribute("focusDistance").Set(float(self.focus_follower.current), timecode)

    def set_look_target(self, pos):
        self.target_follower.target = pos

    def set_pos_target(self, pos):
        self.position_follower.target = pos

    def set_autofocus_target(self, focus):
        self.focus = focus

    def set_pos_settings(self, mass, stiffness, damping):
        self.position_follower.m = mass
        self.position_follower.k = stiffness
        self.position_follower.c = damping

    def set_target_settings(self, mass, stiffness, damping):
        self.target_follower.m = mass
        self.target_follower.k = stiffness
        self.target_follower.c = damping
