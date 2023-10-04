# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdLux, PhysxSchema
import omni
import carb
from pathlib import PurePosixPath as PPath
from functools import partial

joint_positions_list = ["physics:localPos0", "physics:localPos1"]

joint_attribute_list = [
    "physics:minDistance",
    "physics:maxDistance",
    "physics:breakForce",
    *["limit:trans{}:physics:{}".format(x, limit) for x in ["X", "Y", "Z"] for limit in ["high", "low"]],
    *["physxLimits:trans{}:bounceThreshold".format(x) for x in ["X", "Y", "Z"]],
    *["physxLimits:trans{}:contactDistance".format(x) for x in ["X", "Y", "Z"]],
    *[
        "drive:{}:physics:{}".format(x, t)
        for x in ["trans", "transY", "transZ", "linear"]
        for t in ["targetPosition", "targetVelocity", "maxForce"]
    ],
]
# joint drive max force is proportional to the distance when it's linear, and squared when it's revolute (torque)

joint_force_attribute_list = [  # This list is for scale squared
    "physics:breakTorque",
    *["drive:{}:physics:maxForce".format(x) for x in ["rotX", "rotY", "rotZ", "angular"]],
]


def scale_xform(prim, scale, make_delta=False):
    """
    scales the translation component of an Xformable prim
    """
    xform = UsdGeom.Xform(prim)
    layer = prim.GetStage().GetRootLayer()
    for op in [
        o
        for o in xform.GetOrderedXformOps()
        if o.GetOpType() in [UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeTransform]
        and (layer.GetAttributeAtPath(prim.GetAttribute(o.GetName()).GetPath()) or make_delta)
    ]:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            scale_translate_op(op, scale)
        elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
            scale_transform_op(op, scale)


def scale_mesh(prim, scale, add_missing_scale):
    """
    uses the x
    """
    xform = UsdGeom.Xform(prim)
    stage = prim.GetStage()
    layer = prim.GetStage().GetRootLayer()
    scale_ops = [
        o
        for o in xform.GetOrderedXformOps()
        if o.GetOpType() in [UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.TypeTransform]
    ]
    for op in scale_ops:
        if add_missing_scale or layer.GetAttributeAtPath(prim.GetAttribute(op.GetName()).GetPath()):
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(op.Get() * scale)
            elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                mat = op.Get()
                transform = Gf.Transform()
                transform.SetMatrix(mat)
                transform.SetScale(transform.GetScale() * scale)
                mat = transform.GetMatrix()
                op.Set(mat)
    if not scale_ops:
        op_order_attr = xform.GetXformOpOrderAttr()
        if add_missing_scale or layer.GetAttributeAtPath(op_order_attr.GetPath()):
            new_target = stage.GetEditTargetForLocalLayer(stage.GetEditTarget().GetLayer())
            with Usd.EditContext(stage, new_target):
                op = xform.AddScaleOp()
                op.Set(Gf.Vec3d(scale, scale, scale))
                # ops.append(op)
                # xform.SetXformOpOrder(ops, xform.GetResetXformStack())

        # omni.kit.commands.execute("EnableXformOp", op_attr_path=prim.GetAttribute(op.GetName()).GetPath())


def set_prop(prim, prop, scale, make_delta=False):
    layer = prim.GetStage().GetRootLayer()
    if (
        layer.GetAttributeAtPath(prop.GetPath()) or make_delta
    ):  # Check if prop exists on the root layer where the prim was accessed (only happens when the prop is authored in that stage or has a delta on a parent layer)
        prop.Set(prop.Get() * scale)


def scale_translate_op(op, scale):
    """
    scales xformable Translate operation
    """
    if not op.IsInverseOp():
        op.Set(op.Get() * scale)


def scale_transform_op(op, scale):
    """
    scales xformable transform operation
    """
    mat = op.Get()
    if not mat:
        mat = Gf.Matrix4d()
    mat.SetTranslateOnly(mat.ExtractTranslation() * scale)
    op.Set(mat)


def scale_cube(cube, scale, make_delta=False):
    """
    Scales cube size attribute
    """
    prop = cube.GetAttribute("size")
    set_prop(cube, prop, scale, make_delta)


def scale_sphere(sphere, scale, make_delta=False):
    """
    scale sphere radius attribute
    """
    prop = sphere.GetAttribute("radius")
    set_prop(sphere, prop, scale, make_delta=make_delta)


def scale_cylinder(cyl, scale, make_delta=False):
    """
    scale cylinder,cone and capsule radius and height
    """
    for p in ["height", "radius"]:
        prop = cyl.GetAttribute(p)
        set_prop(cyl, prop, scale, make_delta)


def scale_joint(joint_prim, scale, make_delta=False, ignore_prim=False):
    """
    scale joint properties for unit conversion
    """
    if not ignore_prim:
        for prop in [joint_prim.GetAttribute(p) for p in joint_positions_list if joint_prim.GetAttribute(p)]:
            set_prop(joint_prim, prop, scale, make_delta)
    for prop in [joint_prim.GetAttribute(p) for p in joint_attribute_list if joint_prim.GetAttribute(p)]:
        set_prop(joint_prim, prop, scale, make_delta)
    for prop in [joint_prim.GetAttribute(p) for p in joint_force_attribute_list if joint_prim.GetAttribute(p)]:
        set_prop(joint_prim, prop, scale ** 2, make_delta)
    if UsdPhysics.PrismaticJoint(joint_prim):
        for prop in [
            joint_prim.GetAttribute(p)
            for p in ["physics:upperLimit", "physics:lowerLimit"]
            if joint_prim.GetAttribute(p)
        ]:
            set_prop(joint_prim, prop, scale, make_delta)


def scale_mass(prim, scale, make_delta=False):
    """
    scale mass properties for unit conversion
    """
    mass_prim = UsdPhysics.MassAPI(prim)
    density = mass_prim.GetDensityAttr()
    if density.Get():
        set_prop(prim, density, scale ** -3, make_delta)
    com = mass_prim.GetCenterOfMassAttr()
    if com.Get():
        set_prop(prim, com, scale, make_delta)
    inertia = mass_prim.GetDiagonalInertiaAttr()
    if inertia.Get().GetLength():
        set_prop(prim, inertia, scale ** 2, make_delta)


def scale_collision(prim, scale, make_delta=False):
    collision_prim = PhysxSchema.PhysxCollisionAPI(prim)
    cvx = PhysxSchema.PhysxConvexDecompositionCollisionAPI(prim)
    ch = PhysxSchema.PhysxConvexHullCollisionAPI(prim)
    if collision_prim:
        contact_offset = collision_prim.GetContactOffsetAttr()
        if contact_offset.Get():
            set_prop(prim, contact_offset, scale, make_delta)
        rest_offset = collision_prim.GetRestOffsetAttr()
        if rest_offset.Get():
            set_prop(prim, rest_offset, scale, make_delta)
        tpr = collision_prim.GetTorsionalPatchRadiusAttr()
        if tpr.Get():
            set_prop(prim, tpr, scale, make_delta)
        mtpr = collision_prim.GetMinTorsionalPatchRadiusAttr()
        if tpr.Get():
            set_prop(prim, mtpr, scale, make_delta)
    if cvx:
        min_thick = cvx.GetMinThicknessAttr()
        if min_thick.Get():
            set_prop(prim, min_thick, scale, make_delta)
    if ch:
        min_thick = ch.GetMinThicknessAttr()
        if min_thick.Get():
            set_prop(prim, min_thick, scale, make_delta)


def scale_rigid_body(rb_prim, scale, make_delta=False):
    """
    scale rigid body properties for unit conversion
    """
    velocity = rb_prim.GetAttribute("physics:velocity")
    set_prop(rb_prim, velocity, scale, make_delta)
    for p in [
        rb_prim.GetAttribute(p)
        for p in [
            "physxRigidBody:maxLinearVelocity",
            "physxRigidBody:sleepThreshold",
            "physxArticulation:sleepThreshold" "physxRigidBody:maxContactImpulse",
            "physxRigidBody:maxDepenetrationVelocity",
        ]
        if rb_prim.GetAttribute(p).Get()
    ]:
        set_prop(rb_prim, p, scale, make_delta)

    for stabilization in [
        rb_prim.GetAttribute(p)
        for p in ["physxRigidBody:stabilizationThreshold", "physxArticulation:stabilizationThreshold"]
        if rb_prim.GetAttribute(p).Get()
    ]:
        set_prop(rb_prim, stabilization, scale ** 2, make_delta)


def scale_scene(prim, scale, make_delta=False):
    for prop in [
        prim.GetAttribute(p)
        for p in [
            "physics:gravityMagnitude",
            "physxScene:bounceTrhreshold",
            "physxScene:frictionCorrelationDistance",
            "physxScene:frictionOffsetThreshold",
        ]
        if prim.GetAttribute(p)
    ]:
        set_prop(prim, prop, scale, make_delta)


def scale_dr_movement(prim, scale, make_delta=False):
    for prop in [
        f() for f in [prim.GetXRangeAttr, prim.GetYRangeAttr, prim.GetZRangeAttr, prim.GetLookAtTargetOffsetAttr] if f()
    ]:
        set_prop(prim.GetPrim(), prop, scale, make_delta)


def scale_camera_params(prim, scale, make_delta=False):
    cam_prim = UsdGeom.Camera(prim)
    for prop in [
        f()
        for f in [
            cam_prim.GetFocalLengthAttr,
            cam_prim.GetFocusDistanceAttr,
            # cam_prim.GetFStopAttr,
            cam_prim.GetHorizontalApertureAttr,
            cam_prim.GetHorizontalApertureOffsetAttr,
            cam_prim.GetVerticalApertureAttr,
            cam_prim.GetVerticalApertureOffsetAttr,
            # cam_prim.GetClippingRangeAttr,
            partial(prim.GetAttribute, "clippingRange"),
            # partial(prim.GetAttribute, "fthetaWidth"),
            # partial(prim.GetAttribute, "fthetaWidth"),
            # partial(prim.GetAttribute, "fthetaCx"),
            # partial(prim.GetAttribute, "fthetaCy"),
            # partial(prim.GetAttribute, "fthetaPolyA"),
            # partial(prim.GetAttribute, "fthetaPolyB"),
            # partial(prim.GetAttribute, "fthetaPolyC"),
            # partial(prim.GetAttribute, "fthetaPolyD"),
            # partial(prim.GetAttribute, "fthetaPolyEv"),
        ]
        if f()
    ]:
        # prop.Set(prop.Get() * scale)
        set_prop(prim, prop, scale, make_delta)


def set_stage_meters_per_unit(
    stage, new_mpu, stage_recursive=False, parent_stack=set(), base_path="", ignore_prim=set()
):
    current_mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if new_mpu == 0:
        carb.log_error("Meters per unit cannot be zero")
        return
    scale = current_mpu / new_mpu
    carb.log_info("Updating meters per unit for stage {}".format(stage))
    UsdGeom.SetStageMetersPerUnit(stage, new_mpu)
    sub_stages = set()
    referred_children = set()
    ignore_prim = set()
    for prim in [
        p
        for p in stage.Traverse()
        if p.GetName() not in ["OmniverseKit_{}".format(i) for i in ["Persp", "Front", "Top", "Right"]]
    ]:

        # if session_layer == prim.GetPrimStack()[0].layer:
        #     if len(str(prim.GetPath()).split("/")) > 2:
        #         deltas.add("/" + str(prim.GetPath()).split("/")[1])
        # print(prim.GetPrimStack())
        if UsdPhysics.Joint(prim) or UsdGeom.Camera(prim) or UsdLux.Light(prim):
            for child in Usd.PrimRange(prim):
                if child != prim:
                    print(child)
                    ignore_prim.add(child)
        if stage_recursive and not (UsdPhysics.Joint(prim) or UsdGeom.Camera(prim) or UsdLux.Light(prim)):
            for layer in prim.GetPrimStack():
                if (
                    layer.layer.identifier != stage.GetRootLayer().identifier
                    and base_path in layer.layer.identifier
                    and layer.layer not in parent_stack
                    and "anon:" not in layer.layer.identifier
                ):
                    sub_stages.add(layer.layer)
                    parent_stack.add(layer.layer)
        # print(prim)
        composed_refs = []
        if stage_recursive:
            composed_refs = omni.usd.get_composed_references_from_prim(prim)
            composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
            # if composed_refs:
            # print(prim)
            # print("  ", composed_refs)
            for c in composed_refs:
                for child in Usd.PrimRange(prim):
                    if c[0] != stage.GetRootLayer().identifier:
                        # child_usd = PPath(c[1]).parent / PPath(c[0])
                        referred_children.add(child)
            for c in composed_payloads:
                for child in Usd.PrimRange(prim):
                    if c[0] != stage.GetRootLayer().identifier:
                        # child_usd = PPath(c[1]).parent / PPath(c[0])
                        referred_children.add(child)
        add_missing_scale = not stage_recursive or prim not in referred_children
        make_delta = not stage_recursive or prim not in referred_children
        if (
            prim.IsInstanceable() and UsdGeom.Xformable(prim) and not stage_recursive
        ):  # If prim is instanceable add scale to top prim xformable
            if prim not in ignore_prim:
                scale_mesh(prim, scale, add_missing_scale)
                for p in Usd.PrimRange(prim):
                    if p != prim:
                        ignore_prim.add(p)
        if UsdPhysics.Joint(prim):
            scale_joint(prim, scale, make_delta, prim in ignore_prim)
        if prim in ignore_prim:
            continue
        if UsdPhysics.Scene(prim):
            scale_scene(prim, scale, make_delta)
        if UsdGeom.Xformable(prim):
            scale_xform(prim, scale, make_delta)
        if UsdGeom.Mesh(prim) and prim not in ignore_prim:
            scale_mesh(prim, scale, add_missing_scale)
            for child in Usd.PrimRange(prim):
                ignore_prim.add(child)
        if UsdLux.Light(prim) and prim not in ignore_prim:
            scale_mesh(prim, scale, add_missing_scale)
        if UsdGeom.Cube(prim):
            scale_cube(prim, scale, make_delta)
            for child in Usd.PrimRange(prim):
                ignore_prim.add(child)
        if UsdGeom.Sphere(prim):
            for child in Usd.PrimRange(prim):
                ignore_prim.add(child)
            scale_sphere(prim, scale, make_delta)
        if prim.GetTypeName() in ["Cylinder", "Cone", "Capsule"]:
            scale_cylinder(prim, scale, make_delta)
            for child in Usd.PrimRange(prim):
                ignore_prim.add(child)
        if UsdPhysics.RigidBodyAPI(prim):
            scale_rigid_body(prim, scale, make_delta)
        if UsdPhysics.MassAPI(prim):
            scale_mass(prim, scale, make_delta)
        if UsdPhysics.CollisionAPI(prim):
            scale_collision(prim, scale, make_delta)

        if UsdGeom.Camera(prim):
            scale_camera_params(prim, scale, make_delta)

    if stage_recursive:
        for sub_stage in sub_stages:
            ss = Usd.Stage.Open(sub_stage)
            set_stage_meters_per_unit(ss, new_mpu, stage_recursive, parent_stack, ignore_prim=ignore_prim)
            ss.Save()
        stage.Save()
