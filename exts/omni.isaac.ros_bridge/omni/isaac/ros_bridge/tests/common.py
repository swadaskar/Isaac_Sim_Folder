# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
import asyncio
import carb
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np


def create_joint_state(name, position, velocity=[], effort=[]):
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header

    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    js.name = name
    js.position = position
    js.velocity = velocity
    js.effort = effort
    return js


def set_translate(prim, new_loc):
    from pxr import Gf, UsdGeom

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
    from pxr import Gf, UsdGeom

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


async def wait_for_rosmaster():
    carb.log_warn("Waiting for rosmaster to start")
    import rosgraph

    tries = 0
    while True:
        if tries > 10:
            carb.log_warn(f"ROS master was not found after {tries} tries")
            return

        try:
            tries = tries + 1
            rosgraph.Master("/rostopic").getPid()
        except:
            carb.log_warn("ROS master is not running yet...")
            await asyncio.sleep(1.0)
            continue
        else:
            carb.log_warn("ROS master is running, continuing")
            break


async def bridge_rosmaster_connect():
    import rosgraph

    tries = 0
    while True:
        if tries > 100:
            carb.log_warn(f"ROS master was not found after {tries} tries")
            return
        if rosgraph.is_master_online():
            carb.log_warn(f"ROS master was found after {tries} tries")
            return
        else:
            await omni.kit.app.get_app().next_update_async()
            tries = tries + 1


async def add_cube(path, size, offset):
    from pxr import UsdPhysics, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)

    cubeGeom.CreateSizeAttr(size)
    cubeGeom.AddTranslateOp().Set(offset)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(cubePrim)

    return cubeGeom


async def add_carter():
    from pxr import Gf, PhysicsSchemaTools

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd")
    stage = omni.usd.get_context().get_stage()

    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))


async def add_carter_ros():
    from pxr import Gf, PhysicsSchemaTools

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Samples/ROS/Robots/Carter_ROS.usd")

    # Disabling cameras by default
    import omni.graph.core as og

    ros_cameras_graph_path = "/Carter/ROS_Cameras"
    og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_left.inputs:condition"), False)
    og.Controller.set(og.Controller.attribute(ros_cameras_graph_path + "/enable_camera_right.inputs:condition"), False)

    stage = omni.usd.get_context().get_stage()

    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))


async def add_franka():
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")


def fields_to_dtype(fields, point_step):
    """Convert a list of PointFields to a numpy record datatype."""
    DUMMY_FIELD_PREFIX = "__"

    from sensor_msgs.msg import PointField

    # mappings between PointField types and numpy types
    type_mappings = [
        (PointField.INT8, np.dtype("int8")),
        (PointField.UINT8, np.dtype("uint8")),
        (PointField.INT16, np.dtype("int16")),
        (PointField.UINT16, np.dtype("uint16")),
        (PointField.INT32, np.dtype("int32")),
        (PointField.UINT32, np.dtype("uint32")),
        (PointField.FLOAT32, np.dtype("float32")),
        (PointField.FLOAT64, np.dtype("float64")),
    ]
    pftype_to_nptype = dict(type_mappings)
    nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

    # sizes (in bytes) of PointField types
    pftype_sizes = {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }

    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list
