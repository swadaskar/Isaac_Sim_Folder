# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
import pxr
import omni.graph.core as og
from pxr import UsdPhysics
import carb
from pxr import Usd, UsdGeom


class CreateSurfaceGripper(omni.kit.commands.Command):
    """Creates Action graph containing a Surface Gripper node, and all prims to facilitate its creation

    Typical usage example:

    .. code-block:: python

        result, prim  = omni.kit.commands.execute(
                "CreateSurfaceGripper",
                prim_name="SurfaceGripperActionGraph",
                conveyor_prim="/SurfaceGripperRigidBody"
            )
    """

    def __init__(self, prim_name: str = "SurfaceGripperActionGraph", surface_gripper_prim=None):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = None
        self._surface_gripper_prim_selected = surface_gripper_prim is not None
        pass

    def do(self):
        if self._surface_gripper_prim is None:
            _selection = omni.usd.get_context().get_selection()
            selected_paths = _selection.get_selected_prim_paths()
            self._surface_gripper_prim = self._stage.GetDefaultPrim()
            self._surface_gripper_prim_selected = False
            if selected_paths:
                self._surface_gripper_prim_selected = True
                self._surface_gripper_prim = self._stage.GetPrimAtPath(selected_paths[0])
                if not UsdPhysics.RigidBodyAPI(self._surface_gripper_prim):
                    carb.log_warn("Selected prim is not a rigid body. ignoring selection.")
                    self._surface_gripper_prim_selected = False
                    # alt = self._surface_gripper_prim.GetParent()
                    # while alt:
                    #     if UsdPhysics.RigidBodyAPI(alt):
                    #         self._surface_gripper_prim = alt
                    #         break
                    #     alt = alt.GetParent()
                    # if not alt:
                    #     UsdPhysics.RigidBodyAPI.Apply(self._surface_gripper_prim)
                    #     UsdPhysics.CollisionAPI.Apply(self._surface_gripper_prim)
        self._prim_path = omni.usd.get_stage_next_free_path(
            self._stage,
            self._surface_gripper_prim.GetPath().AppendChild(pxr.Tf.MakeValidIdentifier(self._prim_name)),
            True,
        )
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": self._prim_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("SurfaceGripperNode", "omni.isaac.surface_gripper.SurfaceGripper"),
                ],
                keys.SET_VALUES: [],
                keys.CONNECT: [("OnTick.outputs:tick", "SurfaceGripperNode.inputs:onStep")],
            },
        )
        if self._surface_gripper_prim_selected:
            surface_gripper_node = self._stage.GetPrimAtPath(self._prim_path + "/SurfaceGripperNode")
            surface_gripper_offset = UsdGeom.Xform.Define(self._stage, self._prim_path + "/SurfaceGripperOffset")
            surface_gripper_offset.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
            surface_gripper_offset.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
            surface_gripper_offset_prim = surface_gripper_offset.GetPrim()
            input_rel = surface_gripper_node.GetRelationship("inputs:ParentRigidBody")
            if not input_rel:
                input_rel = surface_gripper_node.CreateRelationship("inputs:ParentRigidBody")
            input_rel.SetTargets([self._surface_gripper_prim.GetPath()])
            input_rel = surface_gripper_node.GetRelationship("inputs:GripPosition")
            input_rel.SetTargets([surface_gripper_offset_prim.GetPath()])

        self._prim = self._stage.GetPrimAtPath(self._prim_path)
        return self._prim

    def undo(self):
        if self._prim:
            return self._stage.RemovePrim(self._prim_path)
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
