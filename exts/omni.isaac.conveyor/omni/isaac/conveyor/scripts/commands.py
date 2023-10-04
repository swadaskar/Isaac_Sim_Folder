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


class CreateConveyorBelt(omni.kit.commands.Command):
    """Creates an Action graph containing the Conveyor Belt Node. Must be applied to a Rigid Body prim. 
    If the selected prim is not a rigid body, the node will attempt to apply the rigid body API to it.

    Typical usage example:

    .. code-block:: python

        result, prim  = omni.kit.commands.execute(
                "CreateConveyorBelt",
                prim_name="ConveyorActionGraph",
                conveyor_prim="/ConveyorBeltRigidBody"
            )
    """

    def __init__(self, prim_name: str = "ConveyorBeltGraph", conveyor_prim=None):
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = None
        self._conveyor_prim_selected = conveyor_prim is not None
        pass

    def do(self):
        if self._conveyor_prim is None:
            _selection = omni.usd.get_context().get_selection()
            selected_paths = _selection.get_selected_prim_paths()
            if selected_paths:
                self._conveyor_prim_selected = True
            else:
                selected_paths = [str(self._stage.GetDefaultPrim().GetPath())]
                self._conveyor_prim = self._stage.GetDefaultPrim()
        else:
            selected_paths = [str(self._conveyor_prim.GetPath())]

            # self._prim_path = str(self._stage.GetDefaultPrim().GetPath().AppendChild("ConveyorsGraph"))
        keys = og.Controller.Keys

        for selected_path in selected_paths:
            # self._conveyor_prim_selected = True
            self._conveyor_prim = self._stage.GetPrimAtPath(selected_path)
            if self._conveyor_prim_selected == True and not UsdPhysics.RigidBodyAPI(self._conveyor_prim):
                alt_conveyor = self._conveyor_prim.GetParent()
                while alt_conveyor:
                    if UsdPhysics.RigidBodyAPI(alt_conveyor):
                        self._conveyor_prim = alt_conveyor
                        break
                    alt_conveyor = alt_conveyor.GetParent()
                if not alt_conveyor:
                    UsdPhysics.RigidBodyAPI.Apply(self._conveyor_prim)
                    UsdPhysics.CollisionAPI.Apply(self._conveyor_prim)
            base_path = self._conveyor_prim.GetPath()
            if self._conveyor_prim != self._stage.GetDefaultPrim():
                base_path = self._conveyor_prim.GetParent().GetPath()
            self._prim_path = omni.usd.get_stage_next_free_path(
                self._stage, base_path.AppendChild(pxr.Tf.MakeValidIdentifier(self._prim_name)), True
            )
            conveyor_node_name = "ConveyorNode"
            og.Controller.edit(
                {"graph_path": self._prim_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnPlaybackTick"),
                        (conveyor_node_name, "omni.isaac.conveyor.IsaacConveyor"),
                    ],
                    keys.SET_VALUES: [],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "{}.inputs:onStep".format(conveyor_node_name)),
                        ("OnTick.outputs:deltaSeconds", "{}.inputs:delta".format(conveyor_node_name)),
                    ],
                },
            )
            if self._conveyor_prim_selected:
                conveyor_node = self._stage.GetPrimAtPath(self._prim_path + "/" + conveyor_node_name)
                input_rel = conveyor_node.GetRelationship("inputs:conveyorPrim")
                if not input_rel:
                    input_rel = conveyor_node.CreateRelationship("inputs:conveyorPrim")
                input_rel.SetTargets([self._conveyor_prim.GetPath()])

            self._prim = self._stage.GetPrimAtPath(self._prim_path + "/" + conveyor_node_name)
        return self._prim

    def undo(self):
        if self._prim:
            return self._stage.RemovePrim(self._prim_path)
        pass


omni.kit.commands.register_all_commands_in_module(__name__)
