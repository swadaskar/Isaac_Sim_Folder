from math import radians
import omni
from pxr import UsdPhysics, UsdGeom, Gf, Usd, UsdShade
import omni.graph.core as og
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper_Properties
import numpy as np


class SurfaceGripperInternalState:
    def __init__(self):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._surface_gripper = Surface_Gripper(self._dc)
        self._initialized = False
        self._sgp = Surface_Gripper_Properties()

    def open(self):
        if self.update():
            self._surface_gripper.open()
            return False

    def close(self):
        if not self.update():
            return self._surface_gripper.close()
        return self.update()

    def update(self):
        self._surface_gripper.update()
        return self._surface_gripper.is_closed()

    def initialize(self):
        if not self._initialized or not self._surface_gripper.is_closed():
            self._surface_gripper.initialize(self._sgp)
            self._initialized = True

    def open(self):
        if self._initialized:
            self._surface_gripper.open()


class OgnSurfaceGripper:
    @staticmethod
    def internal_state():
        return SurfaceGripperInternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.enabled and db.inputs.ParentRigidBody and db.inputs.GripPosition:
            if db.inputs.onStep:
                # update internal state properties
                stage = omni.usd.get_context().get_stage()
                parent = stage.GetPrimAtPath(db.inputs.ParentRigidBody.path)
                if parent:
                    grip_point = stage.GetPrimAtPath(db.inputs.GripPosition.path)
                    db.internal_state._sgp.parentPath = str(parent.GetPath())
                    db.internal_state._sgp.d6JointPath = db.internal_state._sgp.parentPath + "/d6FixedJoint"
                    db.internal_state._sgp.gripThreshold = db.inputs.GripThreshold
                    db.internal_state._sgp.forceLimit = db.inputs.ForceLimit
                    db.internal_state._sgp.torqueLimit = db.inputs.TorqueLimit
                    db.internal_state._sgp.bendAngle = radians(db.inputs.BendAngle)
                    db.internal_state._sgp.stiffness = db.inputs.Stiffness
                    db.internal_state._sgp.damping = db.inputs.Damping
                    db.internal_state._sgp.disableGravity = db.inputs.DisableGravity
                    db.internal_state._sgp.retryClose = db.inputs.RetryClose
                    # compute offset between parent and gripping point
                    parent_pose = omni.usd.get_world_transform_matrix(parent)
                    grip_pose = omni.usd.get_world_transform_matrix(grip_point)
                    offset = grip_pose * parent_pose.GetInverse()
                    tr = _dynamic_control.Transform()
                    t = offset.ExtractTranslation()
                    q = offset.ExtractRotationQuat()
                    tr.p = [t[0], t[1], t[2]]
                    qr = q.GetImaginary()
                    tr.r = [qr[0], qr[1], qr[2], q.GetReal()]

                    db.internal_state._sgp.offset = tr
                    db.internal_state.initialize()
                    db.outputs.GripBroken = db.outputs.Closed and not db.internal_state.update()
                    db.outputs.closed = db.internal_state._surface_gripper.is_closed()

                    if db.inputs.Close or db.state.Close:
                        db.outputs.Closed = db.internal_state.close()
                        db.state.Close = db.inputs.RetryClose and not db.outputs.Closed
                    if db.inputs.Open or db.state.Open:
                        db.outputs.Closed = db.internal_state.open()
                        db.state.Open = False
                    db.outputs.Closed = db.internal_state.update()
        else:
            db.internal_state.open()
