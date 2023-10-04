# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy as np

import omni.timeline
import omni.graph.core as og

from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.sensor import _sensor
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.sensor.ogn.OgnIsaacReadIMUDatabase import OgnIsaacReadIMUDatabase


class OgnIsaacReadIMUInternalState(BaseResetNode):
    def __init__(self):
        self.first = True
        self.init_rot = [0.0, 0.0, 0.0, 1.0]
        self._is = _sensor.acquire_imu_sensor_interface()
        self.imu_path = ""
        super().__init__(initialize=False)

    def custom_reset(self):
        self.first = True
        self.init_rot = [0.0, 0.0, 0.0, 1.0]
        self.imu_path = ""
        pass

    def init_compute(self):
        is_name_len = len(self.imu_path) - self.imu_path.rfind("/")

        dc = _dynamic_control.acquire_dynamic_control_interface()
        if (
            self._is.is_imu_sensor(self.imu_path)
            and dc.peek_object_type(self.imu_path[:-is_name_len]) != _dynamic_control.OBJECT_NONE
        ):
            self.initialized = True
            return True

        else:
            return False


class OgnIsaacReadIMU:
    """
         Node that returns IMU Sensor data
    """

    @staticmethod
    def internal_state():
        return OgnIsaacReadIMUInternalState()

    @staticmethod
    def compute(db) -> bool:

        state = db.internal_state

        if not state.initialized:
            if db.inputs.imuPrim.valid:
                state.imu_path = db.inputs.imuPrim.path
                result = state.init_compute()
                if not result:
                    db.outputs.linAcc = [0.0, 0.0, 0.0]
                    db.outputs.angVel = [0.0, 0.0, 0.0]
                    db.outputs.orientation = [0.0, 0.0, 0.0, 1.0]
                    db.log_error("Prim is not an Imu sensor or is not attached to a rigid body")
                    return False

            else:
                db.outputs.linAcc = [0.0, 0.0, 0.0]
                db.outputs.angVel = [0.0, 0.0, 0.0]
                db.outputs.orientation = [0.0, 0.0, 0.0, 1.0]
                db.log_warn("Invalid Imu sensor prim")
                return False

        else:
            readings = state._is.get_sensor_readings(state.imu_path)

            # next pass
            if state.first:
                state.init_rot = readings[-1]["orientation"]
                state.first = False

            else:
                b = readings[-1]["orientation"]

                # compute quaternion inverse + multiplication
                a = [-state.init_rot[i] for i in range(3)]
                a.append(state.init_rot[3])

                db.outputs.orientation = [
                    a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - b[1] * a[2],
                    a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - b[2] * a[0],
                    a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - b[0] * a[1],
                    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
                ]

            lin_acc_l = ["lin_acc_x", "lin_acc_y", "lin_acc_z"]
            ang_vel_l = ["ang_vel_x", "ang_vel_y", "ang_vel_z"]

            lin_acc_l = [float(readings[-1][x]) for x in lin_acc_l]
            ang_vel_l = [float(readings[-1][x]) for x in ang_vel_l]

            db.outputs.linAcc = lin_acc_l
            db.outputs.angVel = ang_vel_l
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacReadIMUDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
