# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy

import omni.timeline
import omni.graph.core as og

from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.sensor import _sensor
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.sensor.ogn.OgnIsaacReadContactSensorDatabase import OgnIsaacReadContactSensorDatabase


class OgnIsaacReadContactSensorInternalState(BaseResetNode):
    def __init__(self):
        self._cs = _sensor.acquire_contact_sensor_interface()
        self.cs_path = ""
        super().__init__(initialize=False)

    def custom_reset(self):
        self.cs_path = ""
        pass

    def init_compute(self):
        cs_name_len = len(self.cs_path) - self.cs_path.rfind("/")

        dc = _dynamic_control.acquire_dynamic_control_interface()
        if (
            self._cs.is_contact_sensor(self.cs_path)
            and dc.peek_object_type(self.cs_path[:-cs_name_len]) != _dynamic_control.OBJECT_NONE
        ):
            self.initialized = True
            return True
        else:
            return False


class OgnIsaacReadContactSensor:
    """   
         Node that returns Contact Sensor data
    """

    @staticmethod
    def internal_state():
        return OgnIsaacReadContactSensorInternalState()

    @staticmethod
    def compute(db) -> bool:

        state = db.internal_state

        if not state.initialized:
            if db.inputs.csPrim.valid:
                state.cs_path = db.inputs.csPrim.path

                result = state.init_compute()
                if not result:
                    db.outputs.inContact = False
                    db.outputs.value = 0
                    db.log_error("Prim is not a contact sensor or is not attached to rigid body")
                    return False

                readings = state._cs.get_sensor_readings(state.cs_path)

                db.outputs.inContact = readings[-1]["inContact"]
                db.outputs.value = readings[-1]["value"]
            else:
                db.outputs.inContact = False
                db.outputs.value = 0
                db.log_warn("Invalid contact sensor prim")
                return False
            return True

        readings = state._cs.get_sensor_readings(state.cs_path)

        db.outputs.inContact = readings[-1]["inContact"]
        db.outputs.value = readings[-1]["value"]

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacReadContactSensorDatabase.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
