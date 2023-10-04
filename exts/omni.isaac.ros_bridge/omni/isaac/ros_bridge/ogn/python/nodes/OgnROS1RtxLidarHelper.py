import omni
import carb
import omni.syntheticdata
import omni.graph.core as og
import traceback
from pxr import Usd, UsdGeom
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
from omni.isaac.core.utils.render_product import get_camera_prim_path
import omni.replicator.core as rep
from omni.isaac.core_nodes import BaseWriterNode, WriterRequest


class OgnROS1RtxLidarHelperInternalState(BaseWriterNode):
    def __init__(self):
        self.viewport = None
        self.viewport_name = ""
        self.resetSimulationTimeOnStop = False
        super().__init__(initialize=False)

    def post_attach(self, writer, render_product):
        try:
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except:
            pass


class OgnROS1RtxLidarHelper:
    @staticmethod
    def internal_state():
        return OgnROS1RtxLidarHelperInternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.internal_state.initialized is False:
            db.internal_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            keys = og.Controller.Keys
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                render_product_path = db.inputs.renderProductPath
                if not render_product_path:
                    carb.log_warn("Render product not valid")
                    db.internal_state.initialized = False
                    return False
                if stage.GetPrimAtPath(render_product_path) is None:
                    # Invalid Render Product Path
                    carb.log_warn("Render product not created yet, retrying on next call")
                    db.internal_state.initialized = False
                    return False
                else:
                    prim = stage.GetPrimAtPath(get_camera_prim_path(render_product_path))
                    if prim.IsA(UsdGeom.Camera):
                        if prim.HasAPI(IsaacSensorSchema.IsaacRtxLidarSensorAPI):
                            db.internal_state.render_product_path = render_product_path
                            db.internal_state.sensor = "lidar"
                        else:
                            db.internal_state.sensor = None

                if db.internal_state.sensor is None:
                    carb.log_warn("Active camera for Render product is not an RTX Lidar")
                    db.internal_state.initialized = False
                    return False

                db.internal_state.render_product_path = render_product_path
                sensor_type = db.inputs.type
                db.internal_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
                writer = None
                try:
                    if sensor_type == "laser_scan":
                        writer = rep.writers.get("RtxLidar" + "ROS1PublishLaserScan")

                    elif sensor_type == "point_cloud":
                        writer = rep.writers.get("RtxLidar" + "ROS1PublishPointCloud")

                    else:
                        carb.log_error("type is not supported")
                        db.internal_state.initialized = False
                        return False
                    if writer is not None:
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                        )
                        db.internal_state.append_writer(writer)
                    db.internal_state.attach_writers(render_product_path)
                except Exception as e:
                    print(traceback.format_exc())
                    pass
        else:
            return True

    @staticmethod
    def release(node):
        try:
            state = OgnROS1RtxLidarHelperInternalState.per_node_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
