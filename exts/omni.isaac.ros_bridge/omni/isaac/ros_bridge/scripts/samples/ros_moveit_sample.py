# Copyright (c) 2021-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import omni.kit.commands
import omni.ext
import omni.appwindow
import weakref
import gc
import asyncio
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description

from pxr import Gf

from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.graph.core as og

MENU_NAME = "MoveIt"
FRANKA_STAGE_PATH = "/Franka"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._window = None

        menu_items = [make_menu_item_description(ext_id, MENU_NAME, lambda a=weakref.proxy(self): a._menu_callback())]
        self._menu_items = [MenuItemDescription(name="ROS", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Examples")

    def _menu_callback(self):
        self._on_environment_setup()

    def create_ros_action_graph(self, franka_stage_path):
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                        ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                        ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                        ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Setting the /Franka target prim to Articulation Controller node
                        ("ArticulationController.inputs:usePath", True),
                        ("ArticulationController.inputs:robotPath", franka_stage_path),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Setting the /Franka target prim to Publish JointState node
        set_target_prims(primPath="/ActionGraph/PublishJointState", targetPrimPaths=[franka_stage_path])

        # Setting the /Franka target prim to Publish Transform Tree node
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:targetPrims", targetPrimPaths=[franka_stage_path]
        )

        pass

    def create_franka(self, stage_path):
        usd_path = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        asset_path = self._assets_root_path + usd_path
        prim = self._stage.DefinePrim(stage_path, "Xform")
        prim.GetReferences().AddReference(asset_path)
        rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=None,
            new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(0, -0.64, 0)),
        )

        pass

    async def _create_moveit_sample(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        set_camera_view(eye=[1.20, 1.20, 0.80], target=[0, 0, 0.50], camera_prim_path="/OmniverseKit_Persp")

        self._stage = self._usd_context.get_stage()

        self.create_franka(FRANKA_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()
        create_prim(
            prim_path="/background", usd_path=self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )
        await omni.kit.app.get_app().next_update_async()
        PhysicsContext(physics_dt=1.0 / 60.0)
        await omni.kit.app.get_app().next_update_async()
        self.create_ros_action_graph(FRANKA_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

    def _on_environment_setup(self):

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        asyncio.ensure_future(self._create_moveit_sample())

    def on_shutdown(self):
        """Cleanup objects on extension shutdown"""
        self._timeline.stop()
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
        gc.collect()
