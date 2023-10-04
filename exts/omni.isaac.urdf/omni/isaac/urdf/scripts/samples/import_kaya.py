# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni
import omni.kit.commands
import asyncio
import math
import weakref
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description

from .common import set_drive_parameters
from pxr import UsdLux, Sdf, Gf, UsdPhysics

from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder
from omni.kit.viewport.utility.camera_state import ViewportCameraState

EXTENSION_NAME = "Import Kaya"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)

        self._menu_items = [
            MenuItemDescription(
                name="Import Robots",
                sub_menu=[
                    make_menu_item_description(ext_id, "Kaya URDF", lambda a=weakref.proxy(self): a._menu_callback())
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

        self._build_ui()

    def _build_ui(self):
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=0, height=0, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                title = "Import a Kaya Robot via URDF"
                doc_link = "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_urdf.html"

                overview = "This Example shows you import an NVIDIA Kaya robot via URDF.\n\nPress the 'Open in IDE' button to view the source code."

                setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

                frame = ui.CollapsableFrame(
                    title="Command Panel",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5):
                        dict = {
                            "label": "Load Robot",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Load a UR10 Robot into the Scene",
                            "on_clicked_fn": self._on_load_robot,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Configure Drives",
                            "type": "button",
                            "text": "Configure",
                            "tooltip": "Configure Joint Drives",
                            "on_clicked_fn": self._on_config_robot,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Spin Robot",
                            "type": "button",
                            "text": "move",
                            "tooltip": "Spin the Robot in Place",
                            "on_clicked_fn": self._on_config_drives,
                        }
                        btn_builder(**dict)

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_load_robot(self):
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._load_kaya(load_stage))

    async def _load_kaya(self, task):
        done, pending = await asyncio.wait({task})
        if task in done:
            status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            import_config.merge_fixed_joints = True
            import_config.import_inertia_tensor = False
            # import_config.distance_scale = 1.0
            import_config.fix_base = False
            import_config.make_default_prim = True
            import_config.create_physics_scene = True
            omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=self._extension_path + "/data/urdf/robots/kaya/urdf/kaya.urdf",
                import_config=import_config,
            )

            camera_state = ViewportCameraState("/OmniverseKit_Persp")
            camera_state.set_position_world(Gf.Vec3d(-1.0, 1.5, 0.5), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.0), True)

            stage = omni.usd.get_context().get_stage()
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)

            result, plane_path = omni.kit.commands.execute(
                "AddGroundPlaneCommand",
                stage=stage,
                planePath="/groundPlane",
                axis="Z",
                size=1500.0,
                position=Gf.Vec3f(0, 0, -0.25),
                color=Gf.Vec3f(0.5),
            )
            # make sure the ground plane is under root prim and not robot
            omni.kit.commands.execute(
                "MovePrimCommand", path_from=plane_path, path_to="/groundPlane", keep_world_transform=True
            )

            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)

    def _on_config_robot(self):
        stage = omni.usd.get_context().get_stage()
        # Make all rollers spin freely by removing extra drive API
        for axle in range(0, 2 + 1):
            for ring in range(0, 1 + 1):
                for roller in range(0, 4 + 1):
                    prim_path = (
                        "/kaya/axle_"
                        + str(axle)
                        + "/roller_"
                        + str(axle)
                        + "_"
                        + str(ring)
                        + "_"
                        + str(roller)
                        + "_joint"
                    )
                    prim = stage.GetPrimAtPath(prim_path)
                    omni.kit.commands.execute(
                        "UnapplyAPISchemaCommand",
                        api=UsdPhysics.DriveAPI,
                        prim=prim,
                        api_prefix="drive",
                        multiple_api_token="angular",
                    )

    def _on_config_drives(self):
        self._on_config_robot()  # make sure drives are configured first
        stage = omni.usd.get_context().get_stage()
        # set each axis to spin at a rate of 1 rad/s
        axle_0 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/kaya/base_link/axle_0_joint"), "angular")
        axle_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/kaya/base_link/axle_1_joint"), "angular")
        axle_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/kaya/base_link/axle_2_joint"), "angular")

        set_drive_parameters(axle_0, "velocity", math.degrees(1), 0, math.radians(1e7))
        set_drive_parameters(axle_1, "velocity", math.degrees(1), 0, math.radians(1e7))
        set_drive_parameters(axle_2, "velocity", math.degrees(1), 0, math.radians(1e7))
