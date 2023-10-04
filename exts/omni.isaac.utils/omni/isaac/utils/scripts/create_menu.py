# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import omni.ext
import omni.kit.commands
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
import carb
import gc
import weakref


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):

        robot_menu = []

        menu_universal_robots = [
            make_menu_item_description(
                ext_id,
                "UR3",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur3/ur3.usd", "/UR3"),
            ),
            make_menu_item_description(
                ext_id,
                "UR5",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur5/ur5.usd", "/UR5"),
            ),
            make_menu_item_description(
                ext_id,
                "UR10",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur10/ur10.usd", "/UR10"),
            ),
            make_menu_item_description(
                ext_id,
                "UR3e",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd", "/UR3e"),
            ),
            make_menu_item_description(
                ext_id,
                "UR5e",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd", "/UR5e"),
            ),
            make_menu_item_description(
                ext_id,
                "UR10e",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "/UR10e"),
            ),
            make_menu_item_description(
                ext_id,
                "UR16e",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd", "/UR16e"),
            ),
        ]

        menu_denso = [
            make_menu_item_description(
                ext_id,
                "Cobotta Pro 900",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Robots/Denso/cobotta_pro_900.usd", "/Cobotta_Pro_900"
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Cobotta Pro 1300",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Robots/Denso/cobotta_pro_1300.usd", "/Cobotta_Pro_1300"
                ),
            ),
        ]

        menu_manipulators = [
            MenuItemDescription(header="Manipulators"),
            make_menu_item_description(
                ext_id,
                "Dofbot",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Dofbot/dofbot.usd", "/Dofbot"),
            ),
            MenuItemDescription(name="Denso", sub_menu=menu_denso),
            make_menu_item_description(
                ext_id,
                "Franka",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Franka/franka_alt_fingers.usd", "/Franka"),
            ),
            MenuItemDescription(name="Univeral Robots", sub_menu=menu_universal_robots),
        ]

        menu_unitree = [
            make_menu_item_description(
                ext_id, "A1", lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Unitree/a1.usd", "/A1")
            ),
            make_menu_item_description(
                ext_id, "Go1", lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Unitree/go1.usd", "/Go1")
            ),
        ]

        menu_quadrupeds = [
            MenuItemDescription(header="Quadrupeds"),
            make_menu_item_description(
                ext_id,
                "ANYmal C",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Unitree/anymal_c.usd", "/ANYmal_C"),
            ),
            MenuItemDescription(name="Unitree", sub_menu=menu_unitree),
            # MenuItemDescription(name="ANYbotics", sub_menu=menu_anybotics), # for some reason, the header needed to be above a item, not submenu, to show up
        ]

        menu_quadrotors = [
            MenuItemDescription(header="Quadcopters"),
            make_menu_item_description(
                ext_id,
                "Crazyflie",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Crazyflie/cf2x.usd", "/Crazyflie"),
            ),
            make_menu_item_description(
                ext_id,
                "Quadcopter",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Quadcopter/quadcopter.usd", "/Quadcopter"),
            ),
        ]

        menu_nvidia = [
            make_menu_item_description(
                ext_id,
                "Carter V1",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Carter/carter_v1.usd", "/Carter"),
            ),
            make_menu_item_description(
                ext_id,
                "Carter V2",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Carter/carter_v2.usd", "/Carter"),
            ),
            make_menu_item_description(
                ext_id,
                "Jetbot",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Jetbot/jetbot.usd", "/Jetbot"),
            ),
            make_menu_item_description(
                ext_id,
                "Jetracer",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Jetracer/jetracer.usd", "/Jetracer"),
            ),
            make_menu_item_description(
                ext_id, "Kaya", lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/Kaya/kaya.usd", "/Kaya")
            ),
        ]

        menu_mobile = [
            MenuItemDescription(header="Wheeled Robots"),
            MenuItemDescription(name="NVIDIA", sub_menu=menu_nvidia),
            make_menu_item_description(
                ext_id,
                "Transporter",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Robots/Transporter/transporter_sensors.usd", "/Transporter"
                ),
            ),
        ]

        robot_menu += menu_manipulators
        robot_menu += menu_quadrupeds
        robot_menu += menu_quadrotors
        robot_menu += menu_mobile

        env_menu = [
            MenuItemDescription(header="Basic"),
            make_menu_item_description(
                ext_id,
                "Flat Grid",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Grid/default_environment.usd", "/FlatGrid"
                ),
            ),
            MenuItemDescription(header="Rooms"),
            make_menu_item_description(
                ext_id,
                "Grid Room",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Grid/gridroom_curved.usd", "/GridRoom"
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Simple Room",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Simple_Room/simple_room.usd", "/SimpleRoom", [3.15, 3.15, 2.0], [0, 0, 0]
                ),
            ),
            MenuItemDescription(header="Warehouse"),
            make_menu_item_description(
                ext_id,
                "Small Warehouse",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/Warehouse"
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Small Warehouse With Multiple Shelves",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd", "/Warehouse"
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Full Warehouse",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/Warehouse"
                ),
            ),
            MenuItemDescription(header="Architectural"),
            make_menu_item_description(
                ext_id,
                "Hospital",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Hospital/hospital.usd", "/Hospital", [7.35, -1.5, 2.3], [0, 0, 0]
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Office",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Environments/Office/office.usd", "/Office", [3.15, 3.15, 2.0], [0, 0, 0]
                ),
            ),
        ]
        apriltag_menu = [
            make_menu_item_description(
                ext_id,
                "tag36h11",
                lambda a=weakref.proxy(self): a.create_apriltag(
                    "/Materials/AprilTag/AprilTag.mdl",
                    "AprilTag",
                    "/Looks/AprilTag",
                    "/Materials/AprilTag/Textures/tag36h11.png",
                ),
            )
        ]

        menu_end_effectors = [
            MenuItemDescription(header="Hands"),
            make_menu_item_description(
                ext_id,
                "Allegro Hand",
                lambda a=weakref.proxy(self): a.create_asset(
                    "/Isaac/Robots/AllegroHand/allegro_hand.usd", "/AllegroHand"
                ),
            ),
            make_menu_item_description(
                ext_id,
                "Shadow Hand",
                lambda a=weakref.proxy(self): a.create_asset("/Isaac/Robots/ShadowHand/shadow_hand.usd", "/ShadowHand"),
            ),
        ]

        self._menu_items = [
            MenuItemDescription(
                name="Isaac",
                glyph="plug.svg",
                sub_menu=[
                    MenuItemDescription(name="Robots", sub_menu=robot_menu),
                    MenuItemDescription(name="End Effectors", sub_menu=menu_end_effectors),
                    MenuItemDescription(name="Environments", sub_menu=env_menu),
                    MenuItemDescription(name="April Tag", sub_menu=apriltag_menu),
                ],
            )
        ]
        add_menu_items(self._menu_items, "Create")

    def create_asset(self, usd_path, stage_path, camera_position=None, camera_target=None):

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to=stage_path,
            asset_path=self._assets_root_path + usd_path,
            instanceable=False,
        )
        if camera_position is not None and camera_target is not None:
            set_camera_view(camera_position, camera_target)

        pass

    def create_apriltag(self, usd_path, shader_name, stage_path, tag_path):
        from pxr import Sdf

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        stage = omni.usd.get_context().get_stage()
        stage_path = omni.usd.get_stage_next_free_path(stage, stage_path, False)

        async def create_tag():
            omni.kit.commands.execute(
                "CreateMdlMaterialPrim",
                mtl_url=self._assets_root_path + usd_path,
                mtl_name=shader_name,
                mtl_path=stage_path,
                select_new_prim=True,
            )
            mtl = stage.GetPrimAtPath(stage_path + "/Shader")
            # it can take multiple frames after mdl is created to be able to set a property
            while mtl.GetAttribute("inputs:tag_mosaic").Get() is None:
                await omni.kit.app.get_app().next_update_async()
                omni.kit.commands.execute(
                    "ChangeProperty",
                    prop_path=Sdf.Path(stage_path + "/Shader.inputs:tag_mosaic"),
                    value=Sdf.AssetPath(self._assets_root_path + tag_path),
                    prev=None,
                )

        asyncio.ensure_future(create_tag())

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Create")
        gc.collect()
