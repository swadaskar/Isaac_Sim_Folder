# Copyright (c) 2021-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.kit.commands
import omni.ui
import gc
import weakref
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from PIL import Image
import asyncio
from omni.isaac.core.utils.stage import set_stage_up_axis
from omni.isaac.core import PhysicsContext
from omni.physx.scripts.physicsUtils import add_ground_plane
from pxr import UsdGeom, Gf, UsdLux, Sdf, UsdPhysics
import omni.ui as ui
import carb
import os

from omni.isaac.ui.ui_utils import float_builder, btn_builder

EXTENSION_NAME = "Block World Generator"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._window = omni.ui.Window(EXTENSION_NAME, width=600, height=400, visible=False)
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self._filepicker = None
        self._visualize_window = None
        with self._window.frame:
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=5, height=0):
                    self._cell_size = float_builder(label="Cell Size", default_val=0.05)
                    self._load_button = btn_builder(label="Image", text="Load", on_clicked_fn=self._load_image_dialog)
                    self._generate_button = btn_builder(
                        label="Block World", text="Generate", on_clicked_fn=self._generate
                    )
                    self._generate_button.enabled = False

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._window = None
        self._visualize_window = None
        self._filepicker = None
        gc.collect()

    def _load_image_dialog(self):
        from omni.kit.window.filepicker import FilePickerDialog
        from omni.kit.widget.filebrowser import FileBrowserItem

        def _on_filter_png_files(item: FileBrowserItem) -> bool:
            """Callback to filter the choices of file names in the open or save dialog"""
            if not item or item.is_folder:
                return True
            # Show only files with listed extensions
            return os.path.splitext(item.path)[1] == ".png"

        self._filepicker = FilePickerDialog(
            "Load .png image",
            allow_multi_selection=False,
            apply_button_label="Load",
            click_apply_handler=self._load_image,
            item_filter_options=[".png Files (*.png, *.PNG)"],
            item_filter_fn=_on_filter_png_files,
        )

    def _load_image(self, file, folder):
        _path = folder + "/" + file
        self._filepicker.hide()
        if _path == "":
            # carb.log_error("File path can't be empty.")
            carb.log_warn("File path can't be empty.")
        else:
            print("Opening file at ", _path)
            self._image = Image.open(_path).convert("RGBA")
            # self._image = Image.open(_path)
            # Visualize image
            # print("Image Band: ", self._image.getbands())
            print("Image Size: ", self._image.size)
            self._cols, self._rows = self._image.size
            self._visualize_window = omni.ui.Window("Visualization", width=300, height=300)
            with self._visualize_window.frame:
                self._rgb_byte_provider = omni.ui.ByteImageProvider()
                self._rgb_byte_provider.set_bytes_data(
                    list(self._image.tobytes("raw", "RGBA")), [self._cols, self._rows]
                )
                with omni.ui.VStack():
                    omni.ui.ImageWithProvider(self._rgb_byte_provider)
            self._generate_button.enabled = True

    def _generate(self):
        asyncio.ensure_future(self._create_block_world())

    async def _create_block_world(self):
        # Wait for new stage before creating objects for block world
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Constant declarations
        kScale = self._cell_size.get_value_as_float()
        kOffset = kScale / 2.0
        kCubeColor = (0.5, 0.5, 0.5)
        kCubeHeight = 2.00
        kGroundPlaneSize = max(self._cols, self._rows) * kScale / 2.0 + 1.00
        kGroundPlanePosition = Gf.Vec3f((self._cols * kScale / 2), -(self._rows * kScale / 2), 0.0)

        # Set up ground plane and physics scene
        self._stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        set_stage_up_axis("z")
        add_ground_plane(self._stage, "/World/groundPlane", "Z", kGroundPlaneSize, kGroundPlanePosition, Gf.Vec3f(1.0))
        PhysicsContext(physics_dt=1.0 / 60.0)

        # Set up distant light
        light_prim = UsdLux.DistantLight.Define(self._stage, Sdf.Path("/World/defaultLight"))
        light_prim.CreateIntensityAttr(2000)
        self._stage.SetDefaultPrim(self._stage.GetPrimAtPath("/World"))

        homePath = "/World/occupancyMap"
        instancePath = homePath + "/occupiedInstances"
        cubePath = instancePath + "/occupiedCube"
        pos_list = []

        if self._stage.GetPrimAtPath(homePath):
            self._stage.RemovePrim(homePath)
        homeXform = UsdGeom.Xform(self._stage.DefinePrim(homePath, "Xform"))
        homeXform.AddTranslateOp().Set(Gf.Vec3d(0, 0, kOffset))

        if self._stage.GetPrimAtPath(instancePath):
            self._stage.RemovePrim(instancePath)
        point_instancer = UsdGeom.PointInstancer(self._stage.DefinePrim(instancePath, "PointInstancer"))
        point_instancer.AddScaleOp().Set(Gf.Vec3f(1))
        point_instancer.AddTranslateOp().Set(Gf.Vec3f(0, 0, (kCubeHeight / 2.0)))

        # UsdPhysics.RigidBodyAPI.Apply(self._stage.GetPrimAtPath(instancePath))

        if self._stage.GetPrimAtPath(cubePath):
            self._stage.RemovePrim(cubePath)
        occupiedCube = UsdGeom.Cube(self._stage.DefinePrim(cubePath, "Cube"))
        occupiedCube.AddScaleOp().Set(Gf.Vec3f(kScale, kScale, kCubeHeight))
        occupiedCube.CreateSizeAttr(1.0)
        occupiedCube.CreateDisplayColorPrimvar().Set([kCubeColor])
        UsdPhysics.CollisionAPI.Apply(self._stage.GetPrimAtPath(cubePath))

        positions_attr = point_instancer.CreatePositionsAttr()
        point_instancer.CreatePrototypesRel().SetTargets([occupiedCube.GetPath()])
        proto_indices_attr = point_instancer.CreateProtoIndicesAttr()
        kFilledPixelThres = 127
        for x in range(self._cols):
            for y in range(self._rows):
                if self._image.getpixel((x, y))[0] < kFilledPixelThres:
                    world_x = (x * kScale) + kOffset
                    world_y = -((y * kScale) + kOffset)
                    pos_list.append(Gf.Vec3f(world_x, world_y, 0))
        positions_attr.Set(pos_list)
        proto_indices_attr.Set([0] * len(pos_list))
        print("Total blocks drawn: ", len(pos_list))
