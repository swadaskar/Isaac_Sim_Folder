# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gc
import os
import weakref
import asyncio
import carb
import omni
import omni.ext
import omni.ui as ui
from pxr import UsdGeom, Gf, Usd
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from .. import _occupancy_map
from .utils import update_location, compute_coordinates, generate_image
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.ui.ui_utils import (
    xyz_builder,
    float_builder,
    color_picker_builder,
    dropdown_builder,
    multi_btn_builder,
    btn_builder,
)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        EXTENSION_NAME = "Occupancy Map"
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = omni.ui.Window(EXTENSION_NAME, width=600, height=400, visible=False)
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self._om = _occupancy_map.acquire_occupancy_map_interface()
        self._layers = omni.usd.get_context().get_layers()
        self._filepicker = None
        self._models = {}

        self.prev_origin = [0, 0]
        self.lower_bound = [-1.00, -1.00]
        self.upper_bound = [1.00, 1.00]

        self.wait_bound_update = False
        self.bound_update_case = 0

        with self._window.frame:
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=5, height=0):
                    change_fn = [self.on_update_location, self.on_update_location, self.on_update_location]
                    self._models["origin"] = xyz_builder(label="Origin", on_value_changed_fn=change_fn)

                    self._models["upper_bound"] = xyz_builder(
                        label="Upper Bound",
                        on_value_changed_fn=change_fn,
                        default_val=[self.upper_bound[0], self.upper_bound[1], 0],
                    )
                    self._models["lower_bound"] = xyz_builder(
                        label="Lower Bound",
                        on_value_changed_fn=change_fn,
                        default_val=[self.lower_bound[0], self.lower_bound[1], 0],
                    )

                    self._models["center_bound"] = multi_btn_builder(
                        "Positioning",
                        text=["Center to Selection", "Bound Selection"],
                        on_clicked_fn=[self._on_center_selection, self._on_bound_selection],
                    )

                    self._models["cell_size"] = float_builder(
                        label="Cell Size",
                        default_val=0.05,
                        tooltip="Size of each pixel in stage units in output occupancy map image",
                    )
                    self._models["cell_size"].add_value_changed_fn(self.on_update_cell_size)
                    self._models["compute"] = multi_btn_builder(
                        "Occupancy Map",
                        text=["Calculate", "Visualize Image"],
                        on_clicked_fn=[self._generate_map, self._generate_image],
                    )

                    # self.draw_voxel_btn = ui.Button("Draw Voxels", clicked_fn=self._draw_instances)
                    # self.draw_voxel_btn.visible = False

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_center_selection(self):
        origin = self.calculate_bounds(True, True)

        self._models["origin"][0].set_value(origin[0])
        self._models["origin"][1].set_value(origin[1])

        self.lower_bound, self.upper_bound = self.calculate_bounds(False, True)
        self.set_bound_value_ui()

    def calculate_bounds(self, origin_calc, stationary_bounds):
        origin_coord = [self._models["origin"][0].get_value_as_float(), self._models["origin"][1].get_value_as_float()]

        if not origin_calc and stationary_bounds:
            lower_bound = [
                self.lower_bound[0] + self.prev_origin[0] - origin_coord[0],
                self.lower_bound[1] + self.prev_origin[1] - origin_coord[1],
            ]

            upper_bound = [
                self.upper_bound[0] + self.prev_origin[0] - origin_coord[0],
                self.upper_bound[1] + self.prev_origin[1] - origin_coord[1],
            ]
            return lower_bound, upper_bound

        selected_prims = omni.usd.get_context().get_selection().get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bbox_cache.Clear()
        total_bounds = Gf.BBox3d()

        if len(selected_prims) > 0:
            for prim_path in selected_prims:
                prim = stage.GetPrimAtPath(prim_path)
                bounds = bbox_cache.ComputeWorldBound(prim)
                total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
            range = total_bounds.GetBox()
            mid_point = range.GetMidpoint()
            if origin_calc:
                self.prev_origin = origin_coord
                origin_value = mid_point
                return origin_value

            min_point = range.GetMin()
            max_point = range.GetMax()

            lower_bound = [None] * 2
            upper_bound = [None] * 2

            lower_bound[0] = min_point[0] - origin_coord[0]
            lower_bound[1] = min_point[1] - origin_coord[1]

            upper_bound[0] = max_point[0] - origin_coord[0]
            upper_bound[1] = max_point[1] - origin_coord[1]

            return lower_bound, upper_bound
        else:
            if origin_calc:
                return [0] * 2
        return [0] * 2, [0] * 2

    def set_bound_value_ui(self):
        self.wait_bound_update = True
        self.bound_update_case = 0
        self._models["lower_bound"][0].set_value(self.lower_bound[0])

        # Updating Case every time bound value is updating
        self.bound_update_case += 1

        self._models["lower_bound"][1].set_value(self.lower_bound[1])

        self.bound_update_case += 1

        self._models["upper_bound"][0].set_value(self.upper_bound[0])

        self.bound_update_case += 1

        self._models["upper_bound"][1].set_value(self.upper_bound[1])

        self.wait_bound_update = False

    def _on_bound_selection(self):
        self.lower_bound, self.upper_bound = self.calculate_bounds(False, False)
        self.set_bound_value_ui()

    def on_update_location(self, value):
        if (
            self._models["lower_bound"][0].get_value_as_float() >= self._models["upper_bound"][0].get_value_as_float()
            or self._models["lower_bound"][1].get_value_as_float()
            >= self._models["upper_bound"][1].get_value_as_float()
            or self._models["lower_bound"][2].get_value_as_float() > self._models["upper_bound"][2].get_value_as_float()
        ):
            # carb.log_warn("lower bound is >= upper bound")
            return
        if self.wait_bound_update:
            if self.bound_update_case == 0:
                self.lower_bound[0] = self._models["lower_bound"][0].get_value_as_float()
            elif self.bound_update_case == 1:
                self.lower_bound[1] = self._models["lower_bound"][1].get_value_as_float()
            elif self.bound_update_case == 2:
                self.upper_bound[0] = self._models["upper_bound"][0].get_value_as_float()
            elif self.bound_update_case == 3:
                self.upper_bound[1] = self._models["upper_bound"][1].get_value_as_float()
        else:
            self.lower_bound[0] = self._models["lower_bound"][0].get_value_as_float()
            self.lower_bound[1] = self._models["lower_bound"][1].get_value_as_float()
            self.upper_bound[0] = self._models["upper_bound"][0].get_value_as_float()
            self.upper_bound[1] = self._models["upper_bound"][1].get_value_as_float()

        update_location(
            self._om,
            [
                self._models["origin"][0].get_value_as_float(),
                self._models["origin"][1].get_value_as_float(),
                self._models["origin"][2].get_value_as_float(),
            ],
            [self.lower_bound[0], self.lower_bound[1], self._models["lower_bound"][2].get_value_as_float()],
            [self.upper_bound[0], self.upper_bound[1], self._models["upper_bound"][2].get_value_as_float()],
        )

    def on_update_cell_size(self, value):
        self._om.set_cell_size(self._models["cell_size"].get_value_as_float())

    def _draw_instances(self):

        instancePath = "/occupancyMap/occupiedInstances"
        cubePath = "/occupancyMap/occupiedCube"
        pos_list = self._om.get_occupied_positions()
        scale = self._models["cell_size"].get_value_as_float() * 0.5
        color = (0.0, 1.0, 1.0)
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(instancePath):
            stage.RemovePrim(instancePath)
        point_instancer = UsdGeom.PointInstancer(stage.DefinePrim(instancePath, "PointInstancer"))
        positions_attr = point_instancer.CreatePositionsAttr()
        if stage.GetPrimAtPath(cubePath):
            stage.RemovePrim(cubePath)
        occupiedCube = UsdGeom.Cube(stage.DefinePrim(cubePath, "Cube"))
        occupiedCube.AddScaleOp().Set(Gf.Vec3d(1, 1, 1) * scale)
        occupiedCube.CreateDisplayColorPrimvar().Set([color])

        point_instancer.CreatePrototypesRel().SetTargets([occupiedCube.GetPath()])
        proto_indices_attr = point_instancer.CreateProtoIndicesAttr()
        print("total points drawn: ", len(pos_list))
        positions_attr.Set(pos_list)
        proto_indices_attr.Set([0] * len(pos_list))

    def _generate_map(self):
        if (
            self._models["lower_bound"][0].get_value_as_float() >= self._models["upper_bound"][0].get_value_as_float()
            or self._models["lower_bound"][1].get_value_as_float()
            >= self._models["upper_bound"][1].get_value_as_float()
            or self._models["lower_bound"][2].get_value_as_float() > self._models["upper_bound"][2].get_value_as_float()
        ):
            carb.log_warn("lower bound is >= upper bound, cannot calculate map")
            return

        self.on_update_location(0)
        self.on_update_cell_size(0)

        async def generate_task():
            do_stop = False
            if not self._timeline.is_playing():
                self._timeline.play()
                do_stop = True
            await omni.kit.app.get_app().next_update_async()
            self._om.generate()
            await omni.kit.app.get_app().next_update_async()
            if do_stop:
                self._timeline.stop()

        asyncio.ensure_future(generate_task())
        # self.generate_image_btn.visible = True
        # self.draw_voxel_btn.visible = True

    def _fill_image(self):
        dims = self._om.get_dimensions()
        scale = self._models["cell_size"].get_value_as_float()
        # Clockwise rotation
        rotate_image_angle = 0
        current_image_rotation_index = self._models["rotation"].get_item_value_model().as_int
        if current_image_rotation_index == 0:
            top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(self._om, scale)
        elif current_image_rotation_index == 1:  # -90 degrees
            top_right, bottom_right, top_left, bottom_left, image_coords = compute_coordinates(self._om, scale)
            rotate_image_angle = -90
        elif current_image_rotation_index == 2:  # 90 degrees
            bottom_left, top_left, bottom_right, top_right, image_coords = compute_coordinates(self._om, scale)
            rotate_image_angle = 90
        elif current_image_rotation_index == 3:  # 180 degrees
            bottom_right, bottom_left, top_right, top_left, image_coords = compute_coordinates(self._om, scale)
            rotate_image_angle = 180

        # print("World coordinates for image in stage units:")
        # print("Top left: ", top_left)
        # print("Top right: ", top_right)

        # print("Bottom left: ", bottom_left)
        # print("Bottom right: ", bottom_right)

        # print(
        #     f"Coordinates of top left of image (pixel 0,0) as origin, + X down, + Y right:\n{float(image_coords[0][0]), float(image_coords[1][0])}"
        # )

        occupied_col = []
        for item in self._models["occupied_color"].get_item_children():
            component = self._models["occupied_color"].get_item_value_model(item)
            occupied_col.append(int(component.get_value_as_float() * 255))

        freespace_col = []
        for item in self._models["freespace_color"].get_item_children():
            component = self._models["freespace_color"].get_item_value_model(item)
            freespace_col.append(int(component.get_value_as_float() * 255))

        unknown_col = []
        for item in self._models["unknown_color"].get_item_children():
            component = self._models["unknown_color"].get_item_value_model(item)
            unknown_col.append(int(component.get_value_as_float() * 255))

        self._image = generate_image(self._om, occupied_col, unknown_col, freespace_col)

        from PIL import Image

        self._im = Image.frombytes("RGBA", (dims.x, dims.y), bytes(self._image))
        self._im = self._im.rotate(-rotate_image_angle, expand=True)
        self._image = list(self._im.tobytes())

        image_width = self._im.width
        image_height = self._im.height

        size = [0, 0, 0]

        size[0] = image_width * scale
        size[1] = image_height * scale

        self._rgb_byte_provider.set_bytes_data(self._image, [int(size[0] / scale), int(size[1] / scale)])
        self._image_frame.rebuild()

        image_details_text = f"Top Left: {top_left}\t\t Top Right: {top_right}\n Bottom Left: {bottom_left}\t\t Bottom Right: {bottom_right}"
        image_details_text += f"\nCoordinates of top left of image (pixel 0,0) as origin, + X down, + Y right:\n{float(image_coords[0][0]), float(image_coords[1][0])}"
        image_details_text += f"\nImage size in pixels: {int(size[0] / scale)}, {int(size[1] / scale)}"

        scale_to_meters = 1.0 / get_stage_units()

        stage = omni.usd.get_context().get_stage()
        root = stage.GetRootLayer()
        default_image_name = root.GetDisplayName().rsplit(".", 1)[0]
        default_image_name += ".png"

        ros_yaml_file_text = "image: " + default_image_name
        ros_yaml_file_text += f"\nresolution: {float( scale/ scale_to_meters)}"
        ros_yaml_file_text += (
            f"\norigin: [{float(bottom_left[0]/scale_to_meters)}, {float(bottom_left[1]/scale_to_meters)}, 0.0000]"
        )
        ros_yaml_file_text += "\nnegate: 0"
        ros_yaml_file_text += f"\noccupied_thresh: {0.65}"
        ros_yaml_file_text += "\nfree_thresh: 0.196"

        current_data_output_index = self._models["config_type"].get_item_value_model().as_int
        if current_data_output_index == 0:
            self._models["config_data"].set_value(image_details_text)
        elif current_data_output_index == 1:
            self._models["config_data"].set_value(ros_yaml_file_text)

    def save_image(self, file, folder):
        from PIL import Image

        image_width = self._im.width
        image_height = self._im.height
        file = file if file[-4:].lower() == ".png" else "{}.png".format(file)
        im = Image.frombytes("RGBA", (image_width, image_height), bytes(self._image))
        print("Saving occupancy map image to", folder + "/" + file)
        im.save(folder + "/" + file)
        self._filepicker.hide()

    def save_file(self):
        from omni.kit.window.filepicker import FilePickerDialog
        from omni.kit.widget.filebrowser import FileBrowserItem

        def _on_filter_png_files(item: FileBrowserItem) -> bool:
            """Callback to filter the choices of file names in the open or save dialog"""
            if not item or item.is_folder:
                return True
            # Show only files with listed extensions
            return os.path.splitext(item.path)[1] == ".png"

        self._filepicker = None
        self._filepicker = FilePickerDialog(
            "Save .png image",
            allow_multi_selection=False,
            apply_button_label="Save",
            click_apply_handler=self.save_image,
            item_filter_options=[".png Files (*.png, *.PNG)"],
            item_filter_fn=_on_filter_png_files,
        )

    def rebuild_frame(self):
        if self._image is not None:
            with ui.VStack():
                omni.ui.ImageWithProvider(self._rgb_byte_provider)
                ui.Button("Save Image", clicked_fn=self.save_file, height=0)

    def _generate_image(self):
        self._image = None
        # check to make sure image has data first
        dims = self._om.get_dimensions()
        if dims.x == 0 or dims.y == 0:
            carb.log_warn(
                "Occupancy map is empty, press CALCULATE first and make sure there is collision geometry in the mapping bounds"
            )
            return
        self._rgb_byte_provider = omni.ui.ByteImageProvider()
        self._visualize_window = omni.ui.Window("Visualization", width=500, height=600)
        with self._visualize_window.frame:
            with ui.VStack(spacing=5):
                with ui.VStack(height=0, spacing=5):
                    kwargs = {"label": "Occupied Color", "default_val": [0, 0, 0, 1]}
                    self._models["occupied_color"] = color_picker_builder(**kwargs)
                    kwargs = {"label": "Freespace Color", "default_val": [1, 1, 1, 1]}
                    self._models["freespace_color"] = color_picker_builder(**kwargs)
                    kwargs = {"label": "Unknown Color", "default_val": [0.5, 0.5, 0.5, 1]}
                    self._models["unknown_color"] = color_picker_builder(**kwargs)
                    self._models["rotation"] = dropdown_builder(
                        label="Rotate Image",
                        items=["0", "-90", "90", "180"],
                        tooltip="Clockwise rotation of image in degrees",
                    )
                    self._models["config_type"] = dropdown_builder(
                        label="Coordinate Type",
                        items=["Coordinates in Stage Space", "ROS Occupancy Map Parameters File (YAML)"],
                        tooltip="Type of config output generated",
                    )
                    self._models["generate"] = btn_builder(
                        label="Occupancy map", text="Re-Generate Image", on_clicked_fn=self._fill_image
                    )
                    self._models["config_data"] = ui.StringField(height=100, multiline=True).model
                self._image_frame = ui.Frame()
                self._image_frame.set_build_fn(self.rebuild_frame)
        # generate image imadeately when this window appears
        self._fill_image()

    def on_shutdown(self):
        if self._filepicker:
            self._filepicker = None
        remove_menu_items(self._menu_items, "Isaac Utils")
        gc.collect()
