# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.dynamic_control import _dynamic_control as dc
import gc
import collections
import carb
import weakref
from pxr import UsdPhysics, UsdGeom

EXTENSION_NAME = "Inspect Physics"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._usd_context = omni.usd.get_context()
        self._dc = dc.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        if self._usd_context is not None:
            self._selection = self._usd_context.get_selection()
            self._events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = self._events.create_subscription_to_pop(
                self._on_stage_event, name="physics inspector stage event"
            )
        self._window = omni.ui.Window(EXTENSION_NAME, width=600, height=400, visible=False)
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self._physx = omni.physx.acquire_physx_interface()

        self._data = {}
        self._plots = {}
        self._labels = {}
        self._frame = {}
        self._selected_prim = None
        self._selected_handle = dc.INVALID_HANDLE
        self._app_event_sub = None
        self._stage_unit = None

        for axis in ["x", "y", "z"]:
            self._data[f"lin_vel_{axis}"] = collections.deque([0.0] * 360, maxlen=360)
            self._data[f"ang_vel_{axis}"] = collections.deque([0.0] * 360, maxlen=360)
            self._data[f"lin_acc_{axis}"] = collections.deque([0.0] * 360, maxlen=360)
            self._data[f"prev_lin_acc_{axis}"] = collections.deque([0.0] * 50, maxlen=50)
            self._data[f"prev_lin_vel_{axis}"] = collections.deque([0.0] * 1, maxlen=1)

        with self._window.frame:
            with ui.VStack(height=0):
                with ui.VStack(height=0):
                    with ui.HStack():
                        ui.Label("Velocity Coordinate Frame: ", width=0, alignment=ui.Alignment.CENTER)
                        self.velocity_combo = ui.ComboBox(0, "Global", "Local")
                    # with ui.HStack():
                    #     ui.Label("Units: ", width=0, alignment=ui.Alignment.CENTER)
                    #     ui.ComboBox(0, "cm", "m")
                with ui.HStack():
                    with ui.VStack(height=0):
                        self._labels["mass"] = ui.Label("Mass: 0.0 kg", width=0, alignment=ui.Alignment.CENTER)
                        self._labels["center of mass"] = ui.Label(
                            "Center of Mass: [0.0, 0.0, 0.0] m", width=0, alignment=ui.Alignment.CENTER
                        )
                        self._labels["moment"] = ui.Label(
                            "Moment: [0.0, 0.0, 0.0] kg*cm^2", width=0, alignment=ui.Alignment.CENTER
                        )
                with ui.HStack():
                    with ui.VStack(height=0):
                        self._labels["position"] = ui.Label("Position", width=0, alignment=ui.Alignment.CENTER)
                        self._labels["rotation"] = ui.Label("Rotation", width=0, alignment=ui.Alignment.CENTER)
                        # ui.MultiFloatField(0.0, 0.0, 0.0, h_spacing=5)
                        ui.Spacer(height=10)
                        self._add_plot(label="lin_vel", title="Linear Velocity: [0.0, 0.0, 0.0] m/s")
                        ui.Spacer(height=10)
                        self._add_plot(label="ang_vel", title="Angular Velocity: [0.0, 0.0, 0.0] rad/s")
                        ui.Spacer(height=10)
                        self._add_plot(label="lin_acc", title="Linear Acceleration: [0.0, 0.0, 0.0] m/s^2")
                        ui.Spacer(height=10)
        selection = self._selection.get_selected_prim_paths()
        if len(selection) == 0:
            self._selected_prim = None
            self._selected_handle = dc.INVALID_HANDLE
        else:
            self._selected_prim = self._usd_context.get_stage().GetPrimAtPath(selection[0])
            self._selected_handle = dc.INVALID_HANDLE
        if self._selected_prim is not None:
            if not self._app_event_sub:
                self._app_event_sub = (
                    omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_app_step)
                )
        else:
            self._app_event_sub = None

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._selected_prim = None
            self._selected_handle = dc.INVALID_HANDLE
            return
        if self._usd_context.get_stage() is None:
            return
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            selection = self._selection.get_selected_prim_paths()
            if len(selection) == 0:
                self._selected_prim = None
                self._selected_handle = dc.INVALID_HANDLE
                return
            else:
                self._selected_prim = self._usd_context.get_stage().GetPrimAtPath(selection[0])
                self._selected_handle = dc.INVALID_HANDLE
                stage = omni.usd.get_context().get_stage()
                self._stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
            if self._selected_prim is not None:
                if not self._app_event_sub:
                    self._app_event_sub = (
                        omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_app_step)
                    )
            else:
                self._app_event_sub = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _add_plot(self, label, title):
        colors = {"x": 0xFF0000FF, "y": 0xFF00FF00, "z": 0xFFFF0000}
        with ui.VStack(height=0):
            self._labels[label] = ui.Label(title, width=0, alignment=ui.Alignment.CENTER)
            ui.Spacer(height=5)
            self._labels[f"max_{label}"] = ui.Label("0.0", width=0, alignment=ui.Alignment.LEFT_TOP)
            with ui.ZStack():
                ui.Rectangle(style={"background_color": 0xFF555555, "border_color": 0xFF000000, "border_width": 1})
                self._frame[label] = ui.Frame(
                    width=ui.Percent(100), height=100, opaque_for_mouse_events=True, style={"color": 0xFFFFFFFF}
                )

                with self._frame[label]:
                    with ui.ZStack():
                        for axis in ["x", "y", "z"]:
                            self._plots[f"{label}_{axis}"] = ui.Plot(
                                ui.Type.LINE,
                                -1,
                                1,
                                *self._data[f"{label}_{axis}"],
                                width=ui.Percent(100),
                                height=100,
                                style={"color": colors[axis], "background_color": 0x00000000},
                            )
            self._labels[f"min_{label}"] = ui.Label("0.0", width=0, alignment=ui.Alignment.LEFT_BOTTOM)

    def _set_plot(self, label, data, units, title):
        self._labels[label].text = f"{title}: {round(data.x,2)},{round(data.y,2)},{round(data.z,2)} {units}"
        self._data[f"{label}_x"].append(data.x)
        self._data[f"{label}_y"].append(data.y)
        self._data[f"{label}_z"].append(data.z)
        min_scale = min(min(self._data[f"{label}_x"]), min(self._data[f"{label}_y"]), min(self._data[f"{label}_z"]))
        max_scale = max(max(self._data[f"{label}_x"]), max(self._data[f"{label}_y"]), max(self._data[f"{label}_z"]))
        self._labels[f"min_{label}"].text = f"{round(min_scale,2)} {units}"
        self._labels[f"max_{label}"].text = f"{round(max_scale,2)} {units}"
        for axis in ["x", "y", "z"]:
            self._plots[f"{label}_{axis}"].scale_min = min_scale
            self._plots[f"{label}_{axis}"].scale_max = max_scale
            self._plots[f"{label}_{axis}"].set_data(*self._data[f"{label}_{axis}"])

    def _on_app_step(self, e: carb.events.IEvent):
        step = e.payload["dt"]
        if len(self._selection.get_selected_prim_paths()) == 0:
            self._app_event_sub = None
            return

        if self._dc.is_simulating() and self._window.visible:
            if self._selected_prim and self._selected_handle == dc.INVALID_HANDLE:
                is_rigid_body = self._selected_prim.HasAPI(UsdPhysics.RigidBodyAPI)
                if is_rigid_body:
                    self._selected_handle = self._dc.get_rigid_body(str(self._selected_prim.GetPath()))
                    for value in self._data.values():
                        for i in range(len(value)):
                            value.append(0.0)
            if self._selected_handle == dc.INVALID_HANDLE:
                self._app_event_sub = None
                return
            rigid_body_props = self._dc.get_rigid_body_properties(self._selected_handle)

            self._labels["mass"].text = f"Mass: {round(rigid_body_props.mass,3)} kg"
            self._labels[
                "center of mass"
            ].text = f"Local Center of Mass: [{round(rigid_body_props.cMassLocalPose.x*self._stage_unit,5)}, {round(rigid_body_props.cMassLocalPose.y*self._stage_unit,5)}, {round(rigid_body_props.cMassLocalPose.z*self._stage_unit,5)}] m"
            self._labels[
                "moment"
            ].text = f"Moment: [{round(rigid_body_props.moment.x*(self._stage_unit**2),5)}, {round(rigid_body_props.moment.y*(self._stage_unit**2),5)}, {round(rigid_body_props.moment.z*(self._stage_unit**2),5)}] kg*m^2"
            pose = self._dc.get_rigid_body_pose(self._selected_handle)
            self._labels[
                "position"
            ].text = f"Position: [xyz] [{round(pose.p.x*self._stage_unit, 4), round(pose.p.y*self._stage_unit, 4), round(pose.p.z*self._stage_unit, 4)}] m"
            self._labels[
                "rotation"
            ].text = (
                f"Rotation: [wxyz] [{round(pose.r.w, 2), round(pose.r.x, 2), round(pose.r.y, 2), round(pose.r.z, 2)}]"
            )
            current_velocity_index = self.velocity_combo.model.get_item_value_model().as_int

            linear_velocity = self._dc.get_rigid_body_linear_velocity(self._selected_handle)
            if current_velocity_index == 1:
                linear_velocity = self._dc.get_rigid_body_local_linear_velocity(self._selected_handle)
            linear_velocity = carb.Float3([(x * self._stage_unit) for x in linear_velocity])

            angular_velocity = self._dc.get_rigid_body_angular_velocity(self._selected_handle)
            acc_x = 0.0
            acc_y = 0.0
            acc_z = 0.0
            num_vel = len(self._data["prev_lin_vel_x"])
            for i in range(num_vel):
                acc_x = acc_x + (self._data["prev_lin_vel_x"][i] - linear_velocity.x) / (step * (num_vel - i))
                acc_y = acc_y + (self._data["prev_lin_vel_y"][i] - linear_velocity.y) / (step * (num_vel - i))
                acc_z = acc_z + (self._data["prev_lin_vel_z"][i] - linear_velocity.z) / (step * (num_vel - i))
            acc_x = acc_x / num_vel
            acc_y = acc_y / num_vel
            acc_z = acc_z / num_vel

            self._data["prev_lin_acc_x"].append(acc_x)
            self._data["prev_lin_acc_y"].append(acc_y)
            self._data["prev_lin_acc_z"].append(acc_z)

            self._set_plot("lin_vel", linear_velocity, "m/s", "Linear Velocity")
            self._set_plot("ang_vel", angular_velocity, "rad/s", "Angular Velocity")
            self._set_plot(
                "lin_acc",
                carb.Float3(
                    sum(self._data["prev_lin_acc_x"]) / len(self._data["prev_lin_acc_x"]),
                    sum(self._data["prev_lin_acc_y"]) / len(self._data["prev_lin_acc_y"]),
                    sum(self._data["prev_lin_acc_z"]) / len(self._data["prev_lin_acc_z"]),
                ),
                "m/s^2",
                "Linear Acceleration",
            )
            self._data["prev_lin_vel_x"].append(linear_velocity.x)
            self._data["prev_lin_vel_y"].append(linear_velocity.y)
            self._data["prev_lin_vel_z"].append(linear_velocity.z)
            # self._frame["linear"].set_tooltip(
            #     f"Linear Velocity: {round(linear_velocity.x,2)},{round(linear_velocity.y,2)},{round(linear_velocity.z,2)}"
            # )
            # self._frame["angular"].set_tooltip(
            #     f"Angular Velocity: {round(angular_velocity.x,2)},{round(angular_velocity.y,2)},{round(angular_velocity.z,2)}"
            # )

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._app_event_sub = None
        self._window = None
        gc.collect()
