from math import asin, degrees


from omni.isaac.core.utils.stage import get_next_free_path
from pxr import Gf, Sdf, UsdGeom, UsdShade
from pxr.Usd import EditContext, Stage
from ..conveyor_builder.conveyor_track import Angle, Ramp, Style, Type
from ..conveyor_builder.conveyor_system import ConveyorBuilder
from ..conveyor_builder.conveyor_system import set_pose_from_transform
from ..conveyor_builder.conveyor_system import ConveyorFilter, ConveyorSelector
import omni.ui as ui
import omni
from .selected_conveyor import SelectedConveyorWidget
import json
import asyncio
import copy


class ConveyorBuilderWidget:
    def on_display(self):
        stage = omni.usd.get_context().get_stage()
        self.builder.clear_system(stage)
        with EditContext(stage, stage.GetSessionLayer()):
            omni.kit.commands.execute(
                "CreatePrim", prim_path="/ConveyorBuilder", prim_type="Scope", attributes={}, select_new_prim=False
            )
            prim = stage.GetPrimAtPath("/ConveyorBuilder")
            omni.usd.editor.set_hide_in_stage_window(prim, True)
            self.temp_prim = stage.DefinePrim("/ConveyorBuilder/conveyorBuilder_Temp", "Xform")
            omni.usd.get_context().set_pickable("/ConveyorBuilder/conveyorBuilder_Temp", False)
            # self.temp_mat = stage.DefinePrim(, "Material")
            # with Sdf.ChangeBlock():
            material = omni.kit.commands.execute(
                "CreateMdlMaterialPrim",
                mtl_url=self.mdl_file,
                mtl_name="voltest_02",
                mtl_path=Sdf.Path("/ConveyorBuilder/conveyorBuilder_Temp_mat"),
            )
            shader_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp_mat/Shader")
            shader_prim.CreateAttribute("inputs:absorption", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.8, 0.8))
            shader_prim.CreateAttribute("inputs:scattering", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))
            shader_prim.CreateAttribute("inputs:transmission_color", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.1, 1.0, 0.3)
            )
            shader_prim.CreateAttribute("inputs:distance_scale", Sdf.ValueTypeNames.Float).Set(1.0)
            shader_prim.CreateAttribute("inputs:emissive_scale", Sdf.ValueTypeNames.Float).Set(300.0)
            shader_prim.CreateAttribute("inputs:transmission_color", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.3, 1.0, 0.3)
            )
            UsdShade.MaterialBindingAPI(self.temp_prim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(Sdf.Path("/ConveyorBuilder/conveyorBuilder_Temp_mat"))),
                UsdShade.Tokens.strongerThanDescendants,
            )
        self._stage_event_subscription = self._events.create_subscription_to_pop(
            self._on_stage_event, name="Onshape Usd Exporter stage Watch"
        )

    def __init__(self, config_file, mdl_file, style):
        self._frame = ui.Frame(style=style)
        self.style = style
        self.style_buttons = []
        self.direction_buttons = []
        self.slope_buttons = []
        self.type_buttons = []
        self.temp_prim = None
        self._selection = None
        self.current_track = None
        self.next_track = None
        self.mdl_file = mdl_file
        self.flip_placement = False
        self.part_usd = None
        self.previous_selection = []

        with open(config_file) as f:
            self.config = json.load(f)
        self._conveyor_selector = ConveyorSelector(self.config, thumb_loaded_callback=self.on_thumb_loaded)

        self._filter = ConveyorFilter(
            styles=[Style.BELT], types=[Type.STRAIGHT], angles=[Angle.NONE], ramps=[Ramp.FLAT]
        )
        self.builder = ConveyorBuilder(omni.usd.get_context().get_stage(), self._conveyor_selector)
        self.available_tiles = self._conveyor_selector.list_tracks(self._filter)
        self._selection = omni.usd.get_context().get_selection()
        self._events = omni.usd.get_context().get_stage_event_stream()

        self.button_to_style = {
            "conveyor_roller": [Style.ROLLER],
            "conveyor_belt": [Style.BELT],
            "conveyor_combine": [Style.DUAL],
        }
        self.button_to_type = {
            "type_start": [Type.START],
            "type_straight": [Type.STRAIGHT],
            "type_t": [Type.T_MERGE],
            "type_y": [Type.Y_MERGE, Type.FORK_MERGE],
            "type_end": [Type.END],
        }
        self.button_to_angle = {
            "arrow_180_left": [Angle.FULL],
            "arrow_180_right": [Angle.FULL],
            "arrow_90_left": [Angle.HALF],
            "arrow_90_right": [Angle.HALF],
            "arrow_up": [Angle.NONE],
        }
        self.button_to_ramp = {
            "ramp_2l": [Ramp.THREE, Ramp.FOUR],
            "ramp_2r": [Ramp.THREE, Ramp.FOUR],
            "ramp_1l": [Ramp.ONE, Ramp.TWO],
            "ramp_1r": [Ramp.ONE, Ramp.TWO],
            "ramp_flat": [Ramp.FLAT],
        }
        self.build_ui()
        self._on_kit_selection_changed()

    def shutdown(self):
        del self._stage_event_subscription
        self._stage_event_subscription = None
        stage = omni.usd.get_context().get_stage()
        self.builder.clear_system(stage)
        if stage:
            for pth in ["/ConveyorBuilder"]:
                temp_prim = stage.GetPrimAtPath(pth)
                if temp_prim:
                    with EditContext(stage, stage.GetSessionLayer()):
                        stage.RemovePrim(temp_prim.GetPath())
                    stage.RemovePrim(temp_prim.GetPath())

    def _on_stage_event(self, event):
        """Called with omni.usd.context when stage event"""

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._on_kit_selection_changed()
        if event.type == int(omni.usd.StageEventType.CLOSED):
            self.shutdown()

    def get_selection(self):
        stage = omni.usd.get_context().get_stage()
        if self._selection:
            for i, p in enumerate(self._selection.get_selected_prim_paths()):
                refs = []
                if "/ConveyorBuilder" in p:
                    self._selection.set_selected_prim_paths(self.previous_selection, False)
                    return None, "/"
                sel = stage.GetPrimAtPath(p)
                while sel != stage.GetPrimAtPath("/"):
                    if sel.GetReferences():
                        refs += omni.usd.get_composed_references_from_prim(sel, False)
                    if sel.GetPayloads():
                        refs += omni.usd.get_composed_payloads_from_prim(sel, False)
                    for a in refs:
                        if self._conveyor_selector.asset_path in a[0].assetPath:
                            asset = a[0].assetPath.replace(self._conveyor_selector.asset_path + "/", "")
                            self._selection.set_selected_prim_paths([str(sel.GetPath())], False)
                            self.previous_selection = [str(sel.GetPath())]
                            return self._conveyor_selector.tracks[asset], p

                    sel = sel.GetParent()
        return None, "/"

    def _on_kit_selection_changed(self):
        """The selection in kit is changed"""
        selections = self.get_selection()
        if "/ConveyorBuilder" not in selections[1]:
            self.current_track.update_selection(selections[0], self.builder.get_available_connections(selections[1]))
            in_level = 0
            out_level = 0
            if self.current_track.selected_conveyor:
                out_level = self.current_track.selected_conveyor.end_level(direction=self.current_direction())
            for a in range(len(self.available_tiles)):
                if (
                    self._conveyor_selector.tracks[self.available_tiles[a]].start_level(
                        direction=self.get_ramp_and_curve_direction()[0]
                    )
                    == out_level
                ):
                    in_level = a
                    break
            scale = self.get_ramp_and_curve_direction()
            self.next_track.update_list(
                [self._conveyor_selector.tracks[a] for a in self.available_tiles], in_level, scale[0]
            )

            self.on_next_anchor_changed("")

    def get_ramp_and_curve_direction(self):
        ramp = -1 if bool([i for i in range(2) if self.slope_buttons[i].selected]) else 1
        curve = -1 if bool([i for i in range(2) if self.direction_buttons[i].selected]) else 1
        if self.next_track.selected_conveyor:
            if self.next_track.selected_conveyor.angle != Angle.NONE and self.next_track.get_next_anchor(ramp):
                curve = -curve
        if self.flip_placement:
            curve = -curve
        return Gf.Vec3d(ramp, curve, 1)

    def current_direction(self):
        stage = omni.usd.get_context().get_stage()
        sel = self.get_selection()
        if sel[0]:
            return self.builder.get_direction(sel[1])
        return 1

    def get_next_base_pose(self):
        scale = self.get_ramp_and_curve_direction()
        return self.builder.get_next_pose(
            self.next_track.selected_conveyor,
            self.next_track.get_next_anchor(scale[0]),
            1,
            scale[1],
            self.get_selection()[1],
            self.current_track.get_current_anchor(),
        )

    def on_track_selection_changed(self, track):
        stage = omni.usd.get_context().get_stage()
        temp_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp")
        if temp_prim:
            with EditContext(stage, stage.GetSessionLayer()):
                temp_prim.GetReferences().ClearReferences()
                temp_prim.GetReferences().AddReference(track.base_usd)
                self.part_usd = track.base_usd
        self.on_next_anchor_changed("")

    def on_next_anchor_changed(self, anchor):
        stage = omni.usd.get_context().get_stage()
        temp_prim = stage.GetPrimAtPath("/ConveyorBuilder/conveyorBuilder_Temp")
        with EditContext(stage, stage.GetSessionLayer()):
            scale = self.get_ramp_and_curve_direction()
            scale[0] = 1
            scale_unit = 1
            if self.part_usd:
                part_stage = Stage.Open(self.part_usd)
                a = UsdGeom.GetStageMetersPerUnit(stage)
                b = UsdGeom.GetStageMetersPerUnit(part_stage)
                scale_unit = b / a
            set_pose_from_transform(temp_prim, self.get_next_base_pose(), scale_unit * scale)

    def on_thumb_loaded(self, track):
        pass
        if self.current_track:
            if self.current_track.selected_conveyor == track or self.current_track.selected_conveyor is None:
                self.current_track.build_ui()
        if self.next_track:
            if self.next_track.selected_conveyor == track:
                self.next_track.build_ui()

    def on_flip(self, current):
        self.flip_placement = not self.flip_placement
        self.on_next_anchor_changed(None)

    def remove_track(self, track):
        selection = self.get_selection()
        parent = None
        stage = omni.usd.get_context().get_stage()
        if selection[0]:
            parent = self.builder.remove_track(selection[1])
            stage.RemovePrim(selection[1])
        self._selection.set_selected_prim_paths([parent], False)

    def on_add_track(self, track):

        selection = self.get_selection()
        parent_anchor = ""
        if selection[0]:
            parent_anchor = self.current_track.get_current_anchor()
        scale = self.get_ramp_and_curve_direction()
        new_track = self.builder.add_track(
            self.next_track.selected_conveyor,
            self.next_track.get_next_anchor(scale[0]),
            1,
            scale[1],
            selection[1],
            parent_anchor,
        )
        self._selection.set_selected_prim_paths([new_track], False)

    def build_ui(self):
        self.on_display()
        with self._frame:
            with ui.VStack(**self.style):
                with ui.HStack(height=ui.Pixel(60), alignment=ui.Alignment.CENTER):  # Conveyor Type
                    ui.Spacer(width=60)

                    self.style_buttons.append(
                        ui.Button(
                            name="conveyor_roller", height=60, width=60, tooltip="Roller-style conveyor", selected=False
                        )
                    )
                    self.style_buttons.append(
                        ui.Button(
                            name="conveyor_belt", height=60, width=60, tooltip="belt-style conveyor", selected=True
                        )
                    )
                    self.style_buttons.append(
                        ui.Button(name="conveyor_combine", height=60, width=60, tooltip="dual conveyor")
                    )

                for b in self.style_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_style_selected(a))
                with ui.HStack(height=ui.Pixel(60), alignment=ui.Alignment.CENTER):  # Conveyor Type

                    self.type_buttons.append(
                        ui.Button(name="type_start", height=60, width=60, tooltip="conveyor start")
                    )
                    self.type_buttons.append(ui.Button(name="type_t", height=60, width=60, tooltip="T-style split"))
                    self.type_buttons.append(
                        ui.Button(
                            name="type_straight", height=60, width=60, tooltip="single-line conveyor", selected=True
                        )
                    )
                    self.type_buttons.append(ui.Button(name="type_y", height=60, width=60, tooltip="y-style split"))
                    self.type_buttons.append(
                        ui.Button(name="type_end", height=60, width=60, tooltip="conveyor endpoint")
                    )

                for b in self.type_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_type_selected(a))
                with ui.HStack(height=ui.Pixel(60)):  # Conveyor Direction
                    self.direction_buttons.append(
                        ui.Button(name="arrow_180_left", height=60, width=60, tooltip="180 degrees turn left")
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_90_left", height=60, width=60, tooltip="90 degrees turn left")
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_up", height=60, width=60, tooltip="forward", selected=True)
                    )
                    self.direction_buttons.append(
                        ui.Button(name="arrow_90_right", height=60, width=60, tooltip="90 degrees turn right")
                    )
                    self.direction_buttons.append(
                        ui.Button(
                            name="arrow_180_right", height=60, width=60, tooltip="180 degrees turn right", enabled=False
                        )
                    )
                for b in self.direction_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_direction_selected(a))
                with ui.HStack(height=ui.Pixel(60)):  # Conveyor ramp
                    self.slope_buttons.append(
                        ui.Button(name="ramp_2l", height=60, width=60, tooltip="2 level down ramp")
                    )
                    self.slope_buttons.append(
                        ui.Button(name="ramp_1l", height=60, width=60, tooltip="1 level down ramp")
                    )
                    self.slope_buttons.append(
                        ui.Button(name="ramp_flat", height=60, width=60, tooltip="flat piece", selected=True)
                    )
                    self.slope_buttons.append(ui.Button(name="ramp_1r", height=60, width=60, tooltip="1 level up ramp"))
                    self.slope_buttons.append(ui.Button(name="ramp_2r", height=60, width=60, tooltip="2 level up ramp"))
                for b in self.slope_buttons:
                    b.set_clicked_fn(lambda a=b: self.on_slope_selected(a))
                ui.Spacer(height=ui.Pixel(3))
                with ui.HStack(height=ui.Pixel(150)):
                    ui.Spacer(width=ui.Pixel(3))
                    self.current_track = SelectedConveyorWidget(
                        **self.style,
                        name="current",
                        anchor_changed_fn=self.on_next_anchor_changed,
                        flip_fn=lambda a: self.on_flip(True),
                        remove_track_fn=self.remove_track,
                    )
                    ui.Spacer(width=ui.Pixel(6))
                    self.next_track = SelectedConveyorWidget(
                        **self.style,
                        thumbnail="",
                        selection_changed_fn=self.on_track_selection_changed,
                        anchor_changed_fn=self.on_next_anchor_changed,
                        flip_fn=lambda a: self.on_flip(False),
                        add_track_fn=self.on_add_track,
                    )
                self.update_filter()

    def update_filter(self):
        self.available_tiles = self._conveyor_selector.list_tracks(self._filter)
        filter = copy.copy(self._filter)
        filter.type = []
        type_list = self._conveyor_selector.list_tracks(filter)
        types = [
            bool([a for a in type_list if self._conveyor_selector.tracks[a].type == b])
            for b in [Type.START, Type.T_MERGE, Type.STRAIGHT, Type.Y_MERGE, Type.END]
        ]
        for a in range(len(types)):
            self.type_buttons[a].enabled = types[a]
        filter.type = self._filter.type
        if filter.type[0] not in [Type.Y_MERGE, Type.T_MERGE]:
            self.flip_placement = False

        filter.angle = []
        curvature_list = self._conveyor_selector.list_tracks(filter)
        curvature_1 = bool([a for a in curvature_list if self._conveyor_selector.tracks[a].angle == Angle.HALF])
        curvature_2 = bool([a for a in curvature_list if self._conveyor_selector.tracks[a].angle == Angle.FULL])
        self.direction_buttons[0].enabled = curvature_2
        self.direction_buttons[1].enabled = curvature_1
        self.direction_buttons[3].enabled = curvature_1
        self.direction_buttons[4].enabled = curvature_2
        if not bool([a for a in self.direction_buttons if a.selected]):
            self.direction_buttons[2].selected = True
            return self.on_direction_selected(self.direction_buttons[2])
        filter.angle = self._filter.angle

        filter.ramp = []
        slope_list = self._conveyor_selector.list_tracks(filter)
        for a in self.direction_buttons:
            if a.selected and not a.enabled:
                a.selected = False
        slope_1 = bool([a for a in slope_list if self._conveyor_selector.tracks[a].ramp == Ramp.ONE])
        slope_2 = bool([a for a in slope_list if self._conveyor_selector.tracks[a].ramp == Ramp.TWO])
        self.slope_buttons[0].enabled = slope_2
        self.slope_buttons[1].enabled = slope_1
        self.slope_buttons[3].enabled = slope_1
        self.slope_buttons[4].enabled = slope_2
        for a in self.slope_buttons:
            if a.selected and not a.enabled:
                a.selected = False

        if not bool([a for a in self.slope_buttons if a.selected]):
            self.slope_buttons[2].selected = True
            return self.on_slope_selected(self.slope_buttons[2])

        in_level = 0
        out_level = 0
        if self.current_track.selected_conveyor:
            out_level = self.current_track.selected_conveyor.end_level(direction=self.current_direction())
        for a in range(len(self.available_tiles)):
            if (
                self._conveyor_selector.tracks[self.available_tiles[a]].start_level(
                    direction=self.get_ramp_and_curve_direction()[0]
                )
                == out_level
            ):
                in_level = a
                break
        scale = self.get_ramp_and_curve_direction()
        self.next_track.update_list(
            [self._conveyor_selector.tracks[a] for a in self.available_tiles], in_level, scale[0]
        )

    def on_style_selected(self, selected):
        for a in self.style_buttons:
            a.selected = False
        selected.selected = True
        self._filter.style = self.button_to_style[selected.name]
        self.update_filter()

    def on_direction_selected(self, selected):
        before = self.get_ramp_and_curve_direction()
        for a in self.direction_buttons:
            a.selected = False
        selected.selected = True
        self._filter.angle = self.button_to_angle[selected.name]
        self.update_filter()
        after = self.get_ramp_and_curve_direction()
        if before[1] != after[1]:
            self.on_track_selection_changed(self.next_track.selected_conveyor)

    def on_slope_selected(self, selected):
        before = self.get_ramp_and_curve_direction()
        for a in self.slope_buttons:
            a.selected = False
        selected.selected = True
        self._filter.ramp = self.button_to_ramp[selected.name]
        self.update_filter()
        after = self.get_ramp_and_curve_direction()
        if before[0] != after[0]:
            self.on_track_selection_changed(self.next_track.selected_conveyor)

    def on_type_selected(self, selected):
        for a in self.type_buttons:
            a.selected = False
        selected.selected = True
        self._filter.type = self.button_to_type[selected.name]
        self.update_filter()
