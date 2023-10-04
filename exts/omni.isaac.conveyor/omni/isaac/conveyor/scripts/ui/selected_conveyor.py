from ..conveyor_builder.conveyor_track import Ramp, Type
import omni.ui as ui


class SelectedConveyorWidget:
    def __init__(self, **kwargs):
        self._frame = ui.Frame(**kwargs, width=128, height=128)
        self.name = kwargs.get("name", "")
        self.style = kwargs
        self.input_boxes = []
        self.output_boxes = []
        self.track_list = kwargs.get("track_list", [])
        self.selected_conveyor = kwargs.get("selected_track", self.track_list[0] if self.track_list else None)
        self.on_add_track_callback = kwargs.get("add_track_fn", None)
        self.on_selection_changed_callback = kwargs.get("selection_changed_fn", None)
        self.on_remove_track_callback = kwargs.get("remove_track_fn", None)
        self.on_anchor_changed_callback = kwargs.get("anchor_changed_fn", None)
        self.on_flip_callback = kwargs.get("flip_fn", None)
        self.current_anchor = 0
        self.available_anchors = []
        # self.build_ui()

    def on_flip(self, track):
        if self.on_flip_callback:
            self.on_flip_callback(track)

    def on_add_track(self, track):
        if self.on_add_track_callback:
            self.on_add_track_callback(track)

    def on_selection_changed(self, track):
        if self.on_selection_changed_callback:
            self.on_selection_changed_callback(track)

    def on_remove_track(self, track):
        if self.on_remove_track_callback:
            self.on_remove_track_callback(track)

    def update_list(self, new_list, new_selection=0, direction=1):
        self.track_list = new_list
        self.update_selection(self.track_list[new_selection], direction=direction)

    def update_selection(self, new_selection, available_anchors=[], direction=1):
        if self.selected_conveyor != new_selection or self.available_anchors != available_anchors:
            self.selected_conveyor = new_selection
            # if new_selection:
            #     self.current_anchor = 0 if self.name != "current" else min(1, len(self.selected_conveyor.anchors) - 1)
            # else:
            self.current_anchor = 0
            if available_anchors:
                self.available_anchors = available_anchors
            else:
                if self.selected_conveyor:
                    self.available_anchors = self.selected_conveyor.get_anchors(direction)
                else:
                    self.available_anchors = []
            self.on_selection_changed(new_selection)
        # print("available anchors", available_anchors, self.available_anchors)
        self.build_ui()

    def anchor_changed(self, new_anchor):
        self.current_anchor = new_anchor
        for i in range(len(self.anchor_btns)):
            self.anchor_btns[i].selected = new_anchor == i
        self.on_anchor_changed(self.available_anchors[new_anchor])

    def on_anchor_changed(self, new_anchor):
        if self.on_anchor_changed_callback:
            self.on_anchor_changed_callback(new_anchor)

    def get_current_anchor(self):
        if self.selected_conveyor:
            return self.available_anchors[self.current_anchor]
        return ""

    def get_next_anchor(self, direction):
        if self.selected_conveyor:
            return self.selected_conveyor.get_anchors(direction)[self.current_anchor]

    def build_ui(self):
        with self._frame:
            with ui.ZStack(width=144, height=144, alignment=ui.Alignment.CENTER):
                self.style["alignment"] = ui.Alignment.CENTER
                ui.Rectangle(
                    **self.style, style_type_name_override="ConveyorCard", selected=bool(self.selected_conveyor)
                )
                if self.selected_conveyor:
                    with ui.ZStack():
                        # ui.Rectangle(
                        #     style_type_name_override="ConveyorCardBackground",
                        #     selected=bool(self.selected_conveyor) and self.name == "current",
                        #     width=100,
                        #     height=100,
                        # )
                        with ui.HStack():
                            ui.ImageWithProvider(
                                self.selected_conveyor.get_thumb(),
                                **self.style,
                                width=ui.Pixel(144),
                                height=ui.Pixel(144),
                                style={"margin": 15},
                            )
                        with ui.VStack(height=ui.Pixel(144)):
                            ui.Spacer(height=ui.Pixel(94))
                            with ui.ZStack():
                                with ui.HStack():
                                    ui.Spacer()

                                    if self.name == "current":
                                        ui.Button(
                                            name="delete",
                                            width=50,
                                            height=50,
                                            style={"border_radius": 6},
                                            clicked_fn=lambda: self.on_remove_track(self.selected_conveyor),
                                        )
                                    else:
                                        # ui.Spacer()
                                        ui.Button(
                                            name="add_box",
                                            width=50,
                                            height=50,
                                            style={"border_radius": 6},
                                            clicked_fn=lambda: self.on_add_track(self.selected_conveyor),
                                        )

                                    ui.Spacer()
                                if self.name != "current" and self.selected_conveyor.type in [
                                    Type.Y_MERGE,
                                    Type.T_MERGE,
                                ]:

                                    with ui.VStack():
                                        ui.Spacer(height=ui.Pixel(15))
                                        with ui.HStack():
                                            ui.Spacer()
                                            ui.Button(
                                                name="flip",
                                                width=35,
                                                height=35,
                                                style={"border_radius": 6},
                                                clicked_fn=lambda: self.on_flip(self.selected_conveyor),
                                            )

                    with ui.HStack(height=ui.Pixel(30), width=ui.Fraction(1)):
                        ui.Spacer(height=ui.Pixel(30))
                        if self.track_list and len(self.track_list) > 1:
                            for track in self.track_list:
                                ui.Button(
                                    image_url=str(track.get_thumb()),
                                    width=30,
                                    height=30,
                                    style={"border_radius": 6},
                                    style_type_name_override="CardButton",
                                    selected=track == self.selected_conveyor,
                                    clicked_fn=lambda t=track: self.update_selection(t),
                                )
                        else:
                            ui.Label(self.name, style={"color": 0x44FFFFFF})
                        ui.Spacer(height=ui.Pixel(30))
                    # with ui.HStack(height=ui.Pixel(30)):
                    #     if self.name == "current":
                    #         ui.Button(name="delete", width=30)
                    #     else:
                    # ui.Button(name="rotate_left", width=30)
                    # ui.Button(name="rotate_right", width=30)
                    # ui.Spacer()
                    # ui.Button(name="add_box", width=30)
                    self.anchor_btns = []
                    anchors = [a for a in self.available_anchors]
                    if self.selected_conveyor.ramp != Ramp.FLAT:
                        anchors = [anchors[0]]
                    if len(anchors) > 1:
                        with ui.Placer(
                            offset_x=5 if self.name != "current" else 125,
                            offset_y=30 if self.track_list and len(self.track_list) > 1 else 0,
                            width=1,
                        ):
                            with ui.VStack(
                                height=ui.Pixel(100 if self.track_list and len(self.track_list) > 1 else 130)
                            ):
                                ui.Spacer()
                                for i in range(len(self.available_anchors)):
                                    self.anchor_btns.append(
                                        ui.Button(
                                            name="square",
                                            selected=i == self.current_anchor,
                                            width=15,
                                            height=15,
                                            clicked_fn=lambda a=i: self.anchor_changed(a),
                                        )
                                    )
                                    if i + 1 < len(self.available_anchors):
                                        ui.Spacer(height=15)
                                ui.Spacer()
                    # with ui.Placer(offset_x=5, offset_y=100):
                    #     ui.Button(name="square", selected=False, width=15, height=15)
                else:
                    ui.Label(
                        "Select a track to connect or add a start",
                        word_wrap=True,
                        alignment=ui.Alignment.CENTER,
                        style={"color": 0x88FFFFDD},
                    )
