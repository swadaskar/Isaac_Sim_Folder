# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from re import I
import omni
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.viewports import set_camera_view
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from pxr import UsdGeom, UsdLux, Sdf, Gf, UsdPhysics
import asyncio
import weakref
import numpy as np
import time

from omni.isaac.ui.ui_utils import (
    setup_ui_headers,
    get_style,
    btn_builder,
    combo_cb_scrolling_frame_builder,
    str_builder,
    state_btn_builder,
)


EXTENSION_NAME = "Generic Info"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        self._ext_id = ext_id

        # The extension acquires the Generic Sensor interface at startup.  It will be released during extension shutdown.  We
        # create a Generic prim using our schema, and then we interact with / query that prim using the python API found
        # in generic/bindings
        self._sensor = _range_sensor.acquire_generic_sensor_interface()

        self._timeline = omni.timeline.get_timeline_interface()

        self._menu_items = [
            MenuItemDescription(
                name="Sensors",
                sub_menu=[
                    make_menu_item_description(
                        ext_id, "Generic Range Sensor", lambda a=weakref.proxy(self): a._menu_callback()
                    )
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

        self._pattern_set = False
        self._generic = False
        self._plot = False
        self._sampling_rate = 2.4e5  # number of samples per second
        self._plot_duration = 4  # seconds to collect sample before plotting
        self._record_start = time.perf_counter()
        self._streaming = True

        self._build_ui()

    def _build_ui(self):
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=600, height=0, visible=False, dockPreference=omni.ui.DockPreference.LEFT_BOTTOM
        )
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                title = "Read a Generic Range Sensor Data Stream"
                doc_link = (
                    "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_range_sensor.html"
                )

                overview = "This sample demonstrates the Generic range sensor python API for Isaac Sim. It shows how to create an Generic Range Sensor, set its properties, and read data streaming from it. "
                overview += "First press the 'Load Sensor' button and then press PLAY to simulate."
                overview += "\n\nPress the 'Open in IDE' button to view the source code."
                overview += "\nNote: The buttons above only work with an Ultrasonic sensor made by the 'Load Sensor' button; not existing ones in the stage."

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
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        dict = {
                            "label": "Load Sensor",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads a Range Sensor and sets its properties",
                            "on_clicked_fn": self._on_spawn_generic_button,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Load Scene",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads a obstacles for the Range Sensor to sense",
                            "on_clicked_fn": self._on_spawn_obstacles_button,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Set Sensor Pattern",
                            "type": "button",
                            "text": "Set",
                            "tooltip": "Sets a Custom Sensor pattern",
                            "on_clicked_fn": self._set_sensor_pattern,
                        }
                        btn_builder(**dict)

                self._output_frame = ui.CollapsableFrame(
                    title="Save Sensor Pattern Images",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with self._output_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        dict = {
                            "label": "Output Directory",
                            "type": "stringfield",
                            "default_val": "/home/",
                            "tooltip": "Save the Scanning Pattern Image on the Wall",
                            "use_folder_picker": True,
                        }
                        self._filepath = str_builder(**dict)
                        btn_builder("", "button", "Save Pattern Image", "", self._on_save_pattern)

    def on_shutdown(self):
        # Perform cleanup once the sample closes
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
        self._generic = False

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    async def _spawn_generic_function(self, task):
        # Wait for stage clear to complete before creating Generic
        done, pending = await asyncio.wait({task})
        if task in done:
            stage = omni.usd.get_context().get_stage()

            # Set up axis to z
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)

            # Create the PhysicsScene.  The generic is going to execute line trace calls in PhysX, and return a value based
            # on how far it travels before colliding with an object that is using the PhysX collision API.  Because of this,
            # to use the sensor extension, you MUST have a physics scene defined
            UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

            # create the Generic Sensor.  Before we can set any attributes on our sensor, we must first create the prim using our
            # Generic schema, and then populate it with the parameters we will be manipulating.  If you try to manipulate
            # a parameter before creating it, you will get a runtime error
            self._genericPath = "/World/GenericSensor"
            self._generic = RangeSensorSchema.Generic.Define(stage, Sdf.Path(self._genericPath))

            # Streaming data bool: True if constantly streaming lidar points in (e.g. non-repeatable patterns)
            # False if only scan in a batch once and repeat it
            self._generic.CreateStreamingAttr().Set(self._streaming)

            # Min and max range for the sensor.  This defines the starting and stopping locations for the linetrace
            self._generic.CreateMinRangeAttr().Set(0.4)
            self._generic.CreateMaxRangeAttr().Set(100.0)

            # sampling rate for the custom data
            self._generic.CreateSamplingRateAttr().Set(self._sampling_rate)

            # These attributes affect drawing the sensor in the viewport.
            # Draw Points = True will draw the actual rays in the viewport.
            self._generic.CreateDrawPointsAttr().Set(False)
            self._generic.CreateDrawLinesAttr().Set(False)

            # We set the attributes we created.  We could have just set the attributes at creation, but this was
            # more illustrative.  It's important to remember that attributes do not exist until you create them; even
            # if they are defined in the schema.
            self._generic.GetDrawLinesAttr().Set(True)
            # self._generic.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 25.0))

            # we want to make sure we can see the sensor we made, so we set the camera position and look target
            set_camera_view(eye=[-5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

            #
            # self._editor_event_subscription = self._editor.subscribe_to_update_events(self._on_editor_step)
            self._editor_event_subscription = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_editor_step)
            )

    def _on_spawn_generic_button(self):
        # wait for new stage before creating sensor
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._spawn_generic_function(task))

    def _on_spawn_obstacles_button(self):
        stage = omni.usd.get_context().get_stage()
        self.CubePath = "/World/Wall"
        offset = Gf.Vec3f(2.00, 0.0, 0.0)
        size = 1

        # Define a light so we can see the obstacle better
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)

        # To create a cube, we first define our geometry at our chosen path.  Then, becuase
        # we will need the primitive later, we query the prim from the stage. If the prim already exists, skip creation
        if stage.GetPrimAtPath(self.CubePath):
            return
        self.cubeGeom = UsdGeom.Cube.Define(stage, self.CubePath)
        self.cubePrim = stage.GetPrimAtPath(self.CubePath)

        # Remember!  Attributes do not exist until they are created.  Here we set the value to the non defualt at
        # creation.  Note that moving the cube to a different location involves adding a translation operation to
        # our primitive.
        self.cubeGeom.CreateSizeAttr(size)
        self.cubeGeom.AddTranslateOp().Set(offset)
        UsdGeom.XformCommonAPI(self.cubePrim).SetScale((1.00, 5.00, 4.00))

        # In order for our cube to interact with the LIDAR, it needs to be able to colide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(self.cubePrim)

    def _set_sensor_pattern(self):

        if self._streaming:
            self.sensor_pattern, self.origin_offsets = self._test_streaming_data()
        else:
            self.sensor_pattern, self.origin_offsets = self._test_repeating_data()

        self._pattern_set = True

    def _test_streaming_data(self):
        """ 
            custom generated data for testing streaming data mode
            data profile: zigzag left to right, slowly going up and down
        """
        # send data in batch that are at least large enough to run a few rendering frames without running out of data.
        # if batch_size > (sampling rate/rendering rate), the sensor will process all of the batch and ask for the next batch right before it runs out.
        # if batch_size < (sampling rate/rendering_rate), the sensor will scan only the provided rays in a given frame, which means it will be scanning slower than intended
        batch_size = int(1e6)  # size of each batch of data being processed
        half_batch = int(batch_size / 2)
        # example scanning pattern is a zigzag
        # each ray specified by an azimuth (horizontal angle measured from x-axis) and a zenith angle (vertical angle measured from z-axis)
        frequency = 10
        N_pts = int(batch_size / frequency / 2)
        # azimuth angle zigzag between the limits (frequency) times every batch
        azimuth = np.tile(
            np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency
        )
        # zenith angle goes up and down once every batch
        zenith = np.append(
            np.linspace(-np.pi / 4, np.pi / 4, half_batch), np.linspace(np.pi / 4, -np.pi / 4, half_batch)
        )

        # custom pattern must be sent as an arrya of [azimuth, zenith] angles.
        sensor_pattern = np.stack((azimuth, zenith))

        # # # import data from file
        # sensor_pattern = np.loadtxt("filename.csv", delimiter=",")
        # batch_size = np.shape(sensor_pattern)[0]
        # sensor_pattern = np.deg2rad(sensor_pattern).T.copy()        ##  MUST USE .copy()

        # individual rays can have an offset at the origin
        # adding random offsets to the origin for the example pattern
        origin_offsets = 0.05 * np.random.random((batch_size, 3))
        # self.origin_offsets = np.zeros((batch_size,3))                  # no offsets

        return sensor_pattern, origin_offsets

    def _test_repeating_data(self):
        """
            custom data to test repeating (non-streaming) mode
            data profile: zigzag left and right, half of it scanning high in zenith, the other half scanning low
            expected behavior: switch between the two sides scanning with no additional data being sent
        """
        batch_size = int(1e6)  # size of each batch of data being processed
        half_batch = int(batch_size / 2)
        frequency = 10
        N_pts = int(batch_size / frequency / 2)
        azimuth = np.tile(
            np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency
        )
        zenith = np.append(-0.5 * np.ones(half_batch), 0.5 * np.ones(half_batch))
        sensor_pattern = np.stack((azimuth, zenith))

        origin_offsets = 0.05 * np.random.random((batch_size, 3))

        return sensor_pattern, origin_offsets

    def _on_editor_step(self, step):
        if not self._timeline.is_playing():
            return

        if self._timeline.is_playing():
            if self._generic:
                if self._pattern_set:
                    if self._sensor.send_next_batch(
                        self._genericPath
                    ):  # send_next_batch will turn True if the sensor is running out data and needs more
                        print("sending more data")
                        self._sensor.set_next_batch_rays(
                            self._genericPath, self.sensor_pattern
                        )  # set the next batch data using set_next_batch_rays()
                        self._sensor.set_next_batch_offsets(
                            self._genericPath, self.origin_offsets
                        )  # (Optional) add indiviaul ray offsets if there are any

                    # one way to visually examine the scanning pattern is to plot the pattern that's hit the wall
                    if self._plot:
                        if (time.perf_counter() - self._record_start) < self._plot_duration:
                            self._point_cloud_data = np.append(
                                self._point_cloud_data, self._sensor.get_point_cloud_data(self._genericPath), axis=0
                            )
                        else:
                            self._plot = False
                            self._plot_pattern(self._point_cloud_data)
                else:
                    print("sensor not added or pattern not set")

    def _on_save_pattern(self):
        if not self._timeline.is_playing():
            print("press play first")
            return

        self._point_cloud_data = np.empty((0, 3))
        self._plot = True
        self._record_start = time.perf_counter()

    def _plot_pattern(self, data):
        import PIL.ImageDraw as ImageDraw
        import PIL.Image as Image

        # set up plot window
        window_length = 600
        window_height = 400
        origin = [window_length / 2.0, window_height / 2.0]

        # scale data with the wall size
        cube_size = self.cubePrim.GetAttribute("xformOp:scale").Get()
        height_ratio = window_height / float(cube_size[2])
        length_ratio = window_length / float(cube_size[1])
        plot_scale = min(height_ratio, length_ratio)

        # get data that's hit the wall
        hit_yz = self.data_processing(data)

        # scale, axis_align, and center data to plot on PIL coordinate
        hit_yz = plot_scale * hit_yz
        plot_x = origin[0] - hit_yz[:, 0]
        plot_y = origin[1] - hit_yz[:, 1]

        plot_data = np.stack([plot_x, plot_y], axis=1)
        xy = plot_data.ravel()

        # actual plotting
        im = Image.new("RGB", (window_length, window_height))
        draw = ImageDraw.Draw(im)
        draw.point(xy.tolist(), fill=255)
        filename = self._filepath.get_value_as_string() + "sensor_pattern.png"
        im.save(filename)

    def data_processing(self, data):
        # only plotting when the wall is offsetted x as in the example no rotation or other axial offsets.
        # find where is the surface of the wall
        cube_pos = self.cubePrim.GetAttribute("xformOp:translate").Get()
        cube_size = self.cubePrim.GetAttribute("xformOp:scale").Get()
        wall_loc = cube_pos[0] - np.sign(cube_pos[0]) * cube_size[0] / 2

        # find in data the group that has the right offset
        hit_idx = np.where(np.isclose(data[:, 0], wall_loc, rtol=1e2))
        if len(hit_idx) == 0:
            print("no ray hit the wall")
            return np.array([])
        else:
            hit_pts = np.squeeze(data[hit_idx, 1:3])
            return hit_pts
