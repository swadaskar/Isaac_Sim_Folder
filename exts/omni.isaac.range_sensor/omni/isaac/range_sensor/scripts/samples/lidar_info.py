# Copyright (c) 2018-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.range_sensor import _range_sensor
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from pxr import UsdGeom, UsdLux, Sdf, Gf, UsdPhysics
import asyncio
import weakref

from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder, combo_cb_scrolling_frame_builder
from omni.isaac.core.utils.viewports import set_camera_view

EXTENSION_NAME = "LIDAR Info"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        self._ext_id = ext_id

        # The extension acquires the LIDAR interface at startup.  It will be released during extension shutdown.  We
        # create a LIDAR prim using our schema, and then we interact with / query that prim using the python API found
        # in lidar/bindings
        self._li = _range_sensor.acquire_lidar_sensor_interface()
        self.lidar = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._menu_items = [
            MenuItemDescription(
                name="Sensors",
                sub_menu=[
                    make_menu_item_description(ext_id, "LIDAR", lambda a=weakref.proxy(self): a._menu_callback())
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

        self._editor_event_subscription = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_editor_step)
        )

        self._build_ui()

    def _build_ui(self):
        # This just defines the window we will use to access the lidar_info GUI.  Note that clicking on the menu item
        # does not create an instance of lidar_info; that is done by the extension when it is loaded by kit.  All this
        # menu does is show or hide our GUI we will use for interacting with lidar_info
        self._window = omni.ui.Window(
            EXTENSION_NAME, width=700, height=0, visible=False, dockPreference=omni.ui.DockPreference.LEFT_BOTTOM
        )
        with self._window.frame:
            with ui.VStack(spacing=5, height=10):
                title = "Read a LIDAR Data Stream"
                doc_link = (
                    "https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_range_sensor.html"
                )

                overview = (
                    "This example shows how to create a LIDAR, set its properties, and read data streaming from it. "
                )
                overview += "First press the 'Load LIDAR' button and then press PLAY to simulate."
                overview += "\n\nPress the 'Open in IDE' button to view the source code."
                overview += "\nNote: The buttons above only work with a LIDAR made by the 'Load LIDAR' button; not existing ones in the stage."

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
                            "label": "Load LIDAR",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads a LIDAR Sensor and sets its properties",
                            "on_clicked_fn": self._on_spawn_lidar_button,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Load LIDAR Scene",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Loads a obstacles for the LIDAR to sense",
                            "on_clicked_fn": self._on_spawn_obstacles_button,
                        }
                        btn_builder(**dict)

                        dict = {
                            "label": "Show Data Stream",
                            "type": "checkbox_scrolling_frame",
                            "default_val": [False, "No Data To Display"],
                            "tooltip": "Show incoming data from an active LIDAR",
                        }
                        self._info_cb, self._info_label = combo_cb_scrolling_frame_builder(**dict)

    def on_shutdown(self):
        # Perform cleanup once the sample closes
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
        self._editor_event_subscription = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    async def _spawn_lidar_function(self, task):
        # Wait for stage clear to complete before creating LIDAR
        done, pending = await asyncio.wait({task})
        if task in done:
            stage = omni.usd.get_context().get_stage()

            # Set up axis to z.  The LIDAR extension scans in the XZ plane, which is assumed to be perpendicular to the
            # rotational plane, and so to use the LIDAR as it is currently written, Z must be up.
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)

            # Create the PhysicsScene.  The lidar is going to execute line trace calls in PhysX, and return a value based
            # on how far it travels before colliding with an object that is using the PhysX collision API.  Because of this,
            # to use the LIDAR extension, you MUST have a physics scene defined
            UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

            # create the LIDAR.  Before we can set any attributes on our LIDAR, we must first create the prim using our
            # LIDAR schema, and then populate it with the parameters we will be manipulating.  If you try to manipulate
            # a parameter before creating it, you will get a runtime error
            self.lidarPath = "/World/Lidar"
            self.lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(self.lidarPath))

            # Horizontal and vertical field of view in degrees
            self.lidar.CreateHorizontalFovAttr().Set(360.0)
            self.lidar.CreateVerticalFovAttr().Set(10)

            # Rotation rate in Hz
            self.lidar.CreateRotationRateAttr().Set(20.0)

            # Horizontal and vertical resolution in degrees.  Rays will be fired on the bin boundries defined by the
            # resolution.  If your FOV is 45 degrees and your resolution is 15 degrees, you will get rays at
            # 0, 15, 30, and 45 degrees.
            self.lidar.CreateHorizontalResolutionAttr().Set(1.0)
            self.lidar.CreateVerticalResolutionAttr().Set(1.0)

            # Min and max range for the LIDAR.  This defines the starting and stopping locations for the linetrace
            self.lidar.CreateMinRangeAttr().Set(0.4)
            self.lidar.CreateMaxRangeAttr().Set(100.0)

            # These attributes affect drawing the lidar in the viewport.  High Level Of Detail (HighLod) = True will draw
            # all rays.  If false it will only draw horizontal rays.  Draw Lidar Points = True will draw the actual
            # LIDAR rays in the viewport.
            self.lidar.CreateHighLodAttr().Set(True)
            self.lidar.CreateDrawPointsAttr().Set(False)
            self.lidar.CreateDrawLinesAttr().Set(False)

            # We set the attributes we created.  We could have just set the attributes at creation, but this was
            # more illustrative.  It's important to remember that attributes do not exist until you create them; even
            # if they are defined in the schema.
            self.lidar.GetRotationRateAttr().Set(0.5)
            self.lidar.GetDrawLinesAttr().Set(True)
            self.lidar.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.250))

            # we want to make sure we can see the lidar we made, so we set the camera position and look target
            set_camera_view(eye=[5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

    def _on_spawn_lidar_button(self):
        # wait for new stage before creating lidar
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._spawn_lidar_function(task))
        # refresh data stream box
        self._info_label.text = ""

    def _on_editor_step(self, step):
        if self._info_cb.get_value_as_bool():
            if self._timeline.is_playing():
                self._get_info_function()
        else:
            self._info_label.text = ""

    def _on_spawn_obstacles_button(self):
        stage = omni.usd.get_context().get_stage()
        self.CubePath = "/World/Cube"
        offset = Gf.Vec3f(-2.00, 0.0, 0.500)
        size = 1.00

        # Define a light so we can see the obstacle better
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)

        # To create a cube, we first define our geometry at our chosen path.  Then, becuase
        # we will need the primitive later, we query the prim from the stage. If the prim already exists, skip creation
        if stage.GetPrimAtPath(self.CubePath):
            return
        cubeGeom = UsdGeom.Cube.Define(stage, self.CubePath)
        cubePrim = stage.GetPrimAtPath(self.CubePath)

        # Remember!  Attributes do not exist until they are created.  Here we set the value to the non defualt at
        # creation.  Note that moving the cube to a different location involves adding a translation operation to
        # our primitive.
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)

        # In order for our cube to interact with the LIDAR, it needs to be able to colide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(cubePrim)

    def _get_info_function(self, val=False):
        if not self.lidar:
            return
        maxDepth = self.lidar.GetMaxRangeAttr().Get()
        self._info_label.text = ""
        # The LIDAR itself exists as a C++ object.  In order to retrieve data from this object we need to call
        # C++ code, but this is handled for us through the use of python bindings.  Here we get the depth value of
        # each ray, and the spherical coordinates of each ray in (azimuth, zenith).
        depth = self._li.get_depth_data(self.lidarPath)
        zenith = self._li.get_zenith_data(self.lidarPath)
        azimuth = self._li.get_azimuth_data(self.lidarPath)

        # most of the below is string formatting in order to display our data in a nice table within our GUI.
        tableString = ""

        numCols = len(zenith)
        rowString = ""
        for i in range(numCols):
            rowString += "{" + str(i + 2) + ":." + str(5) + "f}   "
        rowString = "{0:16}  {1:10}" + rowString + "\n"

        tableString += rowString.format("Azimuth \ Zenith", " | ", *zenith)
        tableString += "-" * len(tableString) + "\n"
        for row, cols in enumerate(depth):
            # The data on the c++ side is stored as uint16.  in order to get our depth values into centimeters, we
            # must first convert from uint16 into float on [0,1], and then scale to the maximum distance.
            entry = [ray * maxDepth / 65535.0 for ray in cols]
            tableString += rowString.format("{0:.5f}".format(azimuth[row]), " | ", *entry)

        self._info_label.text = tableString
