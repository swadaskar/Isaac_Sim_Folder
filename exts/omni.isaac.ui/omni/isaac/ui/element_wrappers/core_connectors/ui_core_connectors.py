import carb
import asyncio

import omni.ui as ui

from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import update_stage_async

from ..ui_widget_wrappers import *

from typing import Callable, List


class LoadButton(UIWidgetWrapper):
    """
    Create a special type of UI button that connects to the omni.isaac.core.World to enable convenient "Load" functionality.
    The World acts as a scene manager that simplifies user interaction with the simulator.
    This provides the user with certain guarantees at the time that their callback functions are
    called.

    The setup_scene_fn() is called with the guarantee that a World has been created.  In this function,
    the user is meant to add the asssets they want to the USD stage.  These assets then must also be added to
    the World. World is a singleton class.  And example setup_scene_fn implementation would include:
        world = World.instance() # Get the unique instance of the World
        world.scene.add(usd_object) # Add the user-loaded usd object to the scene

    The setup_post_load() function is called with the gurantees that the World has been created, the
    setup_scene_fn() has already been called, all objects that the user added to the World have been properly
    initialized, and the timeline is paused at timestep 0.

    Args:
        label (str): Short descriptive text to the left of the LoadButton
        text (str): Text on the LoadButton
        tooltip (str, optional): Text to appear when the mouse hovers over the LoadButton. Defaults to "".
        setup_scene_fn (Callable, optional): A function that will be called when the LoadButton is clicked.
            The user should use this function to add their assets to the USD stage and to add their assets
            to the World. This function should take 0 arguments.  The return value will not be used.
            Defaults to None.
        setup_post_load_fn (Callable, optional): A function that will be called when the LoadButton is clicked.
            The function is called with the gurantees that the World has been created, the
            setup_scene_fn() has already been called, all objects that the user added to the World have been properly
            initialized, and the timeline is paused at timestep 0.  This function should take 0 arguments.
            The return value will not be used.  Defaults to None.
    """

    def __init__(
        self,
        label: str,
        text: str,
        tooltip: str = "",
        setup_scene_fn: Callable = None,
        setup_post_load_fn: Callable = None,
    ):
        self.setup_scene_fn = setup_scene_fn
        self.setup_post_load_fn = setup_post_load_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

        self._world_settings = {}

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def button(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: UI Button element
        """
        return self._button

    def set_setup_scene_fn(self, setup_scene_fn: Callable):
        """
        Set the setup_scene_fn that will be called when the LoadButton is clicked.
        The setup_scene_fn() is called with the guarantee that a World has been created.  In this function,
        the user is meant to add the asssets they want to the USD stage.  These assets then must also be added to
        the World. World is a singleton class.  And example setup_scene_fn implementation would include:

        world = World.instance() # Get the unique instance of the World
        world.scene.add(usd_object) # Add the user-loaded usd object to the scene

        Args:
            setup_scene_fn (Callable): A function that will be called when the LoadButton is clicked.
                the user should use this function to add their assets to the USD stage and to add their assets
                to the World. This function should take 0 arguments.  The return value will not be used.
                Defaults to None.

		"""
        self.setup_scene_fn = setup_scene_fn

    def set_setup_scene_fn(self, setup_scene_fn: Callable):
        """
        Set the setup_scene_fn that will be called when the LoadButton is clicked.
        The setup_scene_fn() is called with the guarantee that a World has been created.  In this function,
        the user is meant to add the asssets they want to the USD stage.  These assets then must also be added to
        the World. World is a singleton class.  An example setup_scene_fn implementation would include:

        world = World.instance() # Get the unique instance of the World
        world.scene.add(usd_object) # Add the user-loaded usd object to the scene

        Args:
            setup_scene_fn (Callable): A function that will be called when the LoadButton is clicked.
                The user should use this function to add their assets to the USD stage and to add their assets
                to the World. This function should take 0 arguments.  The return value will not be used.
                Defaults to None.
        """
        self.setup_scene_fn = setup_scene_fn

    def set_setup_post_load_fn(self, setup_post_load_fn: Callable):
        """
        Set the setup_post_load_fn that will be called when the LoadButton is clicked.

        Args:
            setup_post_load_fn (Callable): A function that will be called when the LoadButton is clicked.
                The function is called with the gurantees that the World has been created, the
                setup_scene_fn() has already been called, all objects that the user added to the World have been properly
                initialized, and the timeline will be paused at timestep 0.  This function should take 0 arguments.
                The return value will not be used.  Defaults to None.
        """
        self.setup_post_load_fn = setup_post_load_fn

    def set_world_settings(self, **kwargs):
        """
        Pressing a Load Button will create a new instance of the omni.isaac.core.World.
        The default settings will be used unless the user specifies new settings at runtime before the Load Button is clicked.

        The default settings will ensure that the physics and rendering timesteps are fixed at 1/60.0 seconds (see set_defaults argument).
        It is important to note that this will ensure that code is deterministic, but may not be executed in real time.
        I.e. physics and render dts will adjust automatically if the simulation is running too fast or slow.

        Args:
            physics_dt (Optional[float], optional): dt between physics steps. Defaults to None.
            rendering_dt (Optional[float], optional): dt between rendering steps. Note: rendering means 
                rendering a frame of the current application and not 
                only rendering a frame to the viewports/ cameras. So UI
                elements of Isaac Sim will be refereshed with this dt 
                as well if running non-headless. 
                Defaults to None.
            stage_units_in_meters (Optional[float], optional): The metric units of assets. This will affect gravity value..etc.
                Defaults to None.
            physics_prim_path (Optional[str], optional): specifies the prim path to create a PhysicsScene at, 
                only in the case where no PhysicsScene already defined. 
                Defaults to "/physicsScene".
            set_defaults (bool, optional): set to True to use the defaults settings
                [physics_dt = 1.0/ 60.0,
                stage units in meters = 1 (i.e in meters),
                rendering_dt = 1.0 / 60.0,
                gravity = -9.81 m / s
                ccd_enabled,
                stabilization_enabled,
                gpu dynamics turned off,
                broadcast type is MBP,
                solver type is TGS]. Defaults to True.
            backend (str, optional): specifies the backend to be used (numpy or torch). Defaults to numpy.
            device (Optional[str], optional): specifies the device to be used if running on the gpu with torch backend.
        """
        self._world_settings = kwargs

    def _on_clicked_fn_wrapper(self):
        """This function is called when the Load Button is Clicked.
		"""

        # From an extension workflow, the stage and world need to be interacted with asynchronously

        async def _on_click_async():
            # Remove any previous World instance
            prev_world = World.instance()
            if prev_world is not None:
                prev_world.clear_all_callbacks()
                prev_world.clear_instance()
                prev_world = None
                # prev_world.clear()
            await update_stage_async()

            # Create a new World instance with user-defined settings.  See self.set_world_settings()
            world = World(**self._world_settings)

            # Call user function to put assets on the stage and add them to the World
            if self.setup_scene_fn is not None:
                self.setup_scene_fn()

            await world.reset_async()
            await world.pause_async()

            # User assets are now initialized, and the timeline is playing at timestep 0
            if self.setup_post_load_fn is not None:
                self.setup_post_load_fn()

        asyncio.ensure_future(_on_click_async())

    def _create_ui_widget(self, label: str, text: str, tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame


class ResetButton(UIWidgetWrapper):
    """
    Create a special type of UI button that connects to the omni.isaac.core.World to perform a reset.
    If no World instance exists when this button will be clicked, this button will not create one.
    In this case, the button logs a warning and calls user callback functions with no guarantees
    on the Simulator State.

    Args:
        label (str): Short descriptive text to the left of the ResetButton.
        text (str): Text on the ResetButton
        tooltip (str, optional): Text to appear when the mouse hovers over the ResetButton. Defaults to "".
        pre_reset_fn (Callable, optional): A function that will be called before resetting the World.
            This function should take 0 arguments.  The return value will not be used. Defaults to None.
        post_reset_fn (Callable, optional): A function that will be called after the World is reset.
            When this function is called, the timeline will be paused at timestep 0, and all
            USD assets added to the World will be properly initialized and placed at their default positions.
            This function should take no arguments. The return value will not be used. Defaults to None.
    """

    def __init__(
        self, label: str, text: str, tooltip="", pre_reset_fn: Callable = None, post_reset_fn: Callable = None
    ):
        self._pre_reset_fn = pre_reset_fn
        self._post_reset_fn = post_reset_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def button(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: UI Button element
        """
        return self._button

    def set_pre_reset_fn(self, pre_reset_fn: Callable):
        """Set the pre_reset_fn for when the ResetButton is clicked.

        Args:
            pre_reset_fn (Callable): A function that will be called before resetting the World.
                This function should take 0 arguments.  The return value will not be used.
        """
        self._pre_reset_fn = pre_reset_fn

    def set_post_reset_fn(self, post_reset_fn: Callable):
        """Set the post_reset_fn for when the ResetButton is clicked.

        Args:
            post_reset_fn (Callable): A function that will be called after the World is reset.
                When this function is called, the timeline will be paused at timestep 0, and all
                USD assets added to the World will be properly initialized and placed at their default locations.
                This function should take no arguments. The return value will not be used.
        """
        self._post_reset_fn = post_reset_fn

    def _on_clicked_fn_wrapper(self):
        """This function is called when the Reset Button is Clicked.
		"""

        # From an extension workflow, the stage and world need to be interacted with asynchronously

        async def _on_click_async():
            # Call user function pre_reset
            if self._pre_reset_fn is not None:
                self._pre_reset_fn()

            world = World.instance()

            if world is None:
                carb.log_warn("Reset Button was used when there is no instance of World.")
            else:
                await world.reset_async()
                await world.pause_async()

            # User assets are initialized, and the timeline is playing at timestep 0
            if self._post_reset_fn is not None:
                self._post_reset_fn()

        asyncio.ensure_future(_on_click_async())

    def _create_ui_widget(self, label: str, text: str, tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame
