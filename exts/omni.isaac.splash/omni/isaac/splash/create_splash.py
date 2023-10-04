# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import asyncio
import glob
import os
import random
from typing import List

import carb
import carb.imgui
import carb.settings
import carb.tokens
import carb.windowing

import omni.ext
import omni.appwindow
import omni.kit.app
import omni.ui as ui


class IsaacSplashExtension(omni.ext.IExt):
    """Splash screen for isaac."""

    def __init__(self):
        self._window: ui.Window = None
        self._build_task: asyncio.Task[None] = None
        self._loop_task: asyncio.Task[None] = None

        self._carousel_left_image_path: str = None
        self._carousel_right_image_path: str = None
        self._carousel_bullet_image_path: str = None
        self._carousel_bullet_active_image_path: str = None
        self._slideshow_image_paths: List[str] = []
        self._slideshow_image: ui.Image = None
        self._navigation_bullets: List[ui.Image] = []
        self._current_slide_index: int = 0

        settings = carb.settings.get_settings()
        # Glob pattern of images paths to search for, and to use for slides of the slideshow (relative to this
        # extension's directory):
        self._slides_file_pattern: str = settings.get_as_string("exts/omni.isaac.splash/slides_file_pattern")
        # Maximum number of images to display in the slideshow:
        self._max_slides_count: int = settings.get_as_int("exts/omni.isaac.splash/max_slides_count")
        # Duration for which to display each individual slide before moving to the next one (in seconds):
        self._slide_hold_time: float = settings.get_as_float("exts/omni.isaac.splash/slide_hold_time")

    def _generate_slide_image_paths(self, image_paths_pattern: str) -> List[str]:
        """
        Return the list of image paths which can be used as slides for the splash screen carousel, taking at most
        `max_slides_count` images.

        Args:
            image_paths_pattern (str): A glob pattern from which to build a list of candidate images to use as slides
                for the splash screen carousel.

        Returns:
            (List[str]): A list of at most `max_slides_count` image paths which can be used as slides for the image
                carousel.

        """
        image_paths = glob.glob(image_paths_pattern, recursive=True)
        return random.sample(
            image_paths,
            # Take at most `max_slides_count` images from the number of candidate images matching the given pattern:
            min(len(image_paths), self._max_slides_count),
        )

    def _set_current_slide_index(self, slide_index: int, from_user_interaction: bool = False) -> None:
        """
        Display the slide at the given index from the list of slideshow images. This also updates the "active"
        navigation bullet to reflect the current index, and reset the timer used to automatically move to the next slide
        in case the action was initiated by the User. This prevents the slide from quickly changing to the next slide in
        line after a User interaction in case the countdown was nearing completion.

        Args:
            slide_index (int): Index of the slide to display.
            from_user_interaction (bool): Flag indicating whether the action was initiated by the User, used to inform
                if the countdown to the next image should be reset upon completing the action (default: `False`).

        Returns:
            None

        """
        if not self._slideshow_image or slide_index < 0 or slide_index > len(self._slideshow_image_paths):
            return

        # Update the "hero" slideshow image:
        self._current_slide_index = slide_index
        self._slideshow_image.source_url = self._slideshow_image_paths[self._current_slide_index]

        # Update the navigation bullets to reflect the slideshow image currently being displayed:
        for bullet_image_index in range(len(self._navigation_bullets)):
            bullet_image_source = self._carousel_bullet_image_path
            if bullet_image_index == self._current_slide_index:
                bullet_image_source = self._carousel_bullet_active_image_path
            self._navigation_bullets[bullet_image_index].source_url = bullet_image_source

        # If the action was initiated by the User, reset the countdown to move to the next slide, in order to abruptly
        # move to the next image in the queue if the countdown was nearing completion when the action was performed:
        if from_user_interaction:
            self._reset_slide_countdown()

    def _move_to_previous_slide(self, from_user_interaction: bool = False) -> None:
        """
        Change the "hero" image of the slideshow to the previous one in the list.

        Args:
            from_user_interaction (bool): Flag indicating whether the action was initiated by the User, used to inform
                if the countdown to the next image should be reset upon completing the action (default: `False`).

        Returns:
            None

        """
        desired_slide_index = self._current_slide_index - 1
        if desired_slide_index < 0:
            desired_slide_index = len(self._slideshow_image_paths) - 1
        self._set_current_slide_index(desired_slide_index, from_user_interaction)

    def _move_to_next_slide(self, from_user_interaction: bool = False) -> None:
        """
        Change the "hero" image of the slideshow to the next one in the list.

        Args:
            from_user_interaction (bool): Flag indicating whether the action was initiated by the User, used to inform
                if the countdown to the next image should be reset upon completing the action (default: `False`).

        Returns:
            None

        """
        desired_slide_index = (self._current_slide_index + 1) % len(self._slideshow_image_paths)
        self._set_current_slide_index(desired_slide_index, from_user_interaction)

    async def _initialize_slideshow_countdown(self) -> None:
        """Initialize the countdown Task which rotates the slide to the next one at `_slide_hold_time` interval."""
        while True:
            await asyncio.sleep(self._slide_hold_time)
            self._move_to_next_slide()

    def _reset_slide_countdown(self) -> None:
        """Reset the timer counting down to the next slide."""
        if self._loop_task:
            self._loop_task.cancel()
        self._loop_task = asyncio.ensure_future(self._initialize_slideshow_countdown())

    def on_startup(self, ext_id) -> None:
        extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        SLIDE_IMAGE_PATHS_PATTERN = os.path.join(extension_path, self._slides_file_pattern)
        ICON_PATH = os.path.join(extension_path, "icons")

        theme = "dark"  # Only one theme is available at the moment.
        self._carousel_left_image_path = os.path.join(ICON_PATH, f"carousel_left_{theme}.svg")
        self._carousel_right_image_path = os.path.join(ICON_PATH, f"carousel_right_{theme}.svg")
        self._carousel_bullet_image_path = os.path.join(ICON_PATH, f"carousel_bullet_{theme}.svg")
        self._carousel_bullet_active_image_path = os.path.join(ICON_PATH, f"carousel_bullet_active_{theme}.svg")

        self._slideshow_image_paths = self._generate_slide_image_paths(SLIDE_IMAGE_PATHS_PATTERN)

        # Configure `carb.imgui` style overrides before building the UI of the splash screen:
        imgui = carb.imgui.acquire_imgui()
        # Remove the docking spliter size, to avoid making it look like there is a narrow border around the images:
        imgui.push_style_var_float(carb.imgui.StyleVar.DockSplitterSize, 0)

        self._build_ui()

        self._build_task = asyncio.ensure_future(self._build_layout())
        if len(self._slideshow_image_paths) > 1:
            self._reset_slide_countdown()

    def _build_ui(self) -> None:
        """
        Build the UI for the Splash screen.

        At a high level, the UI is conceptually a 2-layer, 3-column layout with the "hero" slideshow image at the
        background layer of a ZStack with the controls overlayed on the foreground layer.

        The control layer itself is a 2-row layout, with the top row containing navigation controls aligned to the
        bottom, and the bottom row serving as a spacer to position the control relative to the bottom of the Window.

        Returns:
            None

        """
        ICON_WIDTH = 10

        self._window = ui.Window(title="Splash", style={"Window": {"padding": 0}}, padding_x=0, padding_y=0)

        with self._window.frame:
            with ui.ZStack():
                # Main slide image (at the background layer of the ZStack):
                with ui.VStack():
                    self._slideshow_image = ui.Image(
                        self._slideshow_image_paths[self._current_slide_index],
                        fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                        alignment=ui.Alignment.CENTER,
                    )

                # Slideshow controls (at the foreground layer of the ZStack):
                with ui.VStack():
                    with ui.HStack(spacing=ICON_WIDTH):
                        ui.Spacer()

                        # "Previous" caret:
                        ui.Image(
                            self._carousel_left_image_path,
                            mouse_pressed_fn=lambda x, y, button, modifier: self._move_to_previous_slide(
                                from_user_interaction=True
                            ),
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.RIGHT_BOTTOM,
                            width=ICON_WIDTH,
                        )

                        # Navigation bullets:
                        with ui.VStack(width=0):
                            with ui.HStack(spacing=ICON_WIDTH):
                                for slide_index in range(len(self._slideshow_image_paths)):
                                    bullet_image_source = self._carousel_bullet_image_path
                                    if slide_index == self._current_slide_index:
                                        bullet_image_source = self._carousel_bullet_active_image_path

                                    bullet_image = ui.Image(
                                        bullet_image_source,
                                        mouse_pressed_fn=lambda x, y, button, modifier, slide_index=slide_index: self._set_current_slide_index(
                                            slide_index, from_user_interaction=True
                                        ),
                                        fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                        alignment=ui.Alignment.BOTTOM,
                                        width=ICON_WIDTH,
                                    )
                                    self._navigation_bullets.append(bullet_image)

                        # "Next" caret:
                        ui.Image(
                            self._carousel_right_image_path,
                            mouse_pressed_fn=lambda x, y, button, modifier: self._move_to_next_slide(
                                from_user_interaction=True
                            ),
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.LEFT_BOTTOM,
                            width=ICON_WIDTH,
                        )

                        ui.Spacer()

                    # Offset from the bottom of the window:
                    ui.Spacer(height=12)

    async def _build_layout(self) -> None:
        """Build the layout of the Splash screen."""
        await omni.kit.app.get_app().next_update_async()

        splash_handle = ui.Workspace.get_window("Splash")
        if splash_handle is None:
            return

        # Setup the docking Space:
        main_dockspace = ui.Workspace.get_window("DockSpace")
        splash_handle.dock_in(main_dockspace, ui.DockPosition.SAME)
        splash_handle.dock_tab_bar_visible = False

    def on_shutdown(self) -> None:
        self._window = None

        if self._build_task:
            self._build_task.cancel()
            self._build_task = None
        if self._loop_task:
            self._loop_task.cancel()
            self._loop_task = None

        self._slideshow_image_paths = []
        self._slideshow_image = None
        self._navigation_bullets = []
        self._current_slide_index = 0
