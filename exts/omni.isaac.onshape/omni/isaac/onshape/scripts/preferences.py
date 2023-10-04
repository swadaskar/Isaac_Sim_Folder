import os
import pathlib
import omni
import omni.ui as ui
from omni.kit.window.preferences import PreferenceBuilder, SettingType
import carb
import asyncio

from typing import Callable

from omni.isaac.onshape import SETTINGS_PATH
from .definitions import (
    DEFAULT_ONSHAPE_KEY,
    DEFAULT_ONSHAPE_SECRET,
    ONSHAPE_BASE_URL,
    ONSHAPE_AUTH_URL,
    ONSHAPE_TOKEN_URL,
    USE_ONSHAPE_KEY,
    ONSHAPE_CHORD_TOLERANCE,
    ONSHAPE_ANGLE_TOLERANCE,
    ONSHAPE_MAX_CHORD,
    ONSHAPE_IMPORT_PHYSICS,
    ONSHAPE_FILTER_UNSUPPORTED,
    ONSHAPE_DEFAULT_FOLDER,
    ONSHAPE_IMPORT_IN_PLACE,
)
from omni.kit.window.preferences import PreferenceBuilder, SettingType, PERSISTENT_SETTINGS_PREFIX
from omni.isaac.onshape.widgets.tesselation_properties_widget import TesselationProperties


def create_filepicker(title: str, click_apply_fn: Callable = None, error_fn: Callable = None):
    from omni.kit.window.filepicker import FilePickerDialog

    async def on_click_handler(filename: str, dirname: str, dialog: FilePickerDialog, click_fn: Callable):
        dirname = dirname.strip()
        if dirname and not dirname.endswith("/"):
            dirname += "/"
        fullpath = f"{dirname}{filename}"
        if click_fn:
            click_fn(fullpath)
        dialog.hide()

    dialog = FilePickerDialog(
        title,
        allow_multi_selection=False,
        apply_button_label="Select",
        click_apply_handler=lambda filename, dirname: asyncio.ensure_future(
            on_click_handler(filename, dirname, dialog, click_apply_fn)
        ),
        click_cancel_handler=lambda filename, dirname: dialog.hide(),
        error_handler=error_fn,
    )


def get_default_path():
    try:
        home_dir = pathlib.Path.home()
    except Exception as e:
        carb.log_error(str(e))
        return ""

    config_file_path = os.path.join(home_dir, "Documents")

    return str(config_file_path)


class OnshapeImporterPreferences(PreferenceBuilder):
    def __init__(self):
        super().__init__("Onshape Importer")
        self._settings = carb.settings.get_settings()

        if self._settings.get(ONSHAPE_DEFAULT_FOLDER) is None:
            self._settings.set(ONSHAPE_DEFAULT_FOLDER, get_default_path())
        if self._settings.get(ONSHAPE_IMPORT_IN_PLACE) is None:
            self._settings.set(ONSHAPE_IMPORT_IN_PLACE, True)

        if self._settings.get(ONSHAPE_CHORD_TOLERANCE) is None:
            self._settings.set(ONSHAPE_CHORD_TOLERANCE, 0.001)
        if self._settings.get(ONSHAPE_ANGLE_TOLERANCE) is None:
            self._settings.set(ONSHAPE_ANGLE_TOLERANCE, 15)
        if self._settings.get(ONSHAPE_MAX_CHORD) is None:
            self._settings.set(ONSHAPE_MAX_CHORD, 0.2)

        if self._settings.get(USE_ONSHAPE_KEY) is None:
            # Check if env var is set, and use that
            onshape_use_key = os.environ.get("ONSHAPE_USE_API_KEY", False)
            self._settings.set(USE_ONSHAPE_KEY, onshape_use_key)
        if self._settings.get(DEFAULT_ONSHAPE_KEY) is None:
            # Check if env var is set, and use that
            onshape_key = os.environ.get("ONSHAPE_API_KEY", "")
            self._settings.set(DEFAULT_ONSHAPE_KEY, onshape_key)
        if self._settings.get(DEFAULT_ONSHAPE_SECRET) is None:
            # Check if env var is set, and use that
            onshape_secret = os.environ.get("ONSHAPE_API_SECRET", "")
            self._settings.set(DEFAULT_ONSHAPE_SECRET, onshape_secret)
        if self._settings.get(ONSHAPE_BASE_URL) is None:
            self._settings.set(ONSHAPE_BASE_URL, "https://cad.onshape.com")
        if self._settings.get(ONSHAPE_AUTH_URL) is None:
            self._settings.set(ONSHAPE_AUTH_URL, "https://oauth.onshape.com/oauth/authorize")
        if self._settings.get(ONSHAPE_TOKEN_URL) is None:
            self._settings.set(ONSHAPE_TOKEN_URL, "https://oauth.onshape.com/oauth/token")

    def cleanup_slashes(self, path: str, is_directory: bool = False) -> str:
        """
        Makes path/slashes uniform

        Args:
            path: path
            is_directory is path a directory, so final slash can be added

        Returns:
            path
        """
        path = os.path.normpath(path)
        path = path.replace(":/", "://", 1)
        if is_directory:
            if path[-1] != "/":
                path += "/"
        return path.replace("\\", "/")

    def build(self):

        with ui.VStack(height=0):
            with self.add_frame("General"):
                with ui.VStack():
                    self.create_setting_widget(
                        "Use open stage folder for Imported Assembly", ONSHAPE_IMPORT_IN_PLACE, SettingType.BOOL
                    )
                    with ui.HStack(height=24):
                        self.label("Default Import Folder")
                        widget = ui.StringField(height=20)
                        widget.model.set_value(self.default_save_folder)
                        ui.Button(
                            style={"image_url": "resources/icons/folder.png"},
                            clicked_fn=lambda p=self.cleanup_slashes(
                                self.default_save_folder
                            ), w=widget: self._on_browse_button_fn(p, w),
                            width=24,
                        )

                    self.create_setting_widget("Rig Physics when importing", ONSHAPE_IMPORT_PHYSICS, SettingType.BOOL)
                    self.create_setting_widget(
                        "Filter Unsupported document elements", ONSHAPE_FILTER_UNSUPPORTED, SettingType.BOOL
                    )
            ui.Spacer(height=5)
            with self.add_frame("Tesselation"):
                with ui.VStack():
                    self.create_setting_widget("Chord Tolerance (m)", ONSHAPE_CHORD_TOLERANCE, SettingType.FLOAT)
                    self.create_setting_widget("Angle Tolerance (d)", ONSHAPE_ANGLE_TOLERANCE, SettingType.FLOAT)
                    self.create_setting_widget("Max Cord (m)", ONSHAPE_MAX_CHORD, SettingType.FLOAT)
            ui.Spacer(height=5)
            with self.add_frame("Authentication"):
                with ui.VStack():
                    self.create_setting_widget("Base Url", ONSHAPE_BASE_URL, SettingType.STRING)
                    self.create_setting_widget("Auth Url", ONSHAPE_AUTH_URL, SettingType.STRING)
                    self.create_setting_widget("Token Url", ONSHAPE_TOKEN_URL, SettingType.STRING)
                    api_cb = self.create_setting_widget("Use API Authentication", USE_ONSHAPE_KEY, SettingType.BOOL)
                    api_cb_model = api_cb.model
                    self.api_frame = ui.HStack()
                    with self.api_frame:
                        ui.Spacer(width=15)
                        with ui.VStack():
                            self.create_setting_widget("API Key", DEFAULT_ONSHAPE_KEY, SettingType.STRING)
                            secret = self.create_setting_widget(
                                "API Secret", DEFAULT_ONSHAPE_SECRET, SettingType.STRING
                            )
                            secret.password_mode = True
                        ui.Spacer(width=15)
                    self.api_frame.visible = self.use_api_keys
                    api_cb_model.add_value_changed_fn(lambda a, w=self.api_frame: self.on_use_api_changed(a, widget=w))

    def on_use_api_changed(self, show_frame, widget):
        widget.visible = self.use_api_keys
        if not self.use_api_keys:
            self._settings.set(DEFAULT_ONSHAPE_KEY, "")
            self._settings.set(DEFAULT_ONSHAPE_SECRET, "")

    def _on_browse_button_fn(self, path, widget):
        """Called when the user picks the Browse button."""
        file_pick = create_filepicker(
            title="Select Template Directory",
            click_apply_fn=lambda p=self.cleanup_slashes(path), w=widget: self._on_file_pick(p, widget=w),
        )
        # file_pick.show(path)

    def _on_file_pick(self, full_path, widget):
        """Called when the user accepts directory in the Select Directory dialog."""
        directory = self.cleanup_slashes(full_path, True)
        self._settings.set(ONSHAPE_DEFAULT_FOLDER, directory)
        widget.model.set_value(directory)

    def get_working_folder(self):
        if self._settings.get(ONSHAPE_IMPORT_IN_PLACE):
            stage = omni.usd.get_context().get_stage()
            if "anon" in stage.GetRootLayer().identifier:
                return self.default_save_folder
            return stage.GetRootLayer().identifier
        return self.default_save_folder
        # tmp_prefix = "tmp_isaac_onshape_importer_"
        # if self.default_save_folder:
        # tempdir = tempfile.TemporaryDirectory(
        #     prefix=tmp_prefix,
        # ).name

    @property
    def tesselation_properties(self):
        return TesselationProperties(
            self._settings.get(ONSHAPE_CHORD_TOLERANCE),
            self._settings.get(ONSHAPE_ANGLE_TOLERANCE),
            self._settings.get(ONSHAPE_MAX_CHORD),
        )

    @property
    def rig_physics(self):
        return self._settings.get(ONSHAPE_IMPORT_PHYSICS)

    @property
    def filter_unsupported(self):
        return self._settings.get(ONSHAPE_FILTER_UNSUPPORTED)

    @property
    def default_save_folder(self):
        path = self._settings.get(ONSHAPE_DEFAULT_FOLDER)
        if not path or (path and type(path) is not str):
            path = ""
        return path

    @property
    def use_api_keys(self):
        return self._settings.get(USE_ONSHAPE_KEY)
