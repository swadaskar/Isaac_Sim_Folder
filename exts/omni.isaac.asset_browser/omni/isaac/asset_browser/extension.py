import omni.ext
import omni.kit.ui
import carb.settings
from .window import AssetBrowserWindow

ASSET_BROWSER_MENU_PATH = "Window/Browsers/Isaac Assets (Beta)"
_extension_instance = None
SETTING_ROOT = "/exts/omni.isaac.asset_browser/"
SETTING_VISIBLE_AFTER_STARTUP = SETTING_ROOT + "visible_after_startup"


class AssetBrowserExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        visible = carb.settings.get_settings().get_as_bool(SETTING_VISIBLE_AFTER_STARTUP)

        self._menu = omni.kit.ui.get_editor_menu().add_item(
            ASSET_BROWSER_MENU_PATH, self._on_click, toggle=True, value=visible
        )

        self._window = AssetBrowserWindow(visible)
        self._window.set_visibility_changed_fn(self._on_visibility_changed)

        global _extension_instance
        _extension_instance = self

    def on_shutdown(self):
        if self._window is not None:
            self._window.destroy()
            self._window = None

        global _extension_instance
        _extension_instance = None

    def _on_click(self, *args):
        self._window.visible = not self._window.visible

    def _on_visibility_changed(self, visible):
        omni.kit.ui.get_editor_menu().set_value(ASSET_BROWSER_MENU_PATH, visible)


def get_instance():
    return _extension_instance
