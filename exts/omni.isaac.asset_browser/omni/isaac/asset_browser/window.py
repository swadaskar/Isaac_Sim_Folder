import os
import carb.settings
import omni.ui as ui
from omni.kit.browser.folder.core import FolderBrowserWidget
from .delegate import AssetDetailDelegate
from .model import AssetBrowserModel


class AssetBrowserWindow(ui.Window):
    """
    Represent a window to show Assets
    """

    def __init__(self, visible=True):
        super().__init__("Isaac Assets (Beta)", width=500, height=600, visible=visible)

        self.frame.set_build_fn(self._build_ui)

        # Dock it to the same space where Stage is docked, make it active.
        self.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

    def _build_ui(self):
        preload_folder = os.path.abspath(carb.tokens.get_tokens_interface().resolve("${app}/../predownload"))
        self._browser_model = AssetBrowserModel(
            filter_file_suffixes=[".usd", ".usda", ".usdc"],
            timeout=carb.settings.get_settings().get("/exts/omni.isaac.asset_browser/data/timeout"),
        )
        self._delegate = AssetDetailDelegate(self._browser_model)

        with self.frame:
            with ui.VStack(spacing=15):
                self._widget = FolderBrowserWidget(
                    self._browser_model, detail_delegate=self._delegate, predownload_folder=preload_folder
                )
