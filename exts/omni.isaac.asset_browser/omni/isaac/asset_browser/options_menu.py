from omni.kit.browser.core import OptionMenuDescription, OptionsMenu
from omni.kit.browser.folder.core import FolderBrowserModel
from .delegate import MaterialDetailDelegate


class AssetOptionsMenu(OptionsMenu):
    """
    Represent options menu used in material browser.
    Args:
        delegate (MaterialDetailDelegate): Detail delegate used in the window.
    """

    def __init__(self, delegate: MaterialDetailDelegate):
        super().__init__()

        self.append_menu_item(OptionMenuDescription("", None))
        self.append_menu_item(
            OptionMenuDescription("Download Current Collection", clicked_fn=self._on_download_collection)
        )

    def _on_generate_item_thumbnail(self) -> None:
        for detail_item in self._browser_widget.detail_selection:
            self._delegate.generate_thumbnail(detail_item)

    def _on_generate_category_thumbnails(self) -> None:
        selection = self._browser_widget.category_selection
        if selection:
            for detail_item in self._browser_widget.model.get_detail_items(selection[0]):
                self._delegate.generate_thumbnail(detail_item)

    def _on_download_collection(self) -> None:
        collection_item = self._browser_widget.collection_selection
        if collection_item:
            pass
