import omni.ui as ui
from omni.isaac.ui.widgets import DynamicComboBoxModel
from omni.kit.window.property.templates import LABEL_WIDTH, LABEL_HEIGHT
from omni.isaac.core.utils.prims import get_prim_object_type
from pxr import Usd

from cmath import inf
import sys
from typing import Callable, List

from omni.isaac.ui.ui_utils import (
    xyz_builder,
    progress_bar_builder,
    plot_builder,
    xyz_plot_builder,
    add_line_rect_flourish,
    format_tt,
    on_copy_to_clipboard,
)

import omni.physx as _physx
import carb
from omni.usd import get_context

import sys

from omni.kit.window.filepicker import FilePickerDialog
import omni.ui as ui
from omni.isaac.ui.ui_utils import get_style, BUTTON_WIDTH

from .base_ui_element_wrappers import UIWidgetWrapper


##########################################################################################
#                                 UI Frame Wrappers
##########################################################################################


class Frame(UIWidgetWrapper):
    """Create a Frame UI element

        Args:
            enabled (bool, optional): Frame is enabled. Defaults to True.
            visible (bool, optional): Frame is visible. Defaults to True.
            build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
                Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(self, enabled: bool = True, visible: bool = True, build_fn: Callable = None):
        # Create a Frame UI element
        self._frame = self._create_frame(enabled, visible, build_fn)
        super().__init__(self.frame)

    @property
    def frame(self) -> ui.Frame:
        """
        Returns:
            omni.ui.Frame: A UI Frame
        """
        return self._frame

    def rebuild(self):
        """
        Rebuild the Frame using the specified build_fn
        """
        self.frame.rebuild()

    def set_build_fn(self, build_fn: Callable):
        """Set the build_fn to use when rebuilding the frame.

        Args:
            build_fn (Callable): Build function to use when rebuilding the frame.  Function should take
                no arguments.  Return values will not be used.
        """
        self.frame.set_build_fn(build_fn)

    def __enter__(self):
        self.frame.__enter__()

    def __exit__(self, *args):
        self.frame.__exit__(*args)

    def _create_frame(self, enabled: bool, visible: bool, build_fn: Callable) -> ui.CollapsableFrame:
        frame = ui.Frame(
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="Frame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class CollapsableFrame(Frame):
    """Create a CollapsableFrame UI element

        Args:
            title (str): Title of Collapsable Frame
            collapsed (bool, optional): Frame is collapsed. Defaults to True.
            enabled (bool, optional): Frame is enabled. Defaults to True.
            visible (bool, optional): Frame is visible. Defaults to True.
            build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
                Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(
        self, title: str, collapsed: bool = True, enabled: bool = True, visible: bool = True, build_fn: Callable = None
    ):
        # Create a Frame UI element
        self._frame = self._create_frame(title, collapsed, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    @property
    def collapsed(self) -> bool:
        """
        Returns:
            bool: CollapsableFrame is collapsed
        """
        return self.frame.collapsed

    @collapsed.setter
    def collapsed(self, value: bool):
        self.frame.collapsed = value

    @property
    def title(self) -> str:
        """
        Returns:
            str: Title text of CollapsableFrame
        """
        return self.frame.title

    @title.setter
    def title(self, value: str):
        self.frame.title = value

    def _create_frame(
        self, title: str, collapsed: bool, enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.CollapsableFrame:
        frame = ui.CollapsableFrame(
            title=title,
            name=title,
            height=0,
            collapsed=collapsed,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class ScrollingFrame(Frame):
    """Create a ScrollingFrame UI element with a specified size.

        Args:
            num_lines (int, optional): Determines height of ScrollingFrame element in terms of the 
                typical line height of UI elements. Defaults to 5.
            enabled (bool, optional): Frame is enabled. Defaults to True.
            visible (bool, optional): Frame is visible. Defaults to True.
            build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
                Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(self, num_lines=5, enabled: bool = True, visible: bool = True, build_fn: Callable = None):
        # Create a Frame UI element
        self._frame = self._create_frame(num_lines, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    def set_num_lines(self, num_lines: int):
        """Set the height of the ScrollingFrame element in terms of the typical line height of 
        other UI elements.

        Args:
            num_lines (int): Number of lines that should be shown in a ScrollingFrame.
        """
        self.frame.height = LABEL_HEIGHT * num_lines

    def _create_frame(self, num_lines: int, enabled: bool, visible: bool, build_fn: Callable) -> ui.CollapsableFrame:
        frame = ui.ScrollingFrame(
            height=LABEL_HEIGHT * num_lines,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="ScrollingFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


##########################################################################################
#                           Simple Editable UI Field Wrappers
##########################################################################################


class IntField(UIWidgetWrapper):
    """
    Creates a IntField UI element.

    Args:
        label (str): Short descriptive text to the left of the IntField.
        tooltip (str, optional): Text to appear when the mouse hovers over the IntField. Defaults to "".
        default_value (int, optional): Default value of the IntField. Defaults to 0.
        lower_limit (int, optional): Lower limit of float. Defaults to None.
        upper_limit (int, optional): Upper limit of float. Defaults to None.
        on_value_changed_fn (Callable, optional): Function to be called when the value of the int is changed.
            The function should take an int as an argument.  The return value will not be used. Defaults to None.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: int = 0,
        lower_limit: int = None,
        upper_limit: int = None,
        on_value_changed_fn: Callable = None,
    ):
        self._lower_limit = lower_limit
        self._upper_limit = upper_limit

        self._default_value = default_value

        self._on_value_changed_fn = on_value_changed_fn

        int_field_frame = self._create_ui_widget(label, tooltip, default_value)

        super().__init__(int_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def int_field(self) -> ui.IntField:
        """
        Returns:
            omni.ui.IntField: UI IntField elements
        """
        return self._int_field

    def get_value(self) -> int:
        """Get the current value of the int field

        Returns:
            int: Current value of the int field
        """
        return self.int_field.model.get_value_as_int()

    def get_upper_limit(self) -> int:
        """Get the upper limit on the IntField.

        Returns:
            int: Upper Limit on IntField
        """
        if self._upper_limit is None:
            return sys.maxsize
        return self._upper_limit

    def get_lower_limit(self) -> int:
        """Get the lower limit on the IntField.

        Returns:
            int: Lower Limit on IntField
        """
        if self._lower_limit is None:
            return sys.maxsize * -1
        return self._lower_limit

    def set_value(self, val: int):
        """Set the value in the IntField

        Args:
            val (int): Value to fill IntField
        """
        self.int_field.model.set_value(val)

    def set_upper_limit(self, upper_limit: int):
        """Set upper limit of IntField.
        If current value is higher than upper_limit, the current value will be clipped to upper_limit

        Args:
            upper_limit (int): Upper limit of IntField
        """
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: int):
        """Set lower limit of IntField.
        If current value is lower than lower_limit, the current value will be clipped to lower_limit

        Args:
            lower_limit (int): lower limit of IntField
        """
        self._lower_limit = lower_limit
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the IntField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the IntField is modified.
                Function should take a int as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model):
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())
        val = model.get_value_as_int()

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._int_field = ui.IntDrag(
                    name="Field",
                    height=LABEL_HEIGHT,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=sys.maxsize * -1,
                    max=sys.maxsize,
                )
                self.int_field.model.set_value(default_value)
                add_line_rect_flourish(False)
            self.int_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame


class FloatField(UIWidgetWrapper):
    """
    Creates a FloatField UI element.

    Args:
        label (str): Short descriptive text to the left of the FloatField.
        tooltip (str, optional): Text to appear when the mouse hovers over the FloatField. Defaults to "".
        default_value (float, optional): Default value of the Float Field. Defaults to 0.0.
        step (float, optional): Smallest increment that the user can change the float by when dragging mouse. Defaults to 0.01.
        format (str, optional): Formatting string for float. Defaults to "%.2f".
        lower_limit (float, optional): Lower limit of float. Defaults to None.
        upper_limit (float, optional): Upper limit of float. Defaults to None.
        on_value_changed_fn (Callable, optional): Function to be called when the value of the float is changed.
            The function should take a float as an argument.  The return value will not be used. Defaults to None.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: float = 0.0,
        step: float = 0.01,
        format: str = "%.2f",
        lower_limit: float = None,
        upper_limit: float = None,
        on_value_changed_fn: Callable = None,
    ):
        self._lower_limit = lower_limit
        self._upper_limit = upper_limit

        self._default_value = default_value

        self._on_value_changed_fn = on_value_changed_fn

        float_field_frame = self._create_ui_widget(label, tooltip, default_value, step, format)

        super().__init__(float_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def float_field(self) -> ui.FloatField:
        """
        Returns:
            omni.ui.FloatField: UI FloatField element
        """
        return self._float_field

    def get_value(self) -> float:
        """Return the current value of the FloatField

        Returns:
            float: Current value of the FloatField
        """
        return self.float_field.model.get_value_as_float()

    def get_upper_limit(self) -> float:
        """Get the upper limit on the FloatField

        Returns:
            float: Upper limit on FloatField
        """
        if self._upper_limit is None:
            return inf
        return self._upper_limit

    def get_lower_limit(self) -> float:
        """Get the lower limit on the FloatField

        Returns:
            float: Lower limit on FloatField
        """
        if self._lower_limit is None:
            return -inf
        return self._lower_limit

    def set_value(self, val: float):
        """Set the value in the FloatField

        Args:
            val (float): Value to fill FloatField
        """
        self.float_field.model.set_value(float(val))

    def set_upper_limit(self, upper_limit: float):
        """Set upper limit of FloatField.
        If current value is higher than upper_limit, the current value will be clipped to upper_limit

        Args:
            upper_limit (float): Upper limit of FloatField
        """
        self._upper_limit = float(upper_limit)
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: float):
        """Set lower limit of FloatField.
        If current value is lower than lower_limit, the current value will be clipped to lower_limit

        Args:
            lower_limit (float): lower limit of FloatField
        """
        self._lower_limit = float(lower_limit)
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the FloatField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the FloatField is modified.
                Function should take a float as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model):
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())
        val = model.get_value_as_float()
        if self.get_upper_limit() < val:
            val = self._upper_limit
            model.set_value(float(val + 1))
            return
        elif self.get_lower_limit() > val:
            val = self._lower_limit
            model.set_value(float(val - 1))
            return

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value, step, format):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._float_field = ui.FloatDrag(
                    name="FloatField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=-inf,
                    max=inf,
                    step=step,
                    format=format,
                )
                self.float_field.model.set_value(default_value)
                add_line_rect_flourish(False)

            self.float_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame


class StringField(UIWidgetWrapper):
    """Create StringField UI Element.
    
    Starting at use_folder_picker, the arguments to the StringField all pertain to the folder_picker.
    If the folder_picker is not used, these arguments may all be ignored.
    

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".
        default_val (str, optional): Text to initialize in Stringfield. Defaults to " ".
        read_only (bool, optional): Prevents editing. Defaults to False.
        multiline_okay (bool, optional): If True, allow newline character in input strings. Defaults to False.
        on_value_changed_fn (Callable, optional) Function called when value of StringField is changed.
            The function should take a string as an argument.  The return value will not be used. Defaults to None.
        use_folder_picker (bool, optional): Add a folder picker button to the right. Defaults to False.
        item_filter_fn (Callable, optional): Filter function to pass to the FilePicker.  This function should take a string
            as an argument and return a boolean.  When the user opens the file picker, every file in the directory they are
            viewing will be passed to item_filter_fn, and when True is returned, the file will be shown.  When False is
            returned, the file will not be shown.  This can be used to ensure that the user may only select valid file types.
        bookmark_label (str, optional): Bookmark label to pass to the FilePicker.  This will create a bookmark when the 
            file picker is used with the label specified here.
        bookmark_path (str, optional): Bookmark path to pass to the FilePicker.  This will create a bookmark when the file
            picker is used with the path specified here.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: str = "",
        read_only=False,
        multiline_okay=False,
        on_value_changed_fn: Callable = None,
        use_folder_picker=False,
        item_filter_fn=None,
        bookmark_label=None,
        bookmark_path=None,
        folder_dialog_title="Select Output Folder",
        folder_button_title="Select Folder",
    ):
        self._default_value = default_value

        self._on_value_changed_fn = on_value_changed_fn

        self._item_filter_fn = item_filter_fn

        self._file_picker_frame = None
        self._file_picker_btn = None

        string_field_frame = self._create_ui_widget(
            label,
            default_value,
            tooltip,
            use_folder_picker,
            read_only,
            multiline_okay,
            bookmark_label,
            bookmark_path,
            folder_dialog_title,
            folder_button_title,
        )

        super().__init__(string_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def string_field(self) -> ui.StringField:
        """
        Returns:
            omni.ui.StringField: UI StringField element
        """
        return self._string_field

    @property
    def file_picker_frame(self) -> ui.Frame:
        """
        Returns:
            omni.ui.Frame: UI Frame containing FilePicker
        """
        return self._file_picker_frame

    @property
    def file_picker_btn(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: Button to activate file picker
        """
        return self._file_picker_btn

    def get_value(self) -> str:
        """Return the current value of the StringField

        Returns:
            str: Current value of the StringField
        """
        return self.string_field.model.get_value_as_string()

    def set_value(self, val: str):
        """Set the value of the StringField

        Args:
            val (str): Value to fill StringField
        """
        self.string_field.model.set_value(val)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the StringField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the StringField is modified.
                Function should take a string as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def set_item_filter_fn(self, item_filter_fn: Callable):
        """Set the filter function that will be used with the file picker

        Args:
            item_filter_fn (Callable): Filter function that will be called to filter the files shown in the
                picker.  This function should take a string file_path as the argument. The return value 
                should be a bool, with True indicating the the file should be shown to the user in the file picker.
        """
        self._item_filter_fn = item_filter_fn

    def set_read_only(self, read_only: bool):
        """Set this StringField to be read only

        Args:
            read_only (bool): If True, StringField cannot be modified through the UI; it can still be 
                modified programmatically with set_value()
        """
        self.string_field.read_only = read_only

    def set_multiline_okay(self, multiline_okay: bool):
        """Set the StringFiled to allow the newline character

        Args:
            multiline_okay (bool): If True, allow newline character in strings.
        """
        self.string_field.multiline = multiline_okay

    def _item_filter_fn_wrapper(self, file):
        if self._item_filter_fn is not None:
            return self._item_filter_fn(file.path)

    def _on_value_changed_fn_wrapper(self, model):
        val = model.get_value_as_string()
        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(
        self,
        label="",
        default_val=" ",
        tooltip="",
        use_folder_picker=False,
        read_only=False,
        multiline_okay=True,
        bookmark_label=None,
        bookmark_path=None,
        folder_dialog_title="Select Output Folder",
        folder_button_title="Select Folder",
    ):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._string_field = ui.StringField(
                    name="StringField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    read_only=read_only,
                    multiline=multiline_okay,
                )
                self.string_field.model.set_value(default_val)

                if use_folder_picker:

                    def update_field(filename, path):
                        if filename == "":
                            val = path
                        elif filename[0] != "/" and path[-1] != "/":
                            val = path + "/" + filename
                        elif filename[0] == "/" and path[-1] == "/":
                            val = path + filename[1:]
                        else:
                            val = path + filename
                        self.string_field.model.set_value(val)

                    self.add_folder_picker_icon(
                        update_field,
                        self._item_filter_fn_wrapper,
                        bookmark_label,
                        bookmark_path,
                        dialog_title=folder_dialog_title,
                        button_title=folder_button_title,
                    )
                else:
                    add_line_rect_flourish(False)

                self.string_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame

    def add_folder_picker_icon(
        self,
        on_click_fn,
        item_filter_fn=None,
        bookmark_label=None,
        bookmark_path=None,
        dialog_title="Select Output Folder",
        button_title="Select Folder",
    ):
        def open_file_picker():
            def on_selected(filename, path):
                on_click_fn(filename, path)
                file_picker.hide()

            def on_canceled(a, b):
                file_picker.hide()

            file_picker = FilePickerDialog(
                dialog_title,
                allow_multi_selection=False,
                apply_button_label=button_title,
                click_apply_handler=lambda a, b: on_selected(a, b),
                click_cancel_handler=lambda a, b: on_canceled(a, b),
                item_filter_fn=item_filter_fn,
                enable_versioning_pane=True,
            )
            if bookmark_label and bookmark_path:
                file_picker.toggle_bookmark_from_path(bookmark_label, bookmark_path, True)

        self._file_picker_frame = ui.Frame(width=0, tooltip=button_title)
        with self.file_picker_frame:
            self._file_picker_btn = ui.Button(
                name="IconButton",
                width=24,
                height=24,
                clicked_fn=open_file_picker,
                style=get_style()["IconButton.Image::FolderPicker"],
                alignment=ui.Alignment.RIGHT_TOP,
            )


##########################################################################################
#                               UI Button Wrappers
##########################################################################################


class Button(UIWidgetWrapper):
    """Create a Button UI Element

    Args:
        label (str): Short descriptive text to the left of the Button
        text (str): Text on the Button
        tooltip (str, optional): Text to appear when the mouse hovers over the Button. Defaults to "".
        on_click_fn (Callable, optional): Callback function that will be called when the button is pressed.
            Function should take no arguments.  The return value will not be used.  Defaults to None.
    """

    def __init__(self, label: str, text: str, tooltip="", on_click_fn=None):
        self._on_click_fn = on_click_fn

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

    def set_on_click_fn(self, on_click_fn: Callable):
        """Set the callback function for when the Button is clicked.

        Args:
            on_click_fn (Callable): Callback function for when Button is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def _on_clicked_fn_wrapper(self):
        if self._on_click_fn is not None:
            self._on_click_fn()

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


class StateButton(UIWidgetWrapper):
    """
    Creates a State Button UI element.
    A StateButton is a button that changes between two states A and B when clicked.
    In state A, the StateButton has a_text written on it, and
    in state B, the StateButton has b_text written on it.

    Args:
        label (str): Short descriptive text to the left of the StateButton
        a_text (str): Text on the StateButton in one of its two states
        b_text (str): Text on the StateButton in the other of its two states
        tooltip (str, optional): Text that appears when the mouse hovers over the button. Defaults to "".
        on_a_click_fn (Callable, optional): A function that should be called when the button is clicked while in
            state A. Function should have 0 arguments.  The return value will not be used.  Defaults to None.
        on_b_click_fn (Callable, optional): A function that should be called when the button is clicked while in
            state B. Function should have 0 arguments.  The return value will not be used.  Defaults to None.
        physics_callback_fn (Callable, optional): A function that will be called on every physics step while the 
            button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
            The return value will not be used. Defaults to None.
	"""

    def __init__(
        self,
        label: str,
        a_text: str,
        b_text: str,
        tooltip="",
        on_a_click_fn: Callable = None,
        on_b_click_fn: Callable = None,
        physics_callback_fn: Callable = None,
    ):
        self.a_text = a_text.upper()
        self.b_text = b_text.upper()

        self._on_a_click_fn = on_a_click_fn
        self._on_b_click_fn = on_b_click_fn

        self._physics_callback_fn = physics_callback_fn
        self._physx_subscription = None
        self._physxIFace = _physx.acquire_physx_interface()

        state_btn_frame = self._creat_ui_widget(label, a_text, b_text, tooltip)

        super().__init__(state_btn_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def state_button(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: UI Button element
        """
        return self._state_button

    def set_physics_callback_fn(self, physics_callback_fn: Callable):
        """Set a physics callback function that will be called on every physics step while the StateButton is
        in state B.

        Args:
            physics_callback_fn (Callable): A function that will be called on every physics step while the 
                button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
                The return value will not be used.
        """
        self._physics_callback_fn = physics_callback_fn

    def set_on_a_click_fn(self, on_a_click_fn: Callable):
        """Set a function that is called when the button is clicked in state A.

        Args:
            on_a_click_fn (Callable): A function that is called when the button is clicked in state A.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_a_click_fn = on_a_click_fn

    def set_on_b_click_fn(self, on_b_click_fn: Callable):
        """Set a function that is called when the button is clicked in state B.

        Args:
            on_b_click_fn (Callable): A function that is called when the button is clicked in state B.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_b_click_fn = on_b_click_fn

    def reset(self):
        """Reset StateButton to state A.
        """
        self.state_button.text = self.a_text
        self._remove_physics_callback()

    def cleanup(self):
        """Remove physics callback created by the StateButton if exists.
        """
        self._remove_physics_callback()

    def _create_physics_callback(self):
        self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._physics_callback_fn)

    def _remove_physics_callback(self):
        self._physx_subscription = None

    def _on_clicked_fn_wrapper(self, value):
        # Button pressed while saying a_text
        if value:
            if self._on_a_click_fn is not None:
                self._on_a_click_fn()
            if self._physics_callback_fn is not None:
                self._create_physics_callback()

        # Button pressed while saying b_text
        else:
            if self._on_b_click_fn is not None:
                self._on_b_click_fn()
            if self._physics_callback_fn is not None:
                self._remove_physics_callback()

    def _creat_ui_widget(self, label: str, a_text: str, b_text: str, tooltip: str):
        def toggle():
            if self.state_button.text == a_text.upper():
                self.state_button.text = b_text.upper()
                self._on_clicked_fn_wrapper(True)
            else:
                self.state_button.text = a_text.upper()
                self._on_clicked_fn_wrapper(False)

        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._state_button = ui.Button(
                    a_text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=toggle,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                ui.Spacer(width=ui.Fraction(1))
                ui.Spacer(width=10)
                with ui.Frame(width=0):
                    with ui.VStack():
                        with ui.Placer(offset_x=0, offset_y=7):
                            ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
                ui.Spacer(width=5)
        return containing_frame


class CheckBox(UIWidgetWrapper):
    """Create a CheckBox UI Element

        Args:
            label (str): Short descriptive text to the left of the CheckBox
            default_value (bool, optional): If True, CheckBox will be checked. Defaults to False.
            tooltip (str, optional): Text to appear when the mouse hovers over the CheckBox.  Defaults to "".
            on_click_fn (_type_, optional): Callback function that will be called when the CheckBox is pressed.
                Function should take a single bool argument.  The return value will not be used.  Defaults to None.
    """

    def __init__(self, label: str, default_value: bool = False, tooltip="", on_click_fn=None):
        self._on_click_fn = on_click_fn

        checkbox_frame = self._create_ui_widget(label, default_value, tooltip)
        super().__init__(checkbox_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def checkbox(self) -> ui.CheckBox:
        """
        Returns:
            omni.ui.CheckBox: UI CheckBox element
        """
        return self._checkbox

    def get_value(self) -> bool:
        """
        Returns:
            bool: Check box is checked
        """
        return self.checkbox.model.get_value_as_bool()

    def set_on_click_fn(self, on_click_fn: Callable):
        """Set the function that will be called when the CheckBox is clicked.

        Args:
            on_click_fn (Callable): Callback function for when CheckBox is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def _on_click_fn_wrapper(self, model):
        if self._on_click_fn is not None:
            self._on_click_fn(model.get_value_as_bool())

    def _create_ui_widget(self, label: str, default_value: bool, tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                model = ui.SimpleBoolModel()
                self._checkbox = ui.CheckBox(model=model, tooltip=tooltip, checked=default_value)
                model.add_value_changed_fn(self._on_click_fn_wrapper)

                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                          UI Selection Widget Wrappers
##########################################################################################


class DropDown(UIWidgetWrapper):
    """
    Create a DropDown UI element.
    A DropDown menu can be populated by the user, with a callback function specified
    for when an item is selected.

    Args:
        label (str): Short descriptive text to the left of the DropDown
        tooltip (str, optional): Text to appear when the mouse hovers over the DropDown. Defaults to "".
        populate_fn (Callable, optional): A user-defined function that returns a list[str] of items
            that should populate the drop-down menu.  This Function should have 0 arguments. Defaults to None.
        on_selection_fn (Callable, optional): A user-defined callback function for when an element is selected
            from the DropDown.  The function should take in a string argument of the selection. 
            The return value will not be used.  Defaults to None.
        keep_old_selections (bool, optional): When the DropDown is repopulated with the user-defined populate_fn,
            the default behavior is to reset the selection in the DropDown to be at index 0.  If the user 
            sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
            the options, the new selection will match the old selection.  Defaults to False.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        populate_fn: Callable = None,
        on_selection_fn: Callable = None,
        keep_old_selections: bool = False,
    ):
        self._populate_fn = populate_fn
        self._on_selection_fn = on_selection_fn
        self._keep_old_selection = keep_old_selections
        self._items = []

        combobox_frame = self._create_ui_widget(label, tooltip)
        super().__init__(combobox_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def combobox(self) -> ui.ComboBox:
        """
        Returns:
            omni.ui.ComboBox: UI ComboBox element.
        """
        return self._combobox

    def repopulate(self):
        """A function that the user can call to make the DropDown menu repopulate.
        This will call the populate_fn set by the user.
        """
        if self._populate_fn is None:
            carb.log_warn("Unable to repopulate drop-down meny without a populate_fn being specified")
            return
        else:
            new_items = self._populate_fn()

            old_selection = self.get_selection()
            self.set_items(new_items)
            new_selection = self.get_selection()

            if self._on_selection_fn is not None and new_selection != old_selection:
                # Call the user on_selection_fn if the selection has changed as a result of repopulate()
                self._on_selection_fn(new_selection)

    def set_populate_fn(self, populate_fn: Callable, repopulate: bool = True):
        """Set the populate_fn for this DropDown

        Args:
            populate_fn (Callable): Function used to specify the options that fill the DropDown.
                Function should take no arguments and return a list[str].
            repopulate (bool, optional): If true, repopulate the DropDown using the new populate_fn. Defaults to True.
        """
        self._populate_fn = populate_fn
        if repopulate:
            self.repopulate()

    def get_items(self) -> List[str]:
        """Get the items in the DropDown

        Returns:
            List[str]: A list of the options in the DropDown
        """
        return self._items

    def get_selection_index(self) -> int:
        """Get index of selection in DropDown menu

        Returns:
            int: Index of selection in DropDown menu
        """
        return self.combobox.model.get_item_value_model().as_int

    def get_selection(self) -> str:
        """Get current selection in DropDown

        Returns:
            str: Current selection in DropDown
        """
        if len(self._items) == 0:
            return None
        return self._items[self.get_selection_index()]

    def set_items(self, items: List[str], select_index: int = None):
        """Set the items in the DropDown explicitly.

        Args:
            items (List[str]): New set of items in the DropDown
            select_index (int, optional): Index of item to select.  If left as None, behavior is determined by the
                keep_old_selections flag.  Defaults to None.
        """
        if self._keep_old_selection and select_index is None:
            selection = self.get_selection()
            if selection is not None and selection in items:
                select_index = items.index(selection)

        self._items = items
        self.combobox.model = DynamicComboBoxModel(items)

        if select_index is not None and select_index < len(items):
            self.combobox.model.get_item_value_model().set_value(select_index)

        self.combobox.model.add_item_changed_fn(self._item_changed_fn_wrapper)

    def set_selection(self, selection: str):
        """Set the selected item in the DropDown.
        If the specifified selection is not in the DropDown, nothing will happen.

        Args:
            selection (str): Item to select in the DropDown
        """
        if selection in self.get_items():
            select_index = self.get_items().index(selection)
            self.set_selection_by_index(select_index)
        else:
            carb.log_warn(f"Item {selection} is not present in DropDown, and cannot be set as the selected item.")

    def set_selection_by_index(self, select_index: int):
        """Set the selected item in the DropDown by index.
        If the provided index is out of bounds, nothing will happen.

        Args:
            select_index (int): Index of item to select from DropDown
        """
        if select_index < len(self.get_items()):
            self.combobox.model.get_item_value_model().set_value(select_index)
        else:
            carb.log_warn(
                f"Index {select_index} is out of bounds. The DropDown currently has {len(self.get_items())} items in it."
            )

    def set_on_selection_fn(self, on_selection_fn: Callable):
        """Set the function that gets called when a new item is selected from the DropDown

        Args:
            on_selection_fn (Callable): A function that is called when a new item is selected from the DropDown.
                he function should take in a string argument of the selection.  Its return value is not used.
        """
        self._on_selection_fn = on_selection_fn

    def set_keep_old_selection(self, val: bool):
        """ Set keep_old_selection flag to determine behavior when repopulating the DropDown

        Args:
            val (bool): When the DropDown is repopulated with the user-defined populate_fn,
                the default behavior is to reset the selection in the DropDown to be at index 0.  If the user 
                sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
                the options, the new selection will match the old selection, and the on_selection_fn() will not be called.
        """
        self._keep_old_selection = val

    def set_populate_fn_to_find_all_usd_objects_of_type(self, object_type: str, repopulate=True):
        """
        Set the populate_fn to find all objects of a specified type on the USD stage.  This is
        included as a convenience function to fulfill one common use-case for a DropDown menu.
        This overrides the populate_fn set by the user.   

        Args:
            object_type (str): A string name of the type of USD object matching the output of 
                omni.isaac.core.utils.prims.get_prim_object_type(prim_path)
            repopulate (bool, optional): Repopulate the DropDown immediately. Defaults to True.
        """
        self.set_populate_fn(lambda: self._find_all_usd_objects_of_type(object_type), repopulate=repopulate)

    def _item_changed_fn_wrapper(self, model, val):
        if self._on_selection_fn is not None:
            selected_item = self._items[model.get_item_value_model().as_int]
            self._on_selection_fn(selected_item)

    def _create_ui_widget(self, label, tooltip):
        items = []
        combobox_model = DynamicComboBoxModel(items)
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                self._combobox = ui.ComboBox(combobox_model)
                add_line_rect_flourish(False)

        return containing_frame

    def _find_all_usd_objects_of_type(self, obj_type: str):
        items = []
        stage = get_context().get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == obj_type:
                    items.append(path)

        return items


class ColorPicker(UIWidgetWrapper):
    """Create a ColorPicker UI element to allow user-selection of an RGBA color

        Args:
            label (str): Short descriptive text to the left of the ColorPicker
            default_value (List[float], optional): RGBA color values between 0 and 1. Defaults to [1.0, 1.0, 1.0, 1.0].
            tooltip (str, optional): Text to appear when the mouse hovers over the ColorPicker. Defaults to "".
            on_color_picked_fn (Callable, optional): Function that will be called if the user picks a new color.
                Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
                The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        default_value: List[float] = [1.0, 1.0, 1.0, 1.0],
        tooltip: str = "",
        on_color_picked_fn: Callable = None,
    ):
        self._on_color_picked_fn = on_color_picked_fn

        color_picker_frame = self._create_ui_widget(label, default_value, tooltip)
        super().__init__(color_picker_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def color_picker(self) -> ui.ColorWidget:
        """
        Returns:
            omni.ui.ColorWidget: UI ColorWidget element
        """
        return self._color_picker

    def get_color(self) -> List[float]:
        """Get the RGBA value of the selected color

        Returns:
            List[float]: RGBA color value with four values between 0 and 1
        """
        color = []
        for item in self.color_picker.model.get_item_children():
            val = self.color_picker.model.get_item_value_model(item).get_value_as_float()
            color.append(val)
        return color

    def set_color(self, color: List[float]):
        """Set the RGBA color value of the selected color

        Args:
            color (List[float]): RGBA color value with four values between 0 and 1
        """
        for i, item in enumerate(self.color_picker.model.get_item_children()):
            val = self.color_picker.model.get_item_value_model(item).set_value(color[i])
            color.append(val)

    def set_on_color_picked_fn(self, on_color_picked_fn: Callable):
        """Set the function that should be called if the user picks a new color

        Args:
            on_color_picked_fn (Callable): Function that will be called if the user picks a new color.
                Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
                The return value will not be used.
        """
        self._on_color_picked_fn = on_color_picked_fn

    def _on_color_picked_fn_wrapper(self, *worthless_args):
        if self._on_color_picked_fn is not None:
            self._on_color_picked_fn(self.get_color())

    def _create_ui_widget(self, label: str, default_value: List[float], tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._color_picker = ui.ColorWidget(*default_value, width=BUTTON_WIDTH)
                self.color_picker.model.add_end_edit_fn(self._on_color_picked_fn_wrapper)
                ui.Spacer(width=5)
                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                      UI Information Communication Wrappers
##########################################################################################


class TextBlock(UIWidgetWrapper):
    """Create a text block that is only modifiable through code. The user may not set 
    the value of the text in the UI.

    Args:
        label (str): Short description of the contents of the TextBlock
        text (str, optional): Text to put in the TextBlock. Defaults to "".
        tooltip (str, optional): Text to appear when the mouse hovers over the TextBlock. Defaults to "".
        num_lines (int, optional): Number of lines that should be visible in the TextBlock at one time. Defaults to 5.
        include_copy_button (bool, optional): Include a copy_to_clipboard button. Defaults to True.
    """

    def __init__(self, label: str, text: str = "", tooltip: str = "", num_lines=5, include_copy_button: bool = True):
        self._copy_btn = None

        text_block_frame = self._create_ui_widget(num_lines, label, text, tooltip, include_copy_button)
        super().__init__(text_block_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def scrolling_frame(self) -> ui.ScrollingFrame:
        """
        Returns:
            omni.ui.ScrollingFrame: Scrolling Frame that contains the TextBlock text
        """
        return self._scrolling_frame

    @property
    def copy_btn(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: Copy Button.  If the TextBlock was built without a copy button, this will return None.
        """
        return self._copy_btn

    @property
    def text_block(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI element that contains the text in the text block
        """
        return self._text_block

    def get_text(self) -> str:
        """
        Returns:
            str: Text in the text block
        """
        return self.text_block.text

    def set_text(self, text: str):
        """
        Args:
            text (str): Set the text in the text block.
        """
        self.text_block.text = text

    def set_num_lines(self, num_lines: int):
        """Set the number of lines that should be visible in the TextBlock at one time.

        Args:
            num_lines (int): Number of lines that should be visible in the TextBlock at one time.
        """
        self.scrolling_frame.height = LABEL_HEIGHT * num_lines

    def _create_ui_widget(self, num_lines, label, text, tooltip, include_copy_btn):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.VStack(style=get_style(), spacing=5):
                with ui.HStack():
                    self._label = ui.Label(
                        label, width=LABEL_WIDTH / 2, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip)
                    )
                    self._scrolling_frame = ScrollingFrame(num_lines=num_lines).frame
                    with self._scrolling_frame:
                        self._text_block = ui.Label(
                            text,
                            style_type_name_override="Label::label",
                            word_wrap=True,
                            alignment=ui.Alignment.LEFT_TOP,
                        )
                    if include_copy_btn:
                        with ui.Frame(width=0, tooltip="Copy To Clipboard"):
                            self._copy_btn = ui.Button(
                                name="IconButton",
                                width=20,
                                height=20,
                                clicked_fn=lambda: on_copy_to_clipboard(to_copy=self._text_block.text),
                                style=get_style()["IconButton.Image::CopyToClipboard"],
                                alignment=ui.Alignment.RIGHT_TOP,
                            )
        return containing_frame
