import omni.ui as ui
from omni.isaac.ui.widgets import DynamicComboBoxModel
from omni.kit.window.property.templates import LABEL_WIDTH, LABEL_HEIGHT
from omni.isaac.core.utils.prims import get_prim_object_type
from pxr import Usd

from cmath import inf
from typing import Callable, List

from omni.isaac.ui.ui_utils import (
    cb_builder,
    scrolling_frame_builder,
    xyz_builder,
    color_picker_builder,
    progress_bar_builder,
    plot_builder,
    xyz_plot_builder,
    add_line_rect_flourish,
    format_tt,
)

import omni.physx as _physx
import carb
from omni.usd import get_context

from omni.isaac.ui.element_wrappers import UIWidgetWrapper

import sys

from omni.isaac.ui.ui_utils import add_folder_picker_icon
import omni.ui as ui


class ColorPicker(UIWidgetWrapper):
    def __init__(self):
        pass


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

        self.checkbox = self._create_ui_widget(label, default_value, tooltip)
        super().__init__(self.checkbox)

    def set_on_click_fn(self, on_click_fn: Callable):
        """Set the function that will be called when the CheckBox is clicked.

        Args:
            on_click_fn (Callable): Callback function for when CheckBox is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = function

    def _on_click_fn_wrapper(self, model):
        if self._on_click_fn is not None:
            self._on_click_fn(model.get_value_as_bool())

    def _create_ui_widget(self, label: str, default_value: bool, tooltip: str):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
            model = ui.SimpleBoolModel()
            cb = ui.CheckBox(model=model, tooltip=tooltip, checked=default_value)
            model.add_value_changed_fn(self._on_click_fn_wrapper)

            add_line_rect_flourish()
        return cb


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

        self.int_field = self._create_ui_widget(label, tooltip, default_value)

        super().__init__(self.int_field)

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
        return self._upper_limit

    def get_lower_limit(self) -> int:
        """Get the lower limit on the IntField.

        Returns:
            int: Lower Limit on IntField
        """
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
        """Set function that is called when the value of the FloatField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the FloatField is modified.
                Function should take a float as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model):
        # Enforces upper and lower limits on value change
        model.set_max(self._upper_limit)
        model.set_min(self._lower_limit)
        val = model.get_value_as_int()

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
            int_field = ui.IntDrag(
                name="Field",
                height=LABEL_HEIGHT,
                alignment=ui.Alignment.LEFT_CENTER,
                min=sys.maxsize * -1,
                max=sys.maxsize,
            )
            int_field.model.set_value(default_value)
            add_line_rect_flourish(False)
        int_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return int_field


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

        self.float_field = self._create_ui_widget(label, tooltip, default_value, step, format)

        super().__init__(self.float_field)

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
        return self._upper_limit

    def get_lower_limit(self) -> float:
        """Get the lower limit on the FloatField

        Returns:
            float: Lower limit on FloatField
        """
        return self._lower_limit

    def set_value(self, val: float):
        """Set the value in the FloatField

        Args:
            val (float): Value to fill FloatField
        """
        self.float_field.model.set_value(val)

    def set_upper_limit(self, upper_limit: float):
        """Set upper limit of FloatField.
        If current value is higher than upper_limit, the current value will be clipped to upper_limit

        Args:
            upper_limit (float): Upper limit of FloatField
        """
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: float):
        """Set lower limit of FloatField.
        If current value is lower than lower_limit, the current value will be clipped to lower_limit

        Args:
            lower_limit (float): lower limit of FloatField
        """
        self._lower_limit = lower_limit
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
        model.set_max(self._upper_limit)
        model.set_min(self._lower_limit)
        val = model.get_value_as_float()
        if self._upper_limit is not None and self._upper_limit < val:
            val = self._upper_limit
            model.set_value(float(val + 1))
            return
        elif self._lower_limit is not None and self._lower_limit > val:
            val = self._lower_limit
            model.set_value(float(val - 1))
            return

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value, step, format):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
            float_field = ui.FloatDrag(
                name="FloatField",
                width=ui.Fraction(1),
                height=0,
                alignment=ui.Alignment.LEFT_CENTER,
                min=-inf,
                max=inf,
                step=step,
                format=format,
            )
            float_field.model.set_value(default_value)
            add_line_rect_flourish(False)

        float_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return float_field


class StringField(UIWidgetWrapper):
    """Create StringField UI Element.
    
    Starting at use_folder_picker, the arguments to the StringField all pertain to the folder_picker.
    If the folder_picker is not used, these arguments may all be ignored.
    

    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".
        default_val (str, optional): Text to initialize in Stringfield. Defaults to " ".
        read_only (bool, optional): Prevents editing. Defaults to False.
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

        self.string_field = self._create_ui_widget(
            label,
            default_value,
            tooltip,
            use_folder_picker,
            read_only,
            bookmark_label,
            bookmark_path,
            folder_dialog_title,
            folder_button_title,
        )

        super().__init__(self.string_field)

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
        bookmark_label=None,
        bookmark_path=None,
        folder_dialog_title="Select Output Folder",
        folder_button_title="Select Folder",
    ):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
            str_field = ui.StringField(
                name="StringField",
                width=ui.Fraction(1),
                height=0,
                alignment=ui.Alignment.LEFT_CENTER,
                read_only=read_only,
            )
            str_field.model.set_value(default_val)

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
                    str_field.model.set_value(val)

                add_folder_picker_icon(
                    update_field,
                    self._item_filter_fn_wrapper,
                    bookmark_label,
                    bookmark_path,
                    dialog_title=folder_dialog_title,
                    button_title=folder_button_title,
                )
            else:
                add_line_rect_flourish(False)

            str_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
            return str_field
