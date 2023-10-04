# coding: utf-8

"""
    Onshape REST API

    The Onshape REST API consumed by all clients.  # noqa: E501

    The version of the OpenAPI document: 1.113
    Contact: api-support@onshape.zendesk.com
    Generated by: https://openapi-generator.tech
"""


from __future__ import absolute_import
import re  # noqa: F401
import sys  # noqa: F401

import six  # noqa: F401
import nulltype  # noqa: F401

from omni.isaac.onshape.onshape_client.oas.model_utils import (  # noqa: F401
    ModelComposed,
    ModelNormal,
    ModelSimple,
    date,
    datetime,
    file_type,
    int,
    none_type,
    str,
    validate_get_composed_info,
)

try:
    from omni.isaac.onshape.onshape_client.oas.models import bt_circle_description1145
except ImportError:
    bt_circle_description1145 = sys.modules["omni.isaac.onshape.onshape_client.oas.models.bt_circle_description1145"]
try:
    from omni.isaac.onshape.onshape_client.oas.models import bt_ellipse_description866
except ImportError:
    bt_ellipse_description866 = sys.modules["omni.isaac.onshape.onshape_client.oas.models.bt_ellipse_description866"]
try:
    from omni.isaac.onshape.onshape_client.oas.models import bt_line_description1559
except ImportError:
    bt_line_description1559 = sys.modules["omni.isaac.onshape.onshape_client.oas.models.bt_line_description1559"]
try:
    from omni.isaac.onshape.onshape_client.oas.models import bt_spline_description2118
except ImportError:
    bt_spline_description2118 = sys.modules["omni.isaac.onshape.onshape_client.oas.models.bt_spline_description2118"]


class BTCurveDescription1583(ModelNormal):
    """NOTE: This class is auto generated by OpenAPI Generator.
    Ref: https://openapi-generator.tech

    Do not edit the class manually.

    Attributes:
      allowed_values (dict): The key is the tuple path to the attribute
          and the for var_name this is (var_name,). The value is a dict
          with a capitalized key describing the allowed value and an allowed
          value. These dicts store the allowed enum values.
      attribute_map (dict): The key is attribute name
          and the value is json key in definition.
      discriminator_value_class_map (dict): A dict to go from the discriminator
          variable value to the discriminator class name.
      validations (dict): The key is the tuple path to the attribute
          and the for var_name this is (var_name,). The value is a dict
          that stores validations for max_length, min_length, max_items,
          min_items, exclusive_maximum, inclusive_maximum, exclusive_minimum,
          inclusive_minimum, and regex.
      additional_properties_type (tuple): A tuple of classes accepted
          as additional properties values.
    """

    allowed_values = {
        ("type",): {
            "OTHER": "OTHER",
            "LINE": "LINE",
            "CIRCLE": "CIRCLE",
            "ELLIPSE": "ELLIPSE",
            "BCURVE": "BCURVE",
            "ICURVE": "ICURVE",
            "UNKNOWN": "UNKNOWN",
        }
    }

    validations = {}

    additional_properties_type = None

    @staticmethod
    def openapi_types():
        """
        This must be a class method so a model may have properties that are
        of type self, this ensures that we don't create a cyclic import

        Returns
            openapi_types (dict): The key is attribute name
                and the value is attribute type.
        """
        return {"bt_type": (str,), "type": (str,)}  # noqa: E501  # noqa: E501

    @staticmethod
    def discriminator():
        return {
            "bt_type": {
                "BTLineDescription-1559": bt_line_description1559.BTLineDescription1559,
                "BTCircleDescription-1145": bt_circle_description1145.BTCircleDescription1145,
                "BTEllipseDescription-866": bt_ellipse_description866.BTEllipseDescription866,
                "BTSplineDescription-2118": bt_spline_description2118.BTSplineDescription2118,
            }
        }

    attribute_map = {"bt_type": "btType", "type": "type"}  # noqa: E501  # noqa: E501

    @staticmethod
    def _composed_schemas():
        return None

    required_properties = set(["_data_store", "_check_type", "_from_server", "_path_to_item", "_configuration"])

    def __init__(
        self, _check_type=True, _from_server=False, _path_to_item=(), _configuration=None, **kwargs
    ):  # noqa: E501
        """bt_curve_description1583.BTCurveDescription1583 - a model defined in OpenAPI

        Keyword Args:
            _check_type (bool): if True, values for parameters in openapi_types
                                will be type checked and a TypeError will be
                                raised if the wrong type is input.
                                Defaults to True
            _path_to_item (tuple/list): This is a list of keys or values to
                                drill down to the model in received_data
                                when deserializing a response
            _from_server (bool): True if the data is from the server
                                False if the data is from the client (default)
            _configuration (Configuration): the instance to use when
                                deserializing a file_type parameter.
                                If passed, type conversion is attempted
                                If omitted no type conversion is done.
            bt_type (str): [optional]  # noqa: E501
            type (str): [optional]  # noqa: E501
        """

        self._data_store = {}
        self._check_type = _check_type
        self._from_server = _from_server
        self._path_to_item = _path_to_item
        self._configuration = _configuration

        for var_name, var_value in six.iteritems(kwargs):
            if (
                var_name not in self.attribute_map
                and self._configuration is not None
                and self._configuration.discard_unknown_keys
                and self.additional_properties_type is None
            ):
                # discard variable.
                continue
            setattr(self, var_name, var_value)

    @classmethod
    def get_discriminator_class(cls, from_server, data):
        """Returns the child class specified by the discriminator"""
        discriminator = cls.discriminator()
        discr_propertyname_py = list(discriminator.keys())[0]
        discr_propertyname_js = cls.attribute_map[discr_propertyname_py]
        if from_server:
            class_name = data[discr_propertyname_js]
        else:
            class_name = data[discr_propertyname_py]
        class_name_to_discr_class = discriminator[discr_propertyname_py]
        return class_name_to_discr_class.get(class_name)
