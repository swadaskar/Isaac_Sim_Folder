// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from map_msgs:srv/SetMapProjections.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "map_msgs/srv/detail/set_map_projections__struct.h"
#include "map_msgs/srv/detail/set_map_projections__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool map_msgs__srv__set_map_projections__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("map_msgs.srv._set_map_projections.SetMapProjections_Request", full_classname_dest, 59) == 0);
  }
  map_msgs__srv__SetMapProjections_Request * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * map_msgs__srv__set_map_projections__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetMapProjections_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("map_msgs.srv._set_map_projections");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetMapProjections_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "map_msgs/srv/detail/set_map_projections__struct.h"
// already included above
// #include "map_msgs/srv/detail/set_map_projections__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "map_msgs/msg/detail/projected_map_info__functions.h"
// end nested array functions include
bool map_msgs__msg__projected_map_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * map_msgs__msg__projected_map_info__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool map_msgs__srv__set_map_projections__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[61];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("map_msgs.srv._set_map_projections.SetMapProjections_Response", full_classname_dest, 60) == 0);
  }
  map_msgs__srv__SetMapProjections_Response * ros_message = _ros_message;
  {  // projected_maps_info
    PyObject * field = PyObject_GetAttrString(_pymsg, "projected_maps_info");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'projected_maps_info'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!map_msgs__msg__ProjectedMapInfo__Sequence__init(&(ros_message->projected_maps_info), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create map_msgs__msg__ProjectedMapInfo__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    map_msgs__msg__ProjectedMapInfo * dest = ros_message->projected_maps_info.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!map_msgs__msg__projected_map_info__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * map_msgs__srv__set_map_projections__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetMapProjections_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("map_msgs.srv._set_map_projections");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetMapProjections_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  map_msgs__srv__SetMapProjections_Response * ros_message = (map_msgs__srv__SetMapProjections_Response *)raw_ros_message;
  {  // projected_maps_info
    PyObject * field = NULL;
    size_t size = ros_message->projected_maps_info.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    map_msgs__msg__ProjectedMapInfo * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->projected_maps_info.data[i]);
      PyObject * pyitem = map_msgs__msg__projected_map_info__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "projected_maps_info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
