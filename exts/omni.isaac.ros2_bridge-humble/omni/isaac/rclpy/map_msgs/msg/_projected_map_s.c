// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from map_msgs:msg/ProjectedMap.idl
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
#include "map_msgs/msg/detail/projected_map__struct.h"
#include "map_msgs/msg/detail/projected_map__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool nav_msgs__msg__occupancy_grid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * nav_msgs__msg__occupancy_grid__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool map_msgs__msg__projected_map__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("map_msgs.msg._projected_map.ProjectedMap", full_classname_dest, 40) == 0);
  }
  map_msgs__msg__ProjectedMap * ros_message = _ros_message;
  {  // map
    PyObject * field = PyObject_GetAttrString(_pymsg, "map");
    if (!field) {
      return false;
    }
    if (!nav_msgs__msg__occupancy_grid__convert_from_py(field, &ros_message->map)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // min_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * map_msgs__msg__projected_map__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ProjectedMap */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("map_msgs.msg._projected_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ProjectedMap");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  map_msgs__msg__ProjectedMap * ros_message = (map_msgs__msg__ProjectedMap *)raw_ros_message;
  {  // map
    PyObject * field = NULL;
    field = nav_msgs__msg__occupancy_grid__convert_to_py(&ros_message->map);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "map", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
