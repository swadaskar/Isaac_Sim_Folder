// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vision_msgs:msg/BoundingBox2D.idl
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
#include "vision_msgs/msg/detail/bounding_box2_d__struct.h"
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"

bool vision_msgs__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * vision_msgs__msg__pose2_d__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vision_msgs__msg__bounding_box2_d__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("vision_msgs.msg._bounding_box2_d.BoundingBox2D", full_classname_dest, 46) == 0);
  }
  vision_msgs__msg__BoundingBox2D * ros_message = _ros_message;
  {  // center
    PyObject * field = PyObject_GetAttrString(_pymsg, "center");
    if (!field) {
      return false;
    }
    if (!vision_msgs__msg__pose2_d__convert_from_py(field, &ros_message->center)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // size_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "size_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->size_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // size_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "size_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->size_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vision_msgs__msg__bounding_box2_d__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BoundingBox2D */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vision_msgs.msg._bounding_box2_d");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BoundingBox2D");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vision_msgs__msg__BoundingBox2D * ros_message = (vision_msgs__msg__BoundingBox2D *)raw_ros_message;
  {  // center
    PyObject * field = NULL;
    field = vision_msgs__msg__pose2_d__convert_to_py(&ros_message->center);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // size_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->size_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "size_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // size_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->size_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "size_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
