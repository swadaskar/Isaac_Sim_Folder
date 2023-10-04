// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rmw_dds_common:msg/NodeEntitiesInfo.idl
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
#include "rmw_dds_common/msg/detail/node_entities_info__struct.h"
#include "rmw_dds_common/msg/detail/node_entities_info__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "rmw_dds_common/msg/detail/gid__functions.h"
// end nested array functions include
bool rmw_dds_common__msg__gid__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rmw_dds_common__msg__gid__convert_to_py(void * raw_ros_message);
bool rmw_dds_common__msg__gid__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rmw_dds_common__msg__gid__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rmw_dds_common__msg__node_entities_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("rmw_dds_common.msg._node_entities_info.NodeEntitiesInfo", full_classname_dest, 55) == 0);
  }
  rmw_dds_common__msg__NodeEntitiesInfo * ros_message = _ros_message;
  {  // node_namespace
    PyObject * field = PyObject_GetAttrString(_pymsg, "node_namespace");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->node_namespace, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // node_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "node_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->node_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // reader_gid_seq
    PyObject * field = PyObject_GetAttrString(_pymsg, "reader_gid_seq");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'reader_gid_seq'");
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
    if (!rmw_dds_common__msg__Gid__Sequence__init(&(ros_message->reader_gid_seq), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rmw_dds_common__msg__Gid__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rmw_dds_common__msg__Gid * dest = ros_message->reader_gid_seq.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rmw_dds_common__msg__gid__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // writer_gid_seq
    PyObject * field = PyObject_GetAttrString(_pymsg, "writer_gid_seq");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'writer_gid_seq'");
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
    if (!rmw_dds_common__msg__Gid__Sequence__init(&(ros_message->writer_gid_seq), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rmw_dds_common__msg__Gid__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rmw_dds_common__msg__Gid * dest = ros_message->writer_gid_seq.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rmw_dds_common__msg__gid__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * rmw_dds_common__msg__node_entities_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of NodeEntitiesInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rmw_dds_common.msg._node_entities_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "NodeEntitiesInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rmw_dds_common__msg__NodeEntitiesInfo * ros_message = (rmw_dds_common__msg__NodeEntitiesInfo *)raw_ros_message;
  {  // node_namespace
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->node_namespace.data,
      strlen(ros_message->node_namespace.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "node_namespace", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // node_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->node_name.data,
      strlen(ros_message->node_name.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "node_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reader_gid_seq
    PyObject * field = NULL;
    size_t size = ros_message->reader_gid_seq.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rmw_dds_common__msg__Gid * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->reader_gid_seq.data[i]);
      PyObject * pyitem = rmw_dds_common__msg__gid__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "reader_gid_seq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // writer_gid_seq
    PyObject * field = NULL;
    size_t size = ros_message->writer_gid_seq.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rmw_dds_common__msg__Gid * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->writer_gid_seq.data[i]);
      PyObject * pyitem = rmw_dds_common__msg__gid__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "writer_gid_seq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
