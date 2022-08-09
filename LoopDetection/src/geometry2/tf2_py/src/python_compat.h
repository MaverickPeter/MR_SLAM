#ifndef TF2_PY_PYTHON_COMPAT_H
#define TF2_PY_PYTHON_COMPAT_H

#include <Python.h>

#include <string>

inline PyObject *stringToPython(const std::string &input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromStringAndSize(input.c_str(), input.size());
#else
  return PyString_FromStringAndSize(input.c_str(), input.size());
#endif
}

inline PyObject *stringToPython(const char *input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromString(input);
#else
  return PyString_FromString(input);
#endif
}

inline std::string stringFromPython(PyObject * input)
{
  Py_ssize_t size;
#if PY_MAJOR_VERSION >= 3
  const char * data;
  data = PyUnicode_AsUTF8AndSize(input, &size);
#else
  char * data;
  PyString_AsStringAndSize(input, &data, &size);
#endif
  return std::string(data, size);
}

inline PyObject *pythonImport(const std::string & name)
{
  PyObject *py_name = stringToPython(name);
  PyObject *module  = PyImport_Import(py_name);
  Py_XDECREF(py_name);
  return module;
}

inline PyObject *pythonBorrowAttrString(PyObject* o, const char *name)
{
    PyObject *r = PyObject_GetAttrString(o, name);
    Py_XDECREF(r);
    return r;
}

#endif
