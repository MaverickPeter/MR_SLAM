#include <Python.h>

#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

#include "python_compat.h"

// Run x (a tf method, catching TF's exceptions and reraising them as Python exceptions)
//
#define WRAP(x) \
  do { \
  try \
  { \
    x; \
  }  \
  catch (const tf2::ConnectivityException &e) \
  { \
    PyErr_SetString(tf2_connectivityexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::LookupException &e) \
  { \
    PyErr_SetString(tf2_lookupexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::ExtrapolationException &e) \
  { \
    PyErr_SetString(tf2_extrapolationexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::InvalidArgumentException &e) \
  { \
    PyErr_SetString(tf2_invalidargumentexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::TimeoutException &e) \
  { \
    PyErr_SetString(tf2_timeoutexception, e.what()); \
    return NULL; \
  } \
  catch (const tf2::TransformException &e) \
  { \
    PyErr_SetString(tf2_exception, e.what()); \
    return NULL; \
  } \
  } while (0)

static PyObject *pModulerospy = NULL;
static PyObject *pModulegeometrymsgs = NULL;
static PyObject *tf2_exception = NULL;
static PyObject *tf2_connectivityexception = NULL, *tf2_lookupexception = NULL, *tf2_extrapolationexception = NULL,
                *tf2_invalidargumentexception = NULL, *tf2_timeoutexception = NULL;

struct buffer_core_t {
  PyObject_HEAD
  tf2::BufferCore *bc;
};


static PyTypeObject buffer_core_Type = {
#if PY_MAJOR_VERSION < 3
  PyObject_HEAD_INIT(NULL)
  0,                               /*size*/
# else
  PyVarObject_HEAD_INIT(NULL, 0)
#endif
  "_tf2.BufferCore",               /*name*/
  sizeof(buffer_core_t),           /*basicsize*/
};

static PyObject *transform_converter(const geometry_msgs::TransformStamped* transform)
{
  PyObject *pclass, *pargs, *pinst = NULL;
  pclass = PyObject_GetAttrString(pModulegeometrymsgs, "TransformStamped");
  if(pclass == NULL)
  {
    printf("Can't get geometry_msgs.msg.TransformedStamped\n");
    return NULL;
  }

  pargs = Py_BuildValue("()");
  if(pargs == NULL)
  {
    printf("Can't build argument list\n");
    return NULL;
  }

  pinst = PyEval_CallObject(pclass, pargs);
  Py_DECREF(pclass);
  Py_DECREF(pargs);
  if(pinst == NULL)
  {
    printf("Can't create class\n");
    return NULL;
  }

  //we need to convert the time to python
  PyObject *rospy_time = PyObject_GetAttrString(pModulerospy, "Time");
  PyObject *args = Py_BuildValue("ii", transform->header.stamp.sec, transform->header.stamp.nsec);
  PyObject *time_obj = PyObject_CallObject(rospy_time, args);
  Py_DECREF(args);
  Py_DECREF(rospy_time);

  PyObject* pheader = PyObject_GetAttrString(pinst, "header");
  PyObject_SetAttrString(pheader, "stamp", time_obj);
  Py_DECREF(time_obj);

  PyObject *frame_id = stringToPython(transform->header.frame_id);
  PyObject_SetAttrString(pheader, "frame_id", frame_id);
  Py_DECREF(frame_id);
  Py_DECREF(pheader);

  PyObject *ptransform = PyObject_GetAttrString(pinst, "transform");
  PyObject *ptranslation = PyObject_GetAttrString(ptransform, "translation");
  PyObject *protation = PyObject_GetAttrString(ptransform, "rotation");
  Py_DECREF(ptransform);

  PyObject *child_frame_id = stringToPython(transform->child_frame_id);
  PyObject_SetAttrString(pinst, "child_frame_id", child_frame_id);
  Py_DECREF(child_frame_id);

  PyObject *trans_x = PyFloat_FromDouble(transform->transform.translation.x);
  PyObject *trans_y = PyFloat_FromDouble(transform->transform.translation.y);
  PyObject *trans_z = PyFloat_FromDouble(transform->transform.translation.z);
  PyObject_SetAttrString(ptranslation, "x", trans_x);
  PyObject_SetAttrString(ptranslation, "y", trans_y);
  PyObject_SetAttrString(ptranslation, "z", trans_z);
  Py_DECREF(trans_x);
  Py_DECREF(trans_y);
  Py_DECREF(trans_z);
  Py_DECREF(ptranslation);

  PyObject *rot_x = PyFloat_FromDouble(transform->transform.rotation.x);
  PyObject *rot_y = PyFloat_FromDouble(transform->transform.rotation.y);
  PyObject *rot_z = PyFloat_FromDouble(transform->transform.rotation.z);
  PyObject *rot_w = PyFloat_FromDouble(transform->transform.rotation.w);
  PyObject_SetAttrString(protation, "x", rot_x);
  PyObject_SetAttrString(protation, "y", rot_y);
  PyObject_SetAttrString(protation, "z", rot_z);
  PyObject_SetAttrString(protation, "w", rot_w);
  Py_DECREF(rot_x);
  Py_DECREF(rot_y);
  Py_DECREF(rot_z);
  Py_DECREF(rot_w);
  Py_DECREF(protation);

  return pinst;
}

static int rostime_converter(PyObject *obj, ros::Time *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_sec", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_sec method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int rosduration_converter(PyObject *obj, ros::Duration *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_sec", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_sec method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int BufferCore_init(PyObject *self, PyObject *args, PyObject *kw)
{
  ros::Duration cache_time;

  cache_time.fromSec(tf2::BufferCore::DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|O&", rosduration_converter, &cache_time))
    return -1;

  ((buffer_core_t*)self)->bc = new tf2::BufferCore(cache_time);

  return 0;
}

/* This may need to be implemented later if we decide to have it in the core
static PyObject *getTFPrefix(PyObject *self, PyObject *args)
{
  if (!PyArg_ParseTuple(args, ""))
    return NULL;
  tf::Transformer *t = ((transformer_t*)self)->t;
  return stringToPython(t->getTFPrefix());
}
*/

static PyObject *allFramesAsYAML(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  return stringToPython(bc->allFramesAsYAML());
}

static PyObject *allFramesAsString(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  return stringToPython(bc->allFramesAsString());
}

static PyObject *canTransformCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, source_frame, time, &error_msg);
  //return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject *canTransformFullCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, &error_msg);
  //return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject *asListOfStrings(std::vector< std::string > los)
{
  PyObject *r = PyList_New(los.size());
  size_t i;
  for (i = 0; i < los.size(); i++) {
    PyList_SetItem(r, i, stringToPython(los[i]));
  }
  return r;
}

static PyObject *_chain(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  std::vector< std::string > output;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;

  WRAP(bc->_chainAsVector(target_frame, target_time, source_frame, source_time, fixed_frame, output));
  return asListOfStrings(output);
}

static PyObject *getLatestCommonTime(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame;
  tf2::CompactFrameID target_id, source_id;
  ros::Time time;
  std::string error_string;

  if (!PyArg_ParseTuple(args, "ss", &target_frame, &source_frame))
    return NULL;
  WRAP(target_id = bc->_validateFrameId("get_latest_common_time", target_frame));
  WRAP(source_id = bc->_validateFrameId("get_latest_common_time", source_frame));
  int r = bc->_getLatestCommonTime(target_id, source_id, time, &error_string);
  if (r == 0) {
    PyObject *rospy_time = PyObject_GetAttrString(pModulerospy, "Time");
    PyObject *args = Py_BuildValue("ii", time.sec, time.nsec);
    PyObject *ob = PyObject_CallObject(rospy_time, args);
    Py_DECREF(args);
    Py_DECREF(rospy_time);
    return ob;
  } else {
    PyErr_SetString(tf2_exception, error_string.c_str());
    return NULL;
  }
}

static PyObject *lookupTransformCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame;
  ros::Time time;
  static const char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", (char**)keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  geometry_msgs::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, source_frame, time));
  geometry_msgs::Vector3 origin = transform.transform.translation;
  geometry_msgs::Quaternion rotation = transform.transform.rotation;
  //TODO: Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
  //return Py_BuildValue("(ddd)(dddd)",
  //    origin.x, origin.y, origin.z,
  //    rotation.x, rotation.y, rotation.z, rotation.w);
}

static PyObject *lookupTransformFullCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static const char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", (char**)keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  geometry_msgs::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  geometry_msgs::Vector3 origin = transform.transform.translation;
  geometry_msgs::Quaternion rotation = transform.transform.rotation;
  //TODO: Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
}
/*
static PyObject *lookupTwistCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame;
  ros::Time time;
  ros::Duration averaging_interval;
  static const char *keywords[] = { "tracking_frame", "observation_frame", "time", "averaging_interval", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&O&", (char**)keywords, &tracking_frame, &observation_frame, rostime_converter, &time, rosduration_converter, &averaging_interval))
    return NULL;
  geometry_msgs::Twist twist;
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}

static PyObject *lookupTwistFullCore(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame, *reference_frame, *reference_point_frame;
  ros::Time time;
  ros::Duration averaging_interval;
  double px, py, pz;

  if (!PyArg_ParseTuple(args, "sss(ddd)sO&O&",
                        &tracking_frame,
                        &observation_frame,
                        &reference_frame,
                        &px, &py, &pz,
                        &reference_point_frame,
                        rostime_converter, &time,
                        rosduration_converter, &averaging_interval))
    return NULL;
  geometry_msgs::Twist twist;
  tf::Point pt(px, py, pz);
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, reference_frame, pt, reference_point_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}
*/
static inline int checkTranslationType(PyObject* o)
{
  PyTypeObject *translation_type = (PyTypeObject*) PyObject_GetAttrString(pModulegeometrymsgs, "Vector3");
  int type_check = PyObject_TypeCheck(o, translation_type);
  int attr_check = PyObject_HasAttrString(o, "x") &&
                   PyObject_HasAttrString(o, "y") &&
                   PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "translation should be of type Vector3", 1);
  }
  return attr_check;
}

static inline int checkRotationType(PyObject* o)
{
  PyTypeObject *rotation_type = (PyTypeObject*) PyObject_GetAttrString(pModulegeometrymsgs, "Quaternion");
  int type_check = PyObject_TypeCheck(o, rotation_type);
  int attr_check = PyObject_HasAttrString(o, "w") &&
                   PyObject_HasAttrString(o, "x") &&
                   PyObject_HasAttrString(o, "y") &&
                   PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "rotation should be of type Quaternion", 1);
  }
  return attr_check;
}

static PyObject *setTransform(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  PyObject *py_transform;
  char *authority;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority))
    return NULL;

  geometry_msgs::TransformStamped transform;
  PyObject *header = pythonBorrowAttrString(py_transform, "header");
  transform.child_frame_id = stringFromPython(pythonBorrowAttrString(py_transform, "child_frame_id"));
  transform.header.frame_id = stringFromPython(pythonBorrowAttrString(header, "frame_id"));
  if (rostime_converter(pythonBorrowAttrString(header, "stamp"), &transform.header.stamp) != 1)
    return NULL;

  PyObject *mtransform = pythonBorrowAttrString(py_transform, "transform");

  PyObject *translation = pythonBorrowAttrString(mtransform, "translation");
  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must have members x, y, z");
    return NULL;
  }

  transform.transform.translation.x = PyFloat_AsDouble(pythonBorrowAttrString(translation, "x"));
  transform.transform.translation.y = PyFloat_AsDouble(pythonBorrowAttrString(translation, "y"));
  transform.transform.translation.z = PyFloat_AsDouble(pythonBorrowAttrString(translation, "z"));

  PyObject *rotation = pythonBorrowAttrString(mtransform, "rotation");
  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must have members w, x, y, z");
    return NULL;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "x"));
  transform.transform.rotation.y = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "y"));
  transform.transform.rotation.z = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "z"));
  transform.transform.rotation.w = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "w"));

  bc->setTransform(transform, authority);
  Py_RETURN_NONE;
}

static PyObject *setTransformStatic(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  PyObject *py_transform;
  char *authority;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority))
    return NULL;

  geometry_msgs::TransformStamped transform;
  PyObject *header = pythonBorrowAttrString(py_transform, "header");
  transform.child_frame_id = stringFromPython(pythonBorrowAttrString(py_transform, "child_frame_id"));
  transform.header.frame_id = stringFromPython(pythonBorrowAttrString(header, "frame_id"));
  if (rostime_converter(pythonBorrowAttrString(header, "stamp"), &transform.header.stamp) != 1)
    return NULL;

  PyObject *mtransform = pythonBorrowAttrString(py_transform, "transform");
  PyObject *translation = pythonBorrowAttrString(mtransform, "translation");
  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must be of type Vector3");
    return NULL;
  }

  transform.transform.translation.x = PyFloat_AsDouble(pythonBorrowAttrString(translation, "x"));
  transform.transform.translation.y = PyFloat_AsDouble(pythonBorrowAttrString(translation, "y"));
  transform.transform.translation.z = PyFloat_AsDouble(pythonBorrowAttrString(translation, "z"));

  PyObject *rotation = pythonBorrowAttrString(mtransform, "rotation");
  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must be of type Quaternion");
    return NULL;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "x"));
  transform.transform.rotation.y = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "y"));
  transform.transform.rotation.z = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "z"));
  transform.transform.rotation.w = PyFloat_AsDouble(pythonBorrowAttrString(rotation, "w"));

  // only difference to above is is_static == True
  bc->setTransform(transform, authority, true);
  Py_RETURN_NONE;
}

static PyObject *clear(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  bc->clear();
  Py_RETURN_NONE;
}

static PyObject *_frameExists(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *frame_id_str;
  if (!PyArg_ParseTuple(args, "s", &frame_id_str))
    return NULL;
  return PyBool_FromLong(bc->_frameExists(frame_id_str));
}

static PyObject *_getFrameStrings(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  std::vector< std::string > ids;
  bc->_getFrameStrings(ids);
  return asListOfStrings(ids);
}

static PyObject *_allFramesAsDot(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  static const char *keywords[] = { "time", NULL };
  ros::Time time;
  if (!PyArg_ParseTupleAndKeywords(args, kw, "|O&", (char**)keywords, rostime_converter, &time))
    return NULL;
  return stringToPython(bc->_allFramesAsDot(time.toSec()));
}


static struct PyMethodDef buffer_core_methods[] =
{
  {"all_frames_as_yaml", allFramesAsYAML, METH_VARARGS},
  {"all_frames_as_string", allFramesAsString, METH_VARARGS},
  {"set_transform", setTransform, METH_VARARGS},
  {"set_transform_static", setTransformStatic, METH_VARARGS},
  {"can_transform_core", (PyCFunction)canTransformCore, METH_VARARGS | METH_KEYWORDS},
  {"can_transform_full_core", (PyCFunction)canTransformFullCore, METH_VARARGS | METH_KEYWORDS},
  {"_chain", (PyCFunction)_chain, METH_VARARGS | METH_KEYWORDS},
  {"clear", (PyCFunction)clear, METH_VARARGS | METH_KEYWORDS},
  {"_frameExists", (PyCFunction)_frameExists, METH_VARARGS},
  {"_getFrameStrings", (PyCFunction)_getFrameStrings, METH_VARARGS},
  {"_allFramesAsDot", (PyCFunction)_allFramesAsDot, METH_VARARGS | METH_KEYWORDS},
  {"get_latest_common_time", (PyCFunction)getLatestCommonTime, METH_VARARGS},
  {"lookup_transform_core", (PyCFunction)lookupTransformCore, METH_VARARGS | METH_KEYWORDS},
  {"lookup_transform_full_core", (PyCFunction)lookupTransformFullCore, METH_VARARGS | METH_KEYWORDS},
  //{"lookupTwistCore", (PyCFunction)lookupTwistCore, METH_VARARGS | METH_KEYWORDS},
  //{"lookupTwistFullCore", lookupTwistFullCore, METH_VARARGS},
  //{"getTFPrefix", (PyCFunction)getTFPrefix, METH_VARARGS},
  {NULL,          NULL}
};

static PyMethodDef module_methods[] = {
  // {"Transformer", mkTransformer, METH_VARARGS},
  {0, 0, 0},
};

bool staticInit() {
#if PYTHON_API_VERSION >= 1007
  tf2_exception = PyErr_NewException((char*)"tf2.TransformException", NULL, NULL);
  tf2_connectivityexception = PyErr_NewException((char*)"tf2.ConnectivityException", tf2_exception, NULL);
  tf2_lookupexception = PyErr_NewException((char*)"tf2.LookupException", tf2_exception, NULL);
  tf2_extrapolationexception = PyErr_NewException((char*)"tf2.ExtrapolationException", tf2_exception, NULL);
  tf2_invalidargumentexception = PyErr_NewException((char*)"tf2.InvalidArgumentException", tf2_exception, NULL);
  tf2_timeoutexception = PyErr_NewException((char*)"tf2.TimeoutException", tf2_exception, NULL);
#else
  tf2_exception = stringToPython("tf2.error");
  tf2_connectivityexception = stringToPython("tf2.ConnectivityException");
  tf2_lookupexception = stringToPython("tf2.LookupException");
  tf2_extrapolationexception = stringToPython("tf2.ExtrapolationException");
  tf2_invalidargumentexception = stringToPython("tf2.InvalidArgumentException");
  tf2_timeoutexception = stringToPython("tf2.TimeoutException");
#endif

  pModulerospy        = pythonImport("rospy");
  pModulegeometrymsgs = pythonImport("geometry_msgs.msg");

  if(pModulegeometrymsgs == NULL)
  {
    printf("Cannot load geometry_msgs module");
    return false;
  }

  buffer_core_Type.tp_alloc = PyType_GenericAlloc;
  buffer_core_Type.tp_new = PyType_GenericNew;
  buffer_core_Type.tp_init = BufferCore_init;
  buffer_core_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  buffer_core_Type.tp_methods = buffer_core_methods;
  if (PyType_Ready(&buffer_core_Type) != 0)
    return false;
  return true;
}

PyObject *moduleInit(PyObject *m) {
  PyModule_AddObject(m, "BufferCore", (PyObject *)&buffer_core_Type);
  PyObject *d = PyModule_GetDict(m);
  PyDict_SetItemString(d, "TransformException", tf2_exception);
  PyDict_SetItemString(d, "ConnectivityException", tf2_connectivityexception);
  PyDict_SetItemString(d, "LookupException", tf2_lookupexception);
  PyDict_SetItemString(d, "ExtrapolationException", tf2_extrapolationexception);
  PyDict_SetItemString(d, "InvalidArgumentException", tf2_invalidargumentexception);
  PyDict_SetItemString(d, "TimeoutException", tf2_timeoutexception);
  return m;
}

#if PY_MAJOR_VERSION < 3
extern "C"
{
  ROS_HELPER_EXPORT void init_tf2()
  {
    if (!staticInit())
      return;
    moduleInit(Py_InitModule("_tf2", module_methods));
  }
}

#else
struct PyModuleDef tf_module = {
  PyModuleDef_HEAD_INIT, // base
  "_tf2",                // name
  NULL,                  // docstring
  -1,                    // state size (but we're using globals)
  module_methods         // methods
};

PyMODINIT_FUNC PyInit__tf2()
{
  if (!staticInit())
    return NULL;
  return moduleInit(PyModule_Create(&tf_module));
}
#endif
