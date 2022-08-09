/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */

#ifndef TF2_GEOMETRY_MSGS_H
#define TF2_GEOMETRY_MSGS_H

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/frames.hpp>

#include <array>

#include "ros/macros.h"

namespace tf2
{

/** \brief Convert a TransformStamped message to a KDL frame.
 * \param t TransformStamped message to convert.
 * \return The converted KDL Frame.
 * \deprecated
 */
inline
ROS_DEPRECATED KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t);
inline
KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }


/*************/
/** Vector3 **/
/*************/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Vector3 toMsg(const tf2::Vector3& in)
{
  geometry_msgs::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/********************/
/** Vector3Stamped **/
/********************/

/** \brief Extract a timestamp from the header of a Vector message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t VectorStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::Vector3Stamped& t) {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Vector message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t VectorStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::Vector3Stamped& t) {return t.header.frame_id;}


/** \brief Trivial "conversion" function for Vector3 message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Vector3Stamped message.
 * \return The input argument.
 */
inline
geometry_msgs::Vector3Stamped toMsg(const geometry_msgs::Vector3Stamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Vector3 message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A Vector3Stamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, geometry_msgs::Vector3Stamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Vector3 specialization of the tf2::Stamped template.
 * \return The Vector3Stamped converted to a geometry_msgs Vector3Stamped message type.
 */
inline
geometry_msgs::Vector3Stamped toMsg(const tf2::Stamped<tf2::Vector3>& in)
{
  geometry_msgs::Vector3Stamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.vector.x = in.getX();
  out.vector.y = in.getY();
  out.vector.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3Stamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A Vector3Stamped message.
 * \param out The Vector3Stamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Vector3Stamped& msg, tf2::Stamped<tf2::Vector3>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(tf2::Vector3(msg.vector.x, msg.vector.y, msg.vector.z));
}


/***********/
/** Point **/
/***********/

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Point& toMsg(const tf2::Vector3& in, geometry_msgs::Point& out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Point& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}


/******************/
/** PointStamped **/
/******************/

/** \brief Extract a timestamp from the header of a Point message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PointStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PointStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Point message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PointStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PointStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PointStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::PointStamped toMsg(const geometry_msgs::PointStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Point message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PointStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, geometry_msgs::PointStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Vector3 specialization of the tf2::Stamped template.
 * \return The Vector3Stamped converted to a geometry_msgs PointStamped message type.
 */
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<tf2::Vector3>& in, geometry_msgs::PointStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.point.x = in.getX();
  out.point.y = in.getY();
  out.point.z = in.getZ();
  return out;
}

/** \brief Convert a PointStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PointStamped message.
 * \param out The PointStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<tf2::Vector3>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(tf2::Vector3(msg.point.x, msg.point.y, msg.point.z));
}


/****************/
/** Quaternion **/
/****************/

/** \brief Convert a tf2 Quaternion type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Quaternion object.
 * \return The Quaternion converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Quaternion toMsg(const tf2::Quaternion& in)
{
  geometry_msgs::Quaternion out;
  out.w = in.getW();
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Quaternion message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Quaternion message type.
 * \param out The Quaternion converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}


/***********************/
/** QuaternionStamped **/
/***********************/

/** \brief Extract a timestamp from the header of a Quaternion message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t QuaternionStamped message to extract the timestamp from.
 * \return The timestamp of the message. The lifetime of the returned reference
 * is bound to the lifetime of the argument.
 */
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::QuaternionStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Quaternion message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t QuaternionStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message. The lifetime of the
 * returned reference is bound to the lifetime of the argument.
 */
template <>
inline
const std::string& getFrameId(const geometry_msgs::QuaternionStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Quaternion message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A QuaternionStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const geometry_msgs::QuaternionStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Quaternion message type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A QuaternionStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& msg, geometry_msgs::QuaternionStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Quaternion type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Quaternion specialization of the tf2::Stamped template.
 * \return The QuaternionStamped converted to a geometry_msgs QuaternionStamped message type.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)
{
  geometry_msgs::QuaternionStamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.quaternion.w = in.getW();
  out.quaternion.x = in.getX();
  out.quaternion.y = in.getY();
  out.quaternion.z = in.getZ();
  return out;
}

template <>
inline
ROS_DEPRECATED geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in);


//Backwards compatibility remove when forked for Lunar or newer
template <>
inline
geometry_msgs::QuaternionStamped toMsg(const tf2::Stamped<tf2::Quaternion>& in)
{
  return toMsg(in);
}

/** \brief Convert a QuaternionStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A QuaternionStamped message type.
 * \param out The QuaternionStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out)
{
  out.stamp_ = in.header.stamp;
  out.frame_id_ = in.header.frame_id;
  tf2::Quaternion tmp;
  fromMsg(in.quaternion, tmp);
  out.setData(tmp);
}

template<>
inline
ROS_DEPRECATED void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out);

//Backwards compatibility remove when forked for Lunar or newer
template<>
inline
void fromMsg(const geometry_msgs::QuaternionStamped& in, tf2::Stamped<tf2::Quaternion>& out)
{
    fromMsg(in, out);
}

/**********/
/** Pose **/
/**********/

/** \brief Convert a tf2 Transform type to an equivalent geometry_msgs Pose message.
 * \param in A tf2 Transform object.
 * \param out The Transform converted to a geometry_msgs Pose message type.
 */
inline
geometry_msgs::Pose& toMsg(const tf2::Transform& in, geometry_msgs::Pose& out)
{
  toMsg(in.getOrigin(), out.position);
  out.orientation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a geometry_msgs Pose message to an equivalent tf2 Transform type.
 * \param in A Pose message.
 * \param out The Pose converted to a tf2 Transform type.
 */
inline
void fromMsg(const geometry_msgs::Pose& in, tf2::Transform& out)
{
  out.setOrigin(tf2::Vector3(in.position.x, in.position.y, in.position.z));
  // w at the end in the constructor
  out.setRotation(tf2::Quaternion(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w));
}



/*****************/
/** PoseStamped **/
/*****************/

/** \brief Extract a timestamp from the header of a Pose message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PoseStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Pose message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PoseStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PoseStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PoseStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for Pose message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A PoseStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Pose type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Pose specialization of the tf2::Stamped template.
 * \return The PoseStamped converted to a geometry_msgs PoseStamped message type.
 */
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<tf2::Transform>& in, geometry_msgs::PoseStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  toMsg(in.getOrigin(), out.pose.position);
  out.pose.orientation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a PoseStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PoseStamped message.
 * \param out The PoseStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}

/*******************************/
/** PoseWithCovarianceStamped **/
/*******************************/

/** \brief Extract a timestamp from the header of a PoseWithCovarianceStamped message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t PoseWithCovarianceStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
  const ros::Time& getTimestamp(const geometry_msgs::PoseWithCovarianceStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a PoseWithCovarianceStamped message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t PoseWithCovarianceStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
  const std::string& getFrameId(const geometry_msgs::PoseWithCovarianceStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for PoseWithCovarianceStamped message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A PoseWithCovarianceStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::PoseWithCovarianceStamped toMsg(const geometry_msgs::PoseWithCovarianceStamped& in)
{
  return in;
}

/** \brief Trivial "conversion" function for PoseWithCovarianceStamped message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A PoseWithCovarianceStamped message.
 * \param out The input argument.
 */
inline
void fromMsg(const geometry_msgs::PoseWithCovarianceStamped& msg, geometry_msgs::PoseWithCovarianceStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 PoseWithCovarianceStamped type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Pose specialization of the tf2::Stamped template.
 * \return The PoseWithCovarianceStamped converted to a geometry_msgs PoseWithCovarianceStamped message type.
 */
inline
geometry_msgs::PoseWithCovarianceStamped toMsg(const tf2::Stamped<tf2::Transform>& in, geometry_msgs::PoseWithCovarianceStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  toMsg(in.getOrigin(), out.pose.pose.position);
  out.pose.pose.orientation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a PoseWithCovarianceStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A PoseWithCovarianceStamped message.
 * \param out The PoseWithCovarianceStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::PoseWithCovarianceStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}

/***************/
/** Transform **/
/***************/

/** \brief Convert a tf2 Transform type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Transform object.
 * \return The Transform converted to a geometry_msgs message type.
 */
inline
geometry_msgs::Transform toMsg(const tf2::Transform& in)
{
  geometry_msgs::Transform out;
  out.translation = toMsg(in.getOrigin());
  out.rotation = toMsg(in.getRotation());
  return out;
}

/** \brief Convert a Transform message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A Transform message type.
 * \param out The Transform converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  tf2::Vector3 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  // w at the end in the constructor
  tf2::Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}


/**********************/
/** TransformStamped **/
/**********************/

/** \brief Extract a timestamp from the header of a Transform message.
 * This function is a specialization of the getTimestamp template defined in tf2/convert.h.
 * \param t TransformStamped message to extract the timestamp from.
 * \return The timestamp of the message.
 */
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::TransformStamped& t)  {return t.header.stamp;}

/** \brief Extract a frame ID from the header of a Transform message.
 * This function is a specialization of the getFrameId template defined in tf2/convert.h.
 * \param t TransformStamped message to extract the frame ID from.
 * \return A string containing the frame ID of the message.
 */
template <>
inline
const std::string& getFrameId(const geometry_msgs::TransformStamped& t)  {return t.header.frame_id;}

/** \brief Trivial "conversion" function for Transform message type.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A TransformStamped message.
 * \return The input argument.
 */
inline
geometry_msgs::TransformStamped toMsg(const geometry_msgs::TransformStamped& in)
{
  return in;
}

/** \brief Convert a TransformStamped message to its equivalent tf2 representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg A TransformStamped message type.
 * \param out The TransformStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::TransformStamped& msg, geometry_msgs::TransformStamped& out)
{
  out = msg;
}

/** \brief Convert as stamped tf2 Transform type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in An instance of the tf2::Transform specialization of the tf2::Stamped template.
 * \return The tf2::Stamped<tf2::Transform> converted to a geometry_msgs TransformStamped message type.
 */
inline
geometry_msgs::TransformStamped toMsg(const tf2::Stamped<tf2::Transform>& in)
{
  geometry_msgs::TransformStamped out;
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.transform.translation = toMsg(in.getOrigin());
  out.transform.rotation = toMsg(in.getRotation());
  return out;
}


/** \brief Convert a TransformStamped message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param msg A TransformStamped message.
 * \param out The TransformStamped converted to the equivalent tf2 type.
 */
inline
void fromMsg(const geometry_msgs::TransformStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.transform, tmp);
  out.setData(tmp);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point3 message.
 * \param t_out The transformed point, as a Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Point& t_in, geometry_msgs::Point& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 v_in;
    fromMsg(t_in, v_in);
    tf2::Vector3 v_out = t * v_in;
    toMsg(v_out, t_out);
  }

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Point type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a timestamped Point3 message.
 * \param t_out The transformed point, as a timestamped Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::PointStamped& t_in, geometry_msgs::PointStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    doTransform(t_in.point, t_out.point, transform);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a Quaternion3 message.
 * \param t_out The transformed quaternion, as a Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::Quaternion& t_in, geometry_msgs::Quaternion& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Quaternion t, q_in;
  fromMsg(transform.transform.rotation, t);
  fromMsg(t_in, q_in);

  tf2::Quaternion q_out = t * q_in;
  t_out = toMsg(q_out);
}

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Quaternion type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The quaternion to transform, as a timestamped Quaternion3 message.
 * \param t_out The transformed quaternion, as a timestamped Quaternion3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::QuaternionStamped& t_in, geometry_msgs::QuaternionStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.quaternion, t_out.quaternion, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}


/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a Pose3 message.
* \param t_out The transformed pose, as a Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::Pose& t_in, geometry_msgs::Pose& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out);
}

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Pose type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped Pose3 message.
* \param t_out The transformed pose, as a timestamped Pose3 message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.pose, t_out.pose, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

/** \brief Transform the covariance matrix of a PoseWithCovarianceStamped message to a new frame.
* \param t_in The covariance matrix to transform.
* \param transform The timestamped transform to apply, as a TransformStamped message.
* \return The transformed covariance matrix.
*/
inline
geometry_msgs::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& cov_in, const tf2::Transform& transform)
{
  /**
   * To transform a covariance matrix:
   * 
   * [R 0] COVARIANCE [R' 0 ]
   * [0 R]            [0  R']
   * 
   * Where:
   * 	R is the rotation matrix (3x3).
   * 	R' is the transpose of the rotation matrix.
   * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
   */ 
  
  // get rotation matrix transpose  
  const tf2::Matrix3x3  R_transpose = transform.getBasis().transpose();
  
  // convert the covariance matrix into four 3x3 blocks
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2],
			      cov_in[6], cov_in[7], cov_in[8],
			      cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5],
			      cov_in[9], cov_in[10], cov_in[11],
			      cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20],
			      cov_in[24], cov_in[25], cov_in[26],
			      cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23],
			      cov_in[27], cov_in[28], cov_in[29],
			      cov_in[33], cov_in[34], cov_in[35]);
  
  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = transform.getBasis()*cov_11*R_transpose;
  const tf2::Matrix3x3 result_12 = transform.getBasis()*cov_12*R_transpose;
  const tf2::Matrix3x3 result_21 = transform.getBasis()*cov_21*R_transpose;
  const tf2::Matrix3x3 result_22 = transform.getBasis()*cov_22*R_transpose;
  
  // form the output
  geometry_msgs::PoseWithCovariance::_covariance_type output;
  output[0] = result_11[0][0];
  output[1] = result_11[0][1];
  output[2] = result_11[0][2];
  output[6] = result_11[1][0];
  output[7] = result_11[1][1];
  output[8] = result_11[1][2];
  output[12] = result_11[2][0];
  output[13] = result_11[2][1];
  output[14] = result_11[2][2];
  
  output[3] = result_12[0][0];
  output[4] = result_12[0][1];
  output[5] = result_12[0][2];
  output[9] = result_12[1][0];
  output[10] = result_12[1][1];
  output[11] = result_12[1][2];
  output[15] = result_12[2][0];
  output[16] = result_12[2][1];
  output[17] = result_12[2][2];
  
  output[18] = result_21[0][0];
  output[19] = result_21[0][1];
  output[20] = result_21[0][2];
  output[24] = result_21[1][0];
  output[25] = result_21[1][1];
  output[26] = result_21[1][2];
  output[30] = result_21[2][0];
  output[31] = result_21[2][1];
  output[32] = result_21[2][2];
  
  output[21] = result_22[0][0];
  output[22] = result_22[0][1];
  output[23] = result_22[0][2];
  output[27] = result_22[1][0];
  output[28] = result_22[1][1];
  output[29] = result_22[1][2];
  output[33] = result_22[2][0];
  output[34] = result_22[2][1];
  output[35] = result_22[2][2];
  
  return output;
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs PoseWithCovarianceStamped type.
* This function is a specialization of the doTransform template defined in tf2/convert.h.
* \param t_in The pose to transform, as a timestamped PoseWithCovarianceStamped message.
* \param t_out The transformed pose, as a timestamped PoseWithCovarianceStamped message.
* \param transform The timestamped transform to apply, as a TransformStamped message.
*/
template <>
inline
void doTransform(const geometry_msgs::PoseWithCovarianceStamped& t_in, geometry_msgs::PoseWithCovarianceStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Vector3 v;
  fromMsg(t_in.pose.pose.position, v);
  tf2::Quaternion r;
  fromMsg(t_in.pose.pose.orientation, r);

  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Transform v_out = t * tf2::Transform(r, v);
  toMsg(v_out, t_out.pose.pose);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;

  t_out.pose.covariance = transformCovariance(t_in.pose.covariance, t);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Transform type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The frame to transform, as a timestamped Transform3 message.
 * \param t_out The frame transform, as a timestamped Transform3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::TransformStamped& t_in, geometry_msgs::TransformStamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform input;
    fromMsg(t_in.transform, input);

    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Transform v_out = t * input;

    t_out.transform = toMsg(v_out);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a Vector3 message.
 * \param t_out The transformed vector, as a Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Vector3& t_in, geometry_msgs::Vector3& t_out, const geometry_msgs::TransformStamped& transform)
  {
    tf2::Transform t;
    fromMsg(transform.transform, t);
    tf2::Vector3 v_out = t.getBasis() * tf2::Vector3(t_in.x, t_in.y, t_in.z);
    t_out.x = v_out[0];
    t_out.y = v_out[1];
    t_out.z = v_out[2];
  }

/** \brief Apply a geometry_msgs TransformStamped to an stamped geometry_msgs Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Vector3 message.
 * \param t_out The transformed vector, as a timestamped Vector3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out, const geometry_msgs::TransformStamped& transform)
  {
    doTransform(t_in.vector, t_out.vector, transform);
    t_out.header.stamp = transform.header.stamp;
    t_out.header.frame_id = transform.header.frame_id;
  }


/**********************/
/*** WrenchStamped ****/
/**********************/
template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::WrenchStamped& t) {return t.header.stamp;}


template <>
inline
const std::string& getFrameId(const geometry_msgs::WrenchStamped& t) {return t.header.frame_id;}


inline
geometry_msgs::WrenchStamped toMsg(const geometry_msgs::WrenchStamped& in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::WrenchStamped& msg, geometry_msgs::WrenchStamped& out)
{
  out = msg;
}


inline
geometry_msgs::WrenchStamped toMsg(const tf2::Stamped<std::array<tf2::Vector3, 2>>& in, geometry_msgs::WrenchStamped & out)
{
  out.header.stamp = in.stamp_;
  out.header.frame_id = in.frame_id_;
  out.wrench.force = toMsg(in[0]);
  out.wrench.torque = toMsg(in[1]);
  return out;
}


inline
void fromMsg(const geometry_msgs::WrenchStamped& msg, tf2::Stamped<std::array<tf2::Vector3, 2>>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Vector3 tmp;
  fromMsg(msg.wrench.force, tmp);
  tf2::Vector3 tmp1;
  fromMsg(msg.wrench.torque, tmp1);
  std::array<tf2::Vector3, 2> tmp_array;
  tmp_array[0] = tmp;
  tmp_array[1] = tmp1;
  out.setData(tmp_array);
}

template<>
inline
void doTransform(const geometry_msgs::Wrench& t_in, geometry_msgs::Wrench& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.force, t_out.force, transform);
  doTransform(t_in.torque, t_out.torque, transform);
}


template<>
inline
void doTransform(const geometry_msgs::WrenchStamped& t_in, geometry_msgs::WrenchStamped& t_out, const geometry_msgs::TransformStamped& transform)
{
  doTransform(t_in.wrench, t_out.wrench, transform);
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

} // namespace

#endif // TF2_GEOMETRY_MSGS_H
