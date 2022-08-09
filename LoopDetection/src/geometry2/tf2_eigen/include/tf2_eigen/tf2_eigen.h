/*
 * Copyright (c) Koji Terada
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

/** \author Koji Terada */

#ifndef TF2_EIGEN_H
#define TF2_EIGEN_H

#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>


namespace tf2
{

/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs Transform message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
 inline
 Eigen::Isometry3d transformToEigen(const geometry_msgs::Transform& t) {
 return Eigen::Isometry3d(Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
			 * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
}

/** \brief Convert a timestamped transform to the equivalent Eigen data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an Eigen Isometry3d transform.
 */
inline
Eigen::Isometry3d transformToEigen(const geometry_msgs::TransformStamped& t) {
  return transformToEigen(t.transform);
}

/** \brief Convert an Eigen Affine3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Affine3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped eigenToTransform(const Eigen::Affine3d& T)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.linear());  // assuming that upper 3x3 matrix is orthonormal
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}

/** \brief Convert an Eigen Isometry3d transform to the equivalent geometry_msgs message type.
 * \param T The transform to convert, as an Eigen Isometry3d transform.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped eigenToTransform(const Eigen::Isometry3d& T)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = T.translation().x();
  t.transform.translation.y = T.translation().y();
  t.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.rotation());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Vector3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a Eigen Vector3d data type.
 * \param t_out The transformed vector, as a Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const Eigen::Vector3d& t_in, Eigen::Vector3d& t_out, const geometry_msgs::TransformStamped& transform)
{
  t_out = Eigen::Vector3d(transformToEigen(transform) * t_in);
}

/** \brief Convert a Eigen Vector3d type to a Point message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped Eigen Vector3d to convert.
 * \return The vector converted to a Point message.
 */
inline
geometry_msgs::Point toMsg(const Eigen::Vector3d& in)
{
  geometry_msgs::Point msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

/** \brief Convert a Point message type to a Eigen-specific Vector3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Point message to convert.
 * \param out The point converted to a Eigen Vector3d.
 */
inline
void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d& out)
{
  out.x() = msg.x;
  out.y() = msg.y;
  out.z() = msg.z;
}

/** \brief Convert an Eigen Vector3d type to a Vector3 message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Vector3d to convert.
 * \return The vector converted to a Vector3 message.
 */
inline
geometry_msgs::Vector3& toMsg(const Eigen::Vector3d& in, geometry_msgs::Vector3& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  return out;
}

/** \brief Convert a Vector3 message type to a Eigen-specific Vector3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Vector3 message to convert.
 * \param out The vector converted to a Eigen Vector3d.
 */
inline
void fromMsg(const geometry_msgs::Vector3& msg, Eigen::Vector3d& out)
{
  out.x() = msg.x;
  out.y() = msg.y;
  out.z() = msg.z;
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Vector3d type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Eigen Vector3d data type.
 * \param t_out The transformed vector, as a timestamped Eigen Vector3d data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Vector3d>& t_in,
		 tf2::Stamped<Eigen::Vector3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Vector3d>(transformToEigen(transform) * t_in,
					transform.header.stamp,
					transform.header.frame_id);
}

/** \brief Convert a stamped Eigen Vector3d type to a PointStamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped Eigen Vector3d to convert.
 * \return The vector converted to a PointStamped message.
 */
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<Eigen::Vector3d>& in)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.point = toMsg(static_cast<const Eigen::Vector3d&>(in));
  return msg;
}

/** \brief Convert a PointStamped message type to a stamped Eigen-specific Vector3d type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The PointStamped message to convert.
 * \param out The point converted to a timestamped Eigen Vector3d.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<Eigen::Vector3d>& out) {
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.point, static_cast<Eigen::Vector3d&>(out));
}

/** \brief Apply a geometry_msgs Transform to an Eigen Affine3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a Eigen Affine3d transform.
 * \param t_out The transformed frame, as a Eigen Affine3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const Eigen::Affine3d& t_in,
                 Eigen::Affine3d& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  t_out = Eigen::Affine3d(transformToEigen(transform) * t_in);
}

template <>
inline
void doTransform(const Eigen::Isometry3d& t_in,
                 Eigen::Isometry3d& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  t_out = Eigen::Isometry3d(transformToEigen(transform) * t_in);
}

/** \brief Convert a Eigen Quaterniond type to a Quaternion message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Quaterniond to convert.
 * \return The quaternion converted to a Quaterion message.
 */
inline
geometry_msgs::Quaternion toMsg(const Eigen::Quaterniond& in) {
 geometry_msgs::Quaternion msg;
 msg.w = in.w();
 msg.x = in.x();
 msg.y = in.y();
 msg.z = in.z();
 return msg;
}

/** \brief Convert a Quaternion message type to a Eigen-specific Quaterniond type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The Quaternion message to convert.
 * \param out The quaternion converted to a Eigen Quaterniond.
 */
inline
void fromMsg(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond& out) {
  out = Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * functions rely on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The vector to transform, as a Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template<>
inline
void doTransform(const Eigen::Quaterniond& t_in,
                 Eigen::Quaterniond& t_out,
                 const geometry_msgs::TransformStamped& transform) {
  Eigen::Quaterniond t;
  fromMsg(transform.transform.rotation, t);
  t_out = t.inverse() * t_in * t;
}

/** \brief Convert a stamped Eigen Quaterniond type to a QuaternionStamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped Eigen Quaterniond to convert.
 * \return The quaternion converted to a QuaternionStamped message.
 */
inline
geometry_msgs::QuaternionStamped toMsg(const Stamped<Eigen::Quaterniond>& in) {
  geometry_msgs::QuaternionStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.quaternion = toMsg(static_cast<const Eigen::Quaterniond&>(in));
  return msg;
}

/** \brief Convert a QuaternionStamped message type to a stamped Eigen-specific Quaterniond type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The QuaternionStamped message to convert.
 * \param out The quaternion converted to a timestamped Eigen Quaterniond.
 */
inline
void fromMsg(const geometry_msgs::QuaternionStamped& msg, Stamped<Eigen::Quaterniond>& out) {
  out.frame_id_ = msg.header.frame_id;
  out.stamp_ = msg.header.stamp;
  fromMsg(msg.quaternion, static_cast<Eigen::Quaterniond&>(out));
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen-specific Quaterniond type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped Eigen Quaterniond data type.
 * \param t_out The transformed vector, as a timestamped Eigen Quaterniond data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Quaterniond>& t_in,
     tf2::Stamped<Eigen::Quaterniond>& t_out,
     const geometry_msgs::TransformStamped& transform) {
  t_out.frame_id_ = transform.header.frame_id;
  t_out.stamp_ = transform.header.stamp;
  doTransform(static_cast<const Eigen::Quaterniond&>(t_in), static_cast<Eigen::Quaterniond&>(t_out), transform);
}

/** \brief Convert a Eigen Affine3d transform type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Affine3d to convert.
 * \return The Eigen transform converted to a Pose message.
 */
inline
geometry_msgs::Pose toMsg(const Eigen::Affine3d& in) {
  geometry_msgs::Pose msg;
  msg.position.x = in.translation().x();
  msg.position.y = in.translation().y();
  msg.position.z = in.translation().z();
  Eigen::Quaterniond q(in.linear());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  if (msg.orientation.w < 0) {
    msg.orientation.x *= -1;
    msg.orientation.y *= -1;
    msg.orientation.z *= -1;
    msg.orientation.w *= -1;
  }
  return msg;
}

/** \brief Convert a Eigen Isometry3d transform type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The Eigen Isometry3d to convert.
 * \return The Eigen transform converted to a Pose message.
 */
inline
geometry_msgs::Pose toMsg(const Eigen::Isometry3d& in) {
  geometry_msgs::Pose msg;
  msg.position.x = in.translation().x();
  msg.position.y = in.translation().y();
  msg.position.z = in.translation().z();
  Eigen::Quaterniond q(in.linear());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  if (msg.orientation.w < 0) {
    msg.orientation.x *= -1;
    msg.orientation.y *= -1;
    msg.orientation.z *= -1;
    msg.orientation.w *= -1;
  }
  return msg;
}

/** \brief Convert a Pose message transform type to a Eigen Affine3d.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The Pose message to convert.
 * \param out The pose converted to a Eigen Affine3d.
 */
inline
void fromMsg(const geometry_msgs::Pose& msg, Eigen::Affine3d& out) {
  out = Eigen::Affine3d(
      Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
      Eigen::Quaterniond(msg.orientation.w,
                         msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z));
}

/** \brief Convert a Pose message transform type to a Eigen Isometry3d.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The Pose message to convert.
 * \param out The pose converted to a Eigen Isometry3d.
 */
inline
void fromMsg(const geometry_msgs::Pose& msg, Eigen::Isometry3d& out) {
  out = Eigen::Isometry3d(
      Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
      Eigen::Quaterniond(msg.orientation.w,
                         msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z));
}

/** \brief Convert a Eigen 6x1 Matrix type to a Twist message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The 6x1 Eigen Matrix to convert.
 * \return The Eigen Matrix converted to a Twist message.
 */
inline
geometry_msgs::Twist toMsg(const Eigen::Matrix<double,6,1>& in) {
  geometry_msgs::Twist msg;
  msg.linear.x = in[0];
  msg.linear.y = in[1];
  msg.linear.z = in[2];
  msg.angular.x = in[3];
  msg.angular.y = in[4];
  msg.angular.z = in[5];
  return msg;
}

/** \brief Convert a Twist message transform type to a Eigen 6x1 Matrix.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The Twist message to convert.
 * \param out The twist converted to a Eigen 6x1 Matrix.
 */
inline
void fromMsg(const geometry_msgs::Twist &msg, Eigen::Matrix<double,6,1>& out) {
  out[0] = msg.linear.x;
  out[1] = msg.linear.y;
  out[2] = msg.linear.z;
  out[3] = msg.angular.x;
  out[4] = msg.angular.y;
  out[5] = msg.angular.z;
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen Affine3d transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a timestamped Eigen Affine3d transform.
 * \param t_out The transformed frame, as a timestamped Eigen Affine3d transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Affine3d>& t_in,
		 tf2::Stamped<Eigen::Affine3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Affine3d>(transformToEigen(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

/** \brief Apply a geometry_msgs TransformStamped to an Eigen Isometry transform.
 * This function is a specialization of the doTransform template defined in tf2/convert.h,
 * although it can not be used in tf2_ros::BufferInterface::transform because this
 * function relies on the existence of a time stamp and a frame id in the type which should
 * get transformed.
 * \param t_in The frame to transform, as a timestamped Eigen Isometry transform.
 * \param t_out The transformed frame, as a timestamped Eigen Isometry transform.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const tf2::Stamped<Eigen::Isometry3d>& t_in,
		 tf2::Stamped<Eigen::Isometry3d>& t_out,
		 const geometry_msgs::TransformStamped& transform) {
  t_out = tf2::Stamped<Eigen::Isometry3d>(transformToEigen(transform) * t_in, transform.header.stamp, transform.header.frame_id);
}

/** \brief Convert a stamped Eigen Affine3d transform type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped Eigen Affine3d to convert.
 * \return The Eigen transform converted to a PoseStamped message.
 */
inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<Eigen::Affine3d>& in)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.pose = toMsg(static_cast<const Eigen::Affine3d&>(in));
  return msg;
}

inline
geometry_msgs::PoseStamped toMsg(const tf2::Stamped<Eigen::Isometry3d>& in)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.pose = toMsg(static_cast<const Eigen::Isometry3d&>(in));
  return msg;
}

/** \brief Convert a Pose message transform type to a stamped Eigen Affine3d.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param msg The PoseStamped message to convert.
 * \param out The pose converted to a timestamped Eigen Affine3d.
 */
inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<Eigen::Affine3d>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.pose, static_cast<Eigen::Affine3d&>(out));
}

inline
void fromMsg(const geometry_msgs::PoseStamped& msg, tf2::Stamped<Eigen::Isometry3d>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  fromMsg(msg.pose, static_cast<Eigen::Isometry3d&>(out));
}

} // namespace


namespace Eigen {
// This is needed to make the usage of the following conversion functions usable in tf2::convert().
// According to clangs error note 'fromMsg'/'toMsg' should be declared prior to the call site or
// in an associated namespace of one of its arguments. The stamped versions of this conversion
// functions work because they have tf2::Stamped as an argument which is the same namespace as
// which 'fromMsg'/'toMsg' is defined in. The non-stamped versions have no argument which is
// defined in tf2, so it take the following definitions in Eigen namespace to make them usable in
// tf2::convert().

inline
geometry_msgs::Pose toMsg(const Eigen::Affine3d& in) {
  return tf2::toMsg(in);
}

inline
geometry_msgs::Pose toMsg(const Eigen::Isometry3d& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d& out) {
  tf2::fromMsg(msg, out);
}

inline
geometry_msgs::Point toMsg(const Eigen::Vector3d& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Pose& msg, Eigen::Affine3d& out) {
  tf2::fromMsg(msg, out);
}

inline
void fromMsg(const geometry_msgs::Pose& msg, Eigen::Isometry3d& out) {
  tf2::fromMsg(msg, out);
}

inline
geometry_msgs::Quaternion toMsg(const Eigen::Quaterniond& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond& out) {
  tf2::fromMsg(msg, out);
}

inline
geometry_msgs::Twist toMsg(const Eigen::Matrix<double,6,1>& in) {
  return tf2::toMsg(in);
}

inline
void fromMsg(const geometry_msgs::Twist &msg, Eigen::Matrix<double,6,1>& out) {
  tf2::fromMsg(msg, out);
}

} // namespace

#endif // TF2_EIGEN_H
