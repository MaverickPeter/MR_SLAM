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

#ifndef TF2_ROS_BUFFER_INTERFACE_H
#define TF2_ROS_BUFFER_INTERFACE_H

#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>
#include <sstream>
#include <tf2/convert.h>

namespace tf2_ros
{

/** \brief Abstract interface for wrapping tf2::BufferCore in a ROS-based API.
 * Implementations include tf2_ros::Buffer and tf2_ros::BufferClient.
 */
class BufferInterface
{
public:

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual geometry_msgs::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame, 
		    const ros::Time& time, const ros::Duration timeout) const = 0;

  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time. 
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual geometry_msgs::TransformStamped 
    lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		    const std::string& source_frame, const ros::Time& source_time,
		    const std::string& fixed_frame, const ros::Duration timeout) const = 0;


  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  virtual bool
    canTransform(const std::string& target_frame, const std::string& source_frame, 
		 const ros::Time& time, const ros::Duration timeout, std::string* errstr = NULL) const = 0;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
   * \return True if the transform is possible, false otherwise 
   */
  virtual bool
    canTransform(const std::string& target_frame, const ros::Time& target_time,
		 const std::string& source_frame, const ros::Time& source_time,
		 const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr = NULL) const = 0;

  /** \brief Transform an input into the target frame.
   * This function is templated and can take as input any valid mathematical object that tf knows
   * how to apply a transform to, by way of the templated math conversions interface.
   * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
   * type (as defined in geometry_msgs).
   * \tparam T The type of the object to transform.
   * \param in The object to transform
   * \param out The transformed output, preallocated by the caller.
   * \param target_frame The string identifer for the frame to transform into.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   */
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    tf2::doTransform(in, out, lookupTransform(target_frame, tf2::getFrameId(in), tf2::getTimestamp(in), timeout));
    return out;
  }

  /** \brief Transform an input into the target frame.
   * This function is templated and can take as input any valid mathematical object that tf knows
   * how to apply a transform to, by way of the templated math conversions interface.
   * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
   * type (as defined in geometry_msgs).
   * \tparam T The type of the object to transform.
   * \param in The object to transform.
   * \param target_frame The string identifer for the frame to transform into.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   * \return The transformed output.
   */
  template <class T>
    T transform(const T& in,
		const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, timeout);
  }

  /** \brief Transform an input into the target frame and convert to a specified output type.
   * It is templated on two types: the type of the input object and the type of the
   * transformed output.
   * For example, the template types could be Transform, Pose, Vector, or Quaternion messages
   * type (as defined in geometry_msgs).
   * The function will calculate the transformation and then convert the result into the
   * specified output type.
   * Compilation will fail if a known conversion does not exist bewteen the two template
   * parameters.
   * \tparam A The type of the object to transform.
   * \tparam B The type of the transformed output.
   * \param in The object to transform
   * \param out The transformed output, converted to the specified type.
   * \param target_frame The string identifer for the frame to transform into.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   * \return The transformed output, converted to the specified type.
   */
  template <class A, class B>
    B& transform(const A& in, B& out,
        const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    A copy = transform(in, target_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }

  /** \brief Transform an input into the target frame (advanced).
   * This function is templated and can take as input any valid mathematical object that tf knows
   * how to apply a transform to, by way of the templated math conversions interface.
   * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
   * type (as defined in geometry_msgs).
   * This function follows the advanced API, which allows transforming between different time
   * points, and specifying a fixed frame that does not varying in time.
   * \tparam T The type of the object to transform.
   * \param in The object to transform
   * \param out The transformed output, preallocated by the caller.
   * \param target_frame The string identifer for the frame to transform into.
   * \param target_time The time into which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   */
  template <class T>
    T& transform(const T& in, T& out, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    tf2::doTransform(in, out, lookupTransform(target_frame, target_time, 
                                              tf2::getFrameId(in), tf2::getTimestamp(in), 
                                              fixed_frame, timeout));
    return out;
  }


  /** \brief Transform an input into the target frame (advanced).
   * This function is templated and can take as input any valid mathematical object that tf knows
   * how to apply a transform to, by way of the templated math conversions interface.
   * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
   * type (as defined in geometry_msgs).
   * This function follows the advanced API, which allows transforming between different time
   * points, and specifying a fixed frame that does not varying in time.
   * \tparam T The type of the object to transform.
   * \param in The object to transform
   * \param target_frame The string identifer for the frame to transform into.
   * \param target_time The time into which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   * \return The transformed output.
   */
  template <class T>
    T transform(const T& in, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    T out;
    return transform(in, out, target_frame, target_time, fixed_frame, timeout);
  }


  /** \brief Transform an input into the target frame and convert to a specified output type (advanced).
   * It is templated on two types: the type of the input object and the type of the
   * transformed output.
   * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
   * type (as defined in geometry_msgs).
   * The function will calculate the transformation and then convert the result into the
   * specified output type.
   * Compilation will fail if a known conversion does not exist bewteen the two template
   * parameters.
   * This function follows the advanced API, which allows transforming between different time
   * points, and specifying a fixed frame that does not varying in time.
   * \tparam A The type of the object to transform.
   * \tparam B The type of the transformed output.
   * \param in The object to transform
   * \param out The transformed output, converted to the specified output type.
   * \param target_frame The string identifer for the frame to transform into.
   * \param target_time The time into which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time.
   * \param timeout How long to wait for the target frame. Default value is zero (no blocking).
   * \return The transformed output, converted to the specified output type.
   */
  template <class A, class B>
    B& transform(const A& in, B& out, 
		 const std::string& target_frame, const ros::Time& target_time,
		 const std::string& fixed_frame, ros::Duration timeout=ros::Duration(0.0)) const
  {
    // do the transform
    A copy = transform(in, target_frame, target_time, fixed_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }


 }; // class


} // namespace

#endif // TF2_ROS_BUFFER_INTERFACE_H
