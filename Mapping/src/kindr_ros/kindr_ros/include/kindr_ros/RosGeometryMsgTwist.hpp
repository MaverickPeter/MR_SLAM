/*
 * Copyright (c) 2014, Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#pragma once


// kindr
#include <kindr/Core>

// ros
#include <geometry_msgs/Twist.h>

// kindr ros
#include "kindr_ros/RosGeometryMsgPhysicalQuantities.hpp"


namespace kindr_ros {


template<typename PrimType_, typename PositionDiff_, typename RotationDiff_>
inline static void convertFromRosGeometryMsg(
    const geometry_msgs::Twist& geometryTwistMsg,
    kindr::Twist<PrimType_, PositionDiff_, RotationDiff_>& twist)
{
  convertFromRosGeometryMsg(geometryTwistMsg.linear, twist.getTranslationalVelocity());
  convertFromRosGeometryMsg(geometryTwistMsg.angular, twist.getRotationalVelocity());
}

template<typename PrimType_, typename PositionDiff_, typename RotationDiff_>
inline static void convertToRosGeometryMsg(
    const kindr::Twist<PrimType_, PositionDiff_, RotationDiff_>& twist,
    geometry_msgs::Twist& geometryTwistMsg)
{
  convertToRosGeometryMsg(twist.getTranslationalVelocity(), geometryTwistMsg.linear);
  convertToRosGeometryMsg(twist.getRotationalVelocity(), geometryTwistMsg.angular);
}


} // namespace kindr_ros
