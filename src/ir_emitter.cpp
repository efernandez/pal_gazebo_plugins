/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Enrique Fernandez */

#include <pal_gazebo_plugins/ir_emitter.h>

#include <angles/angles.h>

#include <ros/ros.h> //@todo for debugging

#include<cmath>

IREmitter::IREmitter(const std::string& name, double x, double y, double z, double yaw, int code, double fov, double range)
  : name_(name)
  , pose_(x, y, z, 0.0, 0.0, yaw)
  , code_(code)
  , fov_(fov)
  , range_(range)
  , range2_(range_*range_)
  , fov2_(fov_/2)
{}

double IREmitter::power(const gazebo::math::Pose& pose)
{
  const double d2 = (pose_.pos - pose.pos).GetSquaredLength();

  // @todo the pose orientation must be taken into account as weell!!!
  //const double a = std::atan2(dy, dx);

  // @todo check if a in angle +- fov/2
  //double da;
  //angles::shortest_angular_distance_with_limits(angle_, a, -fov2_, fov2_, da);
  //ROS_ERROR_STREAM("a = " << a << "; da = " << da);

  //ROS_ERROR_STREAM("d2 = " << d2 << "; range2 = " << range2_);
  if (d2 > range2_)
    return 0.0;
  else
  {
    // @todo we don't really need a function, but we could
    // provide a linear one; for now just a constant value
    return 1.0;
  }
}

bool IREmitter::isInRange(const gazebo::math::Pose& pose)
{
  return power(pose) > 0.0;
}

void IREmitter::print(std::ostream& os) const
{
  os << "name: "  << name_  << "; "
     << "pose: "  << pose_  << "; "
     << "code: "  << code_  << "; "
     << "fov: "   << fov_   << "; "
     << "range: " << range_;
}

std::ostream& operator<<(std::ostream& os, const IREmitter& v)
{
  v.print(os);
  return os;
}

