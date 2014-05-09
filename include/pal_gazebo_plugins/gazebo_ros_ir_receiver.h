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

#ifndef GAZEBO_ROS_IR_RECEIVER_H
#define GAZEBO_ROS_IR_RECEIVER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Vector3.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
class GazeboRosIRReceiver : public SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosIRReceiver();

  /// \brief Destructor
  virtual ~GazeboRosIRReceiver();

  /// \brief Load the controller
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

private:
  /// \brief connected by updateConnection, called when contact
  void UpdateChild();

  /// \brief IR receiver callback queue thread
  void IRReceiverQueueThread();

  /// \brief thread out Load function with anything that might be blocking
  void LoadThread();

  /// \brief for setting ROS name space
  std::string robot_namespace_;
  std::string topic_name_;
  std::string frame_name_;
  double      update_rate_;
  double      update_period_;

  /// \brief Pointer to the update event connections
  event::ConnectionPtr updateConnection;

  /// \brief Pointer to the sensor
  sensors::SensorPtr sensor_;
  ros::Publisher pub_;

  boost::thread deferred_load_thread_;

  // ROS stuff
  ros::NodeHandle* nh_;
  ros::CallbackQueue ir_receiver_queue_;
  boost::thread callback_queue_thread_;

  // Pointer to the world
  physics::WorldPtr world_;

  // Controls stuff
  common::Time last_update_time_;

  // Mutex
  boost::mutex mutex_;
};
}

#endif // GAZEBO_ROS_IR_RECEIVER_H
