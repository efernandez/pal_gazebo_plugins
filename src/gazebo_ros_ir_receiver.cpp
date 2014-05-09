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

#include <pal_gazebo_plugins/gazebo_ros_ir_receiver.h>

#include <string>

#include <kobuki_msgs/DockInfraRed.h>

#include <tf/tf.h>

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIRReceiver)

GazeboRosIRReceiver::GazeboRosIRReceiver()
{}

GazeboRosIRReceiver::~GazeboRosIRReceiver()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  ir_receiver_queue_.clear();
  ir_receiver_queue_.disable();
  nh_->shutdown();
  callback_queue_thread_.join();

  delete nh_;
}

void GazeboRosIRReceiver::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  // @todo organize better, probably following:
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/hydro-devel/gazebo_plugins/src/gazebo_ros_block_laser.cpp

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  robot_namespace_ = "";
  if (sdf->HasElement("robotNamespace"))
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  topic_name_ = "ir_receiver";
  if (!sdf->GetElement("topicName"))
    ROS_WARN_STREAM("IR receiver sensor plugin missing <topicName>, defaults to " << topic_name_);
  else
    topic_name_ = sdf->GetElement("topicName")->Get<std::string>();

  frame_name_ = "world";
  if (!sdf->HasElement("frameName"))
    ROS_WARN_STREAM("IR reciever sensor plugin missing <frameName>, defaults to " << frame_name_);
  else
    frame_name_ = sdf->GetElement("frameName")->Get<std::string>();

  update_rate_ = 100.0;
  if (!sdf->HasElement("updateRate"))
    ROS_WARN_STREAM("IR reciever sensor plugin missing <updateRate>, defaults to " << update_rate_);
  else
    update_rate_ = sdf->GetElement("updateRate")->Get<double>();

  update_period_ = update_rate_ > 0.0 ? 1.0/update_rate_ : 0.0;

  // Get the world name
  const std::string world_name = parent->GetWorldName();
  world_ = physics::get_world(world_name);

  last_update_time_ = world_->GetSimTime();

  // Get sensor
  sensor_ = parent;
  if (!sensor_)
    gzerr << "sensor not found\n" << "\n";

  deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosIRReceiver::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosIRReceiver::LoadThread()
{
  nh_ = new ros::NodeHandle(robot_namespace_);
  nh_->setCallbackQueue(&ir_receiver_queue_);

  // resolve tf prefix
  std::string prefix = "";
  nh_->param("tf_prefix", prefix, prefix);
  frame_name_ = tf::resolve(prefix, frame_name_);

  pub_ = nh_->advertise<kobuki_msgs::DockInfraRed>(topic_name_, 10);

  // start custum queue for IR receiver
  callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosIRReceiver::IRReceiverQueueThread, this));

  // Connect to the sensor update event.
  updateConnection = sensor_->ConnectUpdated(
              boost::bind(&GazeboRosIRReceiver::UpdateChild, this));

  // Make sure the parent sensor is active.
  sensor_->SetActive(true);
}

void GazeboRosIRReceiver::UpdateChild()
{
  if (pub_.getNumSubscribers() <= 0)
    return;

  boost::mutex::scoped_lock lock(mutex_);

  common::Time now = world_->GetSimTime();
  if (( now - last_update_time_) >= update_period_)
  {
    ROS_ERROR_THROTTLE(10.0, "IR Receiver logic not implemented yet!");
    last_update_time_ = now;

    // @todo logic for the IRs
    kobuki_msgs::DockInfraRed msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp.sec = now.sec;
    msg.header.stamp.nsec = now.nsec;

    msg.data.clear();
    msg.data.push_back(1);
    msg.data.push_back(2);
    msg.data.push_back(3);

    pub_.publish(msg);
  }
}

void GazeboRosIRReceiver::IRReceiverQueueThread()
{
  ros::Rate rate(update_rate_);

  while (nh_->ok())
  {
    ir_receiver_queue_.callAvailable();
    rate.sleep();
  }
}
}

