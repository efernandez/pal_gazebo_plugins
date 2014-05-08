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

#include <kobuki_msgs/DockInfraRed.h>

#include <string>

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosIRReceiver)

GazeboRosIRReceiver::GazeboRosIRReceiver()
  : SensorPlugin()
  , rosNode()
{}

GazeboRosIRReceiver::~GazeboRosIRReceiver()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

void GazeboRosIRReceiver::Load(sensors::SensorPtr _sensor,
                                 sdf::ElementPtr _sdf)
{
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->topic_name_ = "ir_receiver";
  if (_sdf->GetElement("topicName"))
    this->topic_name_ =
      _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("IR reciever sensor plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  this->update_rate_ = 100.0;
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("IR reciever sensor plugin missing <updateRate>, defaults to %f", this->update_rate_);
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Get sensor
  this->sensor = _sensor;
  if (!this->sensor)
    gzerr << "sensor not found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&GazeboRosIRReceiver::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosIRReceiver::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->pub =
    this->rosNode->advertise<kobuki_msgs::DockInfraRed>("/" + this->robot_namespace_+ "/" + this->topic_name_, 10);

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind(&GazeboRosIRReceiver::RosQueueThread, this));

  // Connect to the sensor update event.
  this->updateConnection = this->sensor->ConnectUpdated(
              boost::bind(&GazeboRosIRReceiver::UpdateChild, this));

  // Make sure the parent sensor is active.
  this->sensor->SetActive(true);

  this->lastUpdateTime = ros::Time::now();
}

void GazeboRosIRReceiver::UpdateChild()
{
  if (this->pub.getNumSubscribers() <= 0)
    return;

  boost::mutex::scoped_lock sclock(this->mutex_);

  static const ros::Duration update_period(1.0/this->update_rate_);

  ros::Time now = ros::Time::now();
  if (( now - this->lastUpdateTime) >= update_period)
  {
    this->lastUpdateTime = now;

    // @todo logic for the IRs
    kobuki_msgs::DockInfraRed msg;
    msg.header.frame_id = this->frame_name_;
    msg.header.stamp = now;

    msg.data.clear();
    msg.data.push_back(1);
    msg.data.push_back(2);
    msg.data.push_back(3);

    this->pub.publish(msg);
  }
}

void GazeboRosIRReceiver::RosQueueThread()
{
  ros::Rate rate(this->update_rate_);

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable();
    rate.sleep();
  }
}
}

