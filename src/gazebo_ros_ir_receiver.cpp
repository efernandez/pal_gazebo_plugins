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

#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <pal_gazebo_plugins/gazebo_ros_ir_receiver.h>

#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <kobuki_msgs/DockInfraRed.h>

#include <tf/tf.h>

#include "xmlrpc_helpers.h"

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

  range_min_ = 0.0;
  if (!sdf->HasElement("minRange"))
    ROS_WARN_STREAM("IR recevier sensor plugin missing <minRange>, defaults to " << range_min_);
  else
    range_min_ = sdf->GetElement("minRange")->Get<double>();

  range_max_ = 100.0;
  if (!sdf->HasElement("maxRange"))
    ROS_WARN_STREAM("IR recevier sensor plugin missing <maxRange>, defaults to " << range_max_);
  else
    range_max_ = sdf->GetElement("maxRange")->Get<double>();

  fov_ = M_PI;
  if (!sdf->HasElement("fov"))
    ROS_WARN_STREAM("IR recevier sensor plugin missing <fov>, defaults to " << fov_);
  else
    fov_ = sdf->GetElement("fov")->Get<double>();

  // Get sensor
  sensor_ = parent;
  if (!sensor_)
    ROS_ERROR_STREAM("Sensor not found");

  // Get the world name
  const std::string world_name = sensor_->GetWorldName();
  world_ = physics::get_world(world_name);

  last_update_time_ = world_->GetSimTime();

  // Sensor relative pose wrt to its parent/robot
  sensor_pose_ = sensor_->GetPose();

  // Get parent/robot entity
  std::string robot_name = sensor_->GetParentName();
  robot_ = world_->GetEntity(robot_name);
  if (robot_ == NULL)
  {
    ROS_FATAL_STREAM("Couldn't find robot/parent entity " << robot_name);
    return;
  }

  // ------------- @test
  transport::PublisherPtr visPub;
  msgs::Visual visualMsg;

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init(world_name);
  visPub = node->Advertise<msgs::Visual>("~/visual", 10);

  visualMsg.set_name("__RED_CYLINDER_VISUAL__");

  msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::CYLINDER);
  geomMsg->mutable_cylinder()->set_radius(1);
  geomMsg->mutable_cylinder()->set_length(.1);

  //visualMsg.mutable_material()->set_script("Gazebo/RedGlow");

  msgs::Set(visualMsg.mutable_pose(), math::Pose(0, 0, 0.6, 0, 0, 0));

  visualMsg.set_cast_shadows(false);

  visPub->Publish(visualMsg);
  // -------------------

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

  // Load IR emitters
  // @todo this shouldn't be called for every single instance,
  // it should be called only once!!!
  try
  {
    xh::Array output;
    xh::fetchParam(*nh_, "ir_emitters", output);

    xh::Struct output_i;
    std::string name;
    int code;
    double fov, range;
    xh::Struct pose;
    double x, y, z, yaw;
    for (int i = 0; i < output.size(); ++i)
    {
      xh::getArrayItem(output, i, output_i);

      xh::getStructMember(output_i, "name", name);
      xh::getStructMember(output_i, "code", code);
      xh::getStructMember(output_i, "fov", fov);
      xh::getStructMember(output_i, "range", range);
      xh::getStructMember(output_i, "pose", pose);
      xh::getStructMember(pose, "x", x);
      xh::getStructMember(pose, "y", y);
      xh::getStructMember(pose, "z", z);
      xh::getStructMember(pose, "yaw", yaw);

      ir_emitters_.emplace_back(name, x, y, z, yaw, code, fov, range);
      ROS_INFO_STREAM("Add IR emitter: " << ir_emitters_.back());
    }
  }
  catch (const xh::XmlrpcHelperException& e)
  {
    ROS_WARN_STREAM("There is no IR emitter; "
        "the IR receivers will not detect anything.");
  }

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
    last_update_time_ = now;

    // Get robot absolute pose wrt the world, and
    // compute sensor pose wrt the world
    math::Pose robot_pose  = robot_->GetWorldPose();
    math::Pose sensor_pose = robot_pose * sensor_pose_;
    ROS_ERROR_STREAM_THROTTLE(0.1, "Robot pose = " << robot_pose << "; Sensor pose = " << sensor_pose);


    kobuki_msgs::DockInfraRed msg;
    msg.header.frame_id = frame_name_;
    msg.header.stamp.sec = now.sec;
    msg.header.stamp.nsec = now.nsec;

    msg.data.clear();

    // Check which IR emitter are detected
    foreach (auto ir_emitter, ir_emitters_)
    {
      //ROS_ERROR_STREAM("Checking " << ir_emitter.getName() << " for " << frame_name_);
      if (ir_emitter.isInRange(sensor_pose))
      {
        const int code = ir_emitter.getCode();
        //ROS_ERROR_STREAM("code " << code << " detected by " << frame_name_);
        msg.data.push_back(code);
      }
    }

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

