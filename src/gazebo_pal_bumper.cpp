/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include <pal_gazebo_plugins/gazebo_pal_bumper.h>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/Collision.hh>

#include "math/Pose.hh"
#include "math/Quaternion.hh"
#include "math/Vector3.hh"

#include "tf/tf.h"

namespace gazebo
{

GazeboPalBumper::GazeboPalBumper()
  : rosnode_()
{
  this->contact_connect_count_ = 0;
}

GazeboPalBumper::~GazeboPalBumper()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

void GazeboPalBumper::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO_STREAM("Loading gazebo bumper");

    sensor_ = boost::shared_dynamic_cast<sensors::ContactSensor>(_parent);
    if (!sensor_)
    {
        gzthrow("GazeboRosSonar requires a Contact Sensor as its parent");
        return;
    }

    // Get the world name.
    std::string worldName = sensor_->GetWorldName();
    parent_ = gazebo::physics::get_world(worldName);

  //  ContactPlugin::Load(_parent, _sdf);

    this->robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
        this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

    // "publishing contact/collisions to this topic name: " << this->bumper_topic_name_ << std::endl;
    this->bumper_topic_name_ = "bumper_base";
    if (_sdf->GetElement("bumperTopicName"))
        this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->Get<std::string>();

    // "transform contact/collisions pose, forces to this body (link) name: " << this->frame_name_ << std::endl;
    if (!_sdf->HasElement("frameName"))
    {
        ROS_INFO("bumper plugin missing <frameName>, defaults to world");
        this->frame_name_ = "world";
    }
    else
        this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

    ROS_INFO("Loaded with values:   robotNamespace = %s, bumperTopicName = %s, frameName = %s",
             this->robot_namespace_.c_str(), this->bumper_topic_name_.c_str(),this->frame_name_.c_str());

    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // resolve tf prefix
    std::string prefix;
    this->rosnode_->getParam(std::string("tf_prefix"), prefix);
    this->frame_name_ = tf::resolve(prefix, this->frame_name_);

    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<pal_device_msgs::Bumper>(this->bumper_topic_name_, 1,
                                                                                boost::bind( &GazeboPalBumper::ContactConnect,this),
                                                                                boost::bind( &GazeboPalBumper::ContactDisconnect,this),
                                                                                ros::VoidPtr(), &this->contact_queue_);
    this->contact_pub_ = this->rosnode_->advertise(ao);

    // Initialize
    // preset myFrame to NULL, will search for the body with matching name in UpdateChild()
    // since most bodies are constructed on the fly
    //this->myFrame = NULL;
    // start custom queue for contact bumper

    this->callback_queue_thread_ = boost::thread(boost::bind( &GazeboPalBumper::ContactQueueThread,this ) );

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPalBumper::Update, this));

    ROS_INFO("loaded bumper");
}

void GazeboPalBumper::ContactConnect()
{
  this->contact_connect_count_++;
}

void GazeboPalBumper::ContactDisconnect()
{
  this->contact_connect_count_--;
}

void GazeboPalBumper::Update()
{
  if (this->contact_connect_count_ <= 0)
    return;

 // boost::mutex::scoped_lock sclock(*this->parentSensor->GetUpdateMutex());

  std::map<std::string, physics::Contact> contacts;
  math::Vector3 body1ContactForce, body2ContactForce;
  common::Time cur_time;

  cur_time = parent_->GetSimTime();

  pal_device_msgs::Bumper bumperMsg;
  bumperMsg.header.stamp = ros::Time::now();
  bumperMsg.header.frame_id = this->frame_name_;

  // information are in inertial coordinates
  //this->contact_state_msg_.header.frame_id = this->frame_name_;
  //this->contact_state_msg_.header.stamp.sec = cur_time.sec;
  //this->contact_state_msg_.header.stamp.nsec = cur_time.nsec;

  // set contact states size
  this->contact_state_msg_.states.clear();  // one contact_count per pair of geoms in contact (up to 64 contact points per pair of geoms)

  // For each collision that the sensor is monitoring
  for (unsigned int i=0; i < sensor_->GetCollisionCount();++i)
  {
    int l = 0;
    std::string collisionName = sensor_->GetCollisionName(i);
    contacts = sensor_->GetContacts(collisionName);

    math::Pose pose, frame_pose;
    math::Quaternion rot, frame_rot;
    math::Vector3 pos, frame_pos;

    {
      // no specific frames specified, use identity pose, keeping
      // relative frame at inertial origin
      frame_pos = math::Vector3(0,0,0);
      frame_rot = math::Quaternion(1,0,0,0); // gazebo u,x,y,z == identity
      frame_pose = math::Pose(frame_pos, frame_rot);
    }

    // For each collision contact
    for (std::map<std::string, gazebo::physics::Contact>::iterator citer =
         contacts.begin(); citer != contacts.end() ; ++citer)
    {
      gazebo::physics::Contact const &contact = citer->second;

      // For each geom-geom contact
      unsigned int pts = contact.count;

      std::ostringstream stream;
      stream    << "touched!    i:" << l
        << "      my geom:" << contact.collision1
        << "   other geom:" << contact.collision2
        << "         time:" << contact.time
        << std::endl;

      gazebo_msgs::ContactState state;
      state.info = stream.str();
      state.collision1_name = contact.collision1->GetName();
      state.collision2_name = contact.collision2->GetName();

      state.wrenches.clear();
      state.contact_positions.clear();
      state.contact_normals.clear();
      state.depths.clear();

      // sum up all wrenches for each DOF
      geometry_msgs::Wrench total_wrench;
      total_wrench.force.x = 0;
      total_wrench.force.y = 0;
      total_wrench.force.z = 0;
      total_wrench.torque.x = 0;
      total_wrench.torque.y = 0;
      total_wrench.torque.z = 0;

      for (unsigned int k=0; k < pts; k++)
      {
        // rotate into user specified frame.
        // frame_rot is identity if world is used.
        math::Vector3 force = frame_rot.RotateVectorReverse(
            math::Vector3(contact.wrench[k].body1Force.x,
              contact.wrench[k].body1Force.y,
              contact.wrench[k].body1Force.z));
        math::Vector3 torque = frame_rot.RotateVectorReverse(
            math::Vector3(contact.wrench[k].body1Torque.x,
              contact.wrench[k].body1Torque.y,
              contact.wrench[k].body1Torque.z));

        // set wrenches
        geometry_msgs::Wrench wrench;
        wrench.force.x  = force.x;
        wrench.force.y  = force.y;
        wrench.force.z  = force.z;
        wrench.torque.x = torque.x;
        wrench.torque.y = torque.y;
        wrench.torque.z = torque.z;
        state.wrenches.push_back(wrench);
        total_wrench.force.x  += wrench.force.x ;
        total_wrench.force.y  += wrench.force.y ;
        total_wrench.force.z  += wrench.force.z ;
        total_wrench.torque.x += wrench.torque.x;
        total_wrench.torque.y += wrench.torque.y;
        total_wrench.torque.z += wrench.torque.z;

        // transform contact positions into relative frame
        // set contact positions
        gazebo::math::Vector3 contact_position;
        contact_position = contact.positions[k] - frame_pos;
        contact_position = frame_rot.RotateVectorReverse(contact_position);
        geometry_msgs::Vector3 tmp;
        tmp.x = contact_position.x;
        tmp.y = contact_position.y;
        tmp.z = contact_position.z;
        state.contact_positions.push_back(tmp);

        // rotate normal into user specified frame.
        // frame_rot is identity if world is used.
        math::Vector3 normal = frame_rot.RotateVectorReverse(
            math::Vector3(contact.normals[k].x,
              contact.normals[k].y,
              contact.normals[k].z));
        // set contact normals
        geometry_msgs::Vector3 contact_normal;
        contact_normal.x = normal.x;
        contact_normal.y = normal.y;
        contact_normal.z = normal.z;
        state.contact_normals.push_back(contact_normal);

        // set contact depth, interpenetration
        state.depths.push_back(contact.depths[k]);
      }
      state.total_wrench = total_wrench;
      this->contact_state_msg_.states.push_back(state);

      bumperMsg.is_pressed = !state.contact_positions.empty();
      if (bumperMsg.is_pressed)
      {
        std::cout << "PRESSED " << bumperMsg.header.frame_id << "!!!" << std::endl;
        std::cout << "\n\n\n" << std::endl;
      }
    }
  }

  this->contact_pub_.publish(bumperMsg);
}


void GazeboPalBumper::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboPalBumper)

}
