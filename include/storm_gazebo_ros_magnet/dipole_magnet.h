/*
 * Copyright (c) 2016, Vanderbilt University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Addisu Z. Taddese
 */

#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <memory>

#include "storm_gazebo_ros_magnet/dipole_magnet_container.h"

namespace gazebo {

class DipoleMagnet : public ModelPlugin {
 public:
  DipoleMagnet();

  ~DipoleMagnet();

  /// \brief Loads the plugin
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for when subscribers connect
  void Connect();

  /// \brief Callback for when subscribers disconnect
  void Disconnect();

  /// \brief Thread to interact with ROS
  void QueueThread();

  /// \brief Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/);


  /// \brief Publishes data to ros topics
  /// \pram[in] force A vector of force that makes up the wrench to be published
  /// \pram[in] torque A vector of torque that makes up the wrench to be published
  /// \pram[in] mfs A vector of magnetic field data
  void PublishData(
      const math::Vector3& force, 
      const math::Vector3& torque,
      const math::Vector3& mfs);

  /// \brief Calculate force and torque of a magnet on another
  /// \parama[in] p_self Pose of the first magnet
  /// \parama[in] m_self Dipole moment of the first magnet
  /// \parama[in] p_other Pose of the second magnet
  /// \parama[in] m_other Dipole moment of the second magnet on which the force is calculated
  /// \param[out] force Calculated force vector
  /// \param[out] torque Calculated torque vector
  void GetForceTorque(const math::Pose& p_self,  const math::Vector3& m_self,
      const math::Pose& p_other, const math::Vector3& m_other,
      math::Vector3& force, math::Vector3& torque);

  /// \brief Calculate the magnetic field on all 6 sensors
  /// \parama[in] p_self Pose of the first magnet
  /// \parama[in] p_other Pose of the second magnet
  /// \parama[in] m_other Dipole moment of the second magnet
  /// \param[out] mfs magnetic field sensors
  void GetMFS(const math::Pose& p_self,
      const math::Pose& p_other,
      const math::Vector3& m_other,
      math::Vector3& mfs);

  // Pointer to the model
 private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  physics::WorldPtr world;

  std::shared_ptr<DipoleMagnetContainer::Magnet> mag;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool should_publish;
  ros::NodeHandle* rosnode;
  ros::Publisher wrench_pub;
  ros::Publisher mfs_pub;

  geometry_msgs::WrenchStamped wrench_msg;
  sensor_msgs::MagneticField mfs_msg;

  private: boost::mutex lock;
  int connect_count;

  // Custom Callback Queue
  ros::CallbackQueue queue;
  boost::thread callback_queue_thread;

  common::Time last_time;
  double update_rate;
  // Pointer to the update event connection
  event::ConnectionPtr update_connection;
};

}
#endif  // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
