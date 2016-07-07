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

#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>

#include <gazebo/common/common.hh>

namespace gazebo {

class DipoleMagnetContainer {
 public:
  DipoleMagnetContainer() {
  }

  static DipoleMagnetContainer& Get() {
    static DipoleMagnetContainer instance;
    return instance;
  }

  struct Magnet {
    bool calculate;
    math::Vector3 moment;
    math::Pose offset;
    math::Pose pose;
    std::uint32_t model_id;
  };

  typedef std::shared_ptr<Magnet> MagnetPtr ;
  typedef std::vector<MagnetPtr> MagnetPtrV ;

  void Add(MagnetPtr mag) {
    std::cout << "Adding mag id:" << mag->model_id << std::endl;
    this->magnets.push_back(mag);
    std::cout << "Total: " << this->magnets.size() << " magnets" << std::endl;
  }
  void Remove(MagnetPtr mag) {
    std::cout << "Removing mag id:" << mag->model_id << std::endl;
    this->magnets.erase(std::remove(this->magnets.begin(), this->magnets.end(), mag), this->magnets.end());
    std::cout << "Total: " << this->magnets.size() << " magnets" << std::endl;
  }

  MagnetPtrV magnets;
};
}  // namespace gazebo

#endif  // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_
