/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_BASE_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_BASE_FILTER_H_

#include <memory>
#include <string>

#include "Eigen/Core"
#include "../object.h"
#include "tracked_object.h"

namespace apollo {
namespace perception {

class BaseFilter {
 public:
  typedef Object ObjectType;

  BaseFilter() { name_ = "BaseFilter"; }
  virtual ~BaseFilter() {}

  // @brief initialize the state of filter
  // @param[IN] anchor_point: initial anchor point for filtering
  // @param[IN] velocity: initial velocity for filtering
  // @return nothing
  virtual void Initialize(const Eigen::Vector3f& anchor_point,
                          const Eigen::Vector3f& velocity) = 0;

  // @brief predict the state of filter
  // @param[IN] time_diff: time interval for predicting
  // @return predicted states of filtering
  virtual Eigen::VectorXf Predict(const double time_diff) = 0;

  // @brief update filter with object
  // @param[IN] new_object: recently detected object for current updating
  // @param[IN] old_object: last detected object for last updating
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  virtual void UpdateWithObject(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double time_diff) = 0;

  // @brief update filter without object
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  virtual void UpdateWithoutObject(const double time_diff) = 0;

  // @brief get state of filter
  // @param[OUT] anchor_point: anchor point of current state
  // @param[OUT] velocity: velocity of current state
  // @return nothing
  virtual void GetState(Eigen::Vector3f* anchor_point,
                        Eigen::Vector3f* velocity) = 0;

  // @brief get state of filter with acceleration
  // @param[OUT] anchor_point: anchor point of current state
  // @param[OUT] velocity: velocity of current state
  // @param[OUT] velocity_acceleration: acceleration of current state
  // @return nothing
  virtual void GetState(Eigen::Vector3f* anchor_point,
                        Eigen::Vector3f* velocity,
                        Eigen::Vector3f* velocity_acceleration) = 0;

  virtual void GetAccelerationGain(Eigen::Vector3f* acceleration_gain) = 0;

  // @brief get online covariance of filter
  // @param[OUT] online_covariance: online covariance
  // @return noting
  virtual void GetOnlineCovariance(Eigen::Matrix3f* online_covariance) = 0;

  // @brief get name of filter
  // @return name of filter
  std::string Name() { return name_; }

 protected:
  std::string name_;
};  // class BaseFilter

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_BASE_FILTER_H_
