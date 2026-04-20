/**
 * @file trajopt_default_composite_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CUSTOM_COMPOSITE_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CUSTOM_COMPOSITE_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <trajopt/fwd.hpp>
#include <trajopt_common/fwd.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>

#include <tesseract_collision/core/fwd.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_planning
{
class TrajOptIfoptCustomCompositeProfile : public TrajOptIfoptDefaultCompositeProfile
{
public:
  TrajOptIfoptCustomCompositeProfile() = default;

  /* The costs were added from the TrajOptDefaultCompositeProfile. To run the scan_n_plan workshop with
   the default parameters they have used, these parameters are needed. They can be removed if it is not used.
  */
  tesseract_collision::ContactTestType contact_test_type{ tesseract_collision::ContactTestType::ALL };
  CollisionCostConfig collision_cost_config;
  CollisionConstraintConfig collision_constraint_config;

  /**@brief Special link collision cost distances */
  std::shared_ptr<trajopt_common::SafetyMarginData> special_collision_cost{ nullptr };
  /**@brief Special link collision constraint distances */
  std::shared_ptr<trajopt_common::SafetyMarginData> special_collision_constraint{ nullptr };

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptIfoptCustomCompositeProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H
