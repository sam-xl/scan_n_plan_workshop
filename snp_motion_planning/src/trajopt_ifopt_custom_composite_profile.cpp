/**
 * @file trajopt_custom_composite_profile.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/utils.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "trajopt_ifopt_custom_composite_profile.h"
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_collision/core/serialization.h>

namespace tesseract_planning
{

template <class Archive>
void TrajOptIfoptCustomCompositeProfile::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TrajOptIfoptDefaultCompositeProfile);
  ar& BOOST_SERIALIZATION_NVP(collision_cost_config);
  ar& BOOST_SERIALIZATION_NVP(collision_constraint_config);
  ar& BOOST_SERIALIZATION_NVP(smooth_velocities);
  ar& BOOST_SERIALIZATION_NVP(velocity_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_accelerations);
  ar& BOOST_SERIALIZATION_NVP(acceleration_coeff);
  ar& BOOST_SERIALIZATION_NVP(smooth_jerks);
  ar& BOOST_SERIALIZATION_NVP(jerk_coeff);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_fraction);
  ar& BOOST_SERIALIZATION_NVP(longest_valid_segment_length);
  ar& BOOST_SERIALIZATION_NVP(special_collision_constraint);
  ar& BOOST_SERIALIZATION_NVP(special_collision_cost);
  ar& BOOST_SERIALIZATION_NVP(contact_test_type);
}

}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::TrajOptIfoptCustomCompositeProfile)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::TrajOptIfoptCustomCompositeProfile)
