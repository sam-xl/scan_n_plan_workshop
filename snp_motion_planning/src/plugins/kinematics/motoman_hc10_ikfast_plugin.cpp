/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
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
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <tesseract/kinematics/ikfast/impl/ikfast_inv_kin.hpp>
#include "motoman_hc10_ikfast_solver.hpp"
#include <tesseract/kinematics/ikfast/ikfast_factory_boilerplate.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::IKFastInvKinFactory, MotomanHC10InvKinFactory)
