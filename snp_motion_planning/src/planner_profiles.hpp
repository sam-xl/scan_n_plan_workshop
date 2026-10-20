#pragma once

#include <thread>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract/motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>
#include <tesseract/motion_planners/descartes/cereal_serialization.h>
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/ompl/cereal_serialization.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/cereal_serialization.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/cereal_serialization.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract/motion_planners/simple/cereal_serialization.h>
#include <tesseract/task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract/task_composer/planning/cereal_serialization.h>
#include <tesseract/time_parameterization/kdl/cereal_serialization.h>
#include <tesseract/time_parameterization/isp/cereal_serialization.h>

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_CUSTOM_NAMESPACE = "CustomTrajOptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MIN_LENGTH_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string CONTACT_CHECK_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";

/**
 * @brief Container for a collision pair with a specifiable minimum allowable contact distance between the link pair
 */
struct ExplicitCollisionPair
{
  explicit ExplicitCollisionPair(std::string first_, std::string second_, double distance_)
    : first(first_), second(second_), distance(distance_)
  {
  }

  /** @brief Name of the first link */
  std::string first;
  /** @brief Name of the second link */
  std::string second;
  /** @brief Minimum allowable contact distance between the two links */
  double distance;
};

template <typename FloatType>
typename tesseract::motion_planners::DescartesLadderGraphSolverProfile<FloatType>::Ptr createDescartesSolverProfile()
{
  auto profile = std::make_shared<tesseract::motion_planners::DescartesLadderGraphSolverProfile<FloatType>>();
  profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
  return profile;
}

template <typename FloatType>
typename tesseract::motion_planners::DescartesDefaultMoveProfile<FloatType>::Ptr
createDescartesPlanProfile(FloatType min_contact_distance,
                           const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                           const FloatType longest_valid_segment_length)
{
  auto profile = std::make_shared<tesseract::motion_planners::DescartesDefaultMoveProfile<FloatType>>();
  profile->use_redundant_joint_solutions = false;

  // Tool pose sampler
  profile->target_pose_fixed = false;
  profile->target_pose_sample_axis = Eigen::Vector3d::UnitZ();
  profile->target_pose_sample_resolution = 10.0 * M_PI / 180.0;
  profile->target_pose_sample_min = -M_PI;
  profile->target_pose_sample_max = M_PI - profile->target_pose_sample_resolution;

  // Collision checking
  profile->allow_collision = false;
  profile->enable_collision = true;
  profile->vertex_collision_check_config.contact_request.type = tesseract::collision::ContactTestType::FIRST;
  profile->vertex_contact_manager_config.default_margin = min_contact_distance;
  profile->vertex_collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  profile->vertex_contact_manager_config.pair_margin_override_type =
      tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->vertex_contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);

  profile->enable_edge_collision = false;
  profile->edge_collision_check_config.contact_request.type = tesseract::collision::ContactTestType::FIRST;
  profile->edge_contact_manager_config.default_margin = min_contact_distance;
  profile->edge_collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  profile->edge_contact_manager_config.pair_margin_override_type =
      tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->edge_contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);

  return profile;
}

tesseract::motion_planners::OMPLRealVectorMoveProfile::Ptr
createOMPLProfile(const double min_contact_distance, const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                  const double longest_valid_segment_length)
{
  // OMPL freespace and transition profiles
  // Create the RRT parameters
  auto n = static_cast<Eigen::Index>(std::thread::hardware_concurrency());
  auto range = Eigen::VectorXd::LinSpaced(n, 0.05, 0.5);

  // Add as many planners as available threads so mulitple OMPL plans can happen in parallel
  auto profile = std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>();
  profile->solver_config.planning_time = 5.0;
  profile->solver_config.max_solutions = 1;

  profile->solver_config.planners.clear();
  profile->solver_config.planners.reserve(static_cast<std::size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i)
  {
    auto rrt = std::make_shared<tesseract::motion_planners::RRTConnectConfigurator>();
    rrt->range = range(i);
    profile->solver_config.planners.push_back(rrt);
  }

  // Collision checking
  profile->collision_check_config.contact_request.type = tesseract::collision::ContactTestType::FIRST;
  profile->contact_manager_config.default_margin = min_contact_distance;
  profile->contact_manager_config.pair_margin_override_type =
      tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  profile->collision_check_config.longest_valid_segment_length = longest_valid_segment_length;

  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);

  return profile;
}

std::shared_ptr<tesseract::motion_planners::TrajOptMoveProfile>
createTrajOptToolZFreePlanProfile(const Eigen::VectorXd& cart_tolerance = Eigen::VectorXd::Zero(6),
                                  const Eigen::VectorXd& cart_coeff = Eigen::VectorXd::Constant(6, 1, 2.5))
{
  auto profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();

  profile->cartesian_cost_config.enabled = true;
  profile->cartesian_cost_config.use_tolerance_override = true;
  profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.coeff = cart_coeff;

  profile->cartesian_constraint_config.enabled = true;
  profile->cartesian_constraint_config.use_tolerance_override = true;
  profile->cartesian_constraint_config.lower_tolerance = -1 * cart_tolerance;
  profile->cartesian_constraint_config.upper_tolerance = cart_tolerance;
  profile->cartesian_constraint_config.coeff = cart_coeff;

  profile->joint_cost_config.enabled = false;
  profile->joint_cost_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  profile->joint_constraint_config.enabled = true;
  profile->joint_constraint_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  return profile;
}

/**
 * @brief Configures the collision cost and constraint of a TrajOpt composite profile (TrajOpt or TrajOpt IFOPT)
 *
 * Safety margin notes:
 *   - A non-zero cost for distance to collision is computed only when the distance to collision < safety margin
 *   - The gradient of the collision cost is computed when the distance to collision is < safety margin + safety
 * margin buffer
 *   - When safety margin + safety margin buffer > distance to collision > safety margin, the gradient of the cost is
 * computed to prevent optimization instability, but the computed collision cost will be zero
 *
 * Therefore, we should set the safety margin relatively high such that non-zero collision costs are computed and
 * drive the robot away from collision. We can make the safety margin buffer a fixed size (usually 2cm is appropriate)
 * as the safety margin to ensure that the optimization does not move in the direction of collision even when the cost
 * is zero.
 */
template <typename ProfileT>
void configureTrajOptCollision(ProfileT& profile, double min_contact_distance,
                               const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                               double longest_valid_segment_length)
{
  profile.smooth_velocities = false;
  profile.velocity_coeff = Eigen::VectorXd::Constant(1, 1, 10.0);
  profile.smooth_accelerations = true;
  profile.acceleration_coeff = Eigen::VectorXd::Constant(1, 1, 25.0);
  profile.smooth_jerks = true;
  profile.jerk_coeff = Eigen::VectorXd::Constant(1, 1, 50.0);

  // Collision cost
  auto& cost = profile.collision_cost_config;
  cost.enabled = true;
  cost.collision_check_config.contact_request.type = tesseract::collision::ContactTestType::CLOSEST;
  cost.collision_check_config.type = tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE;
  cost.collision_check_config.longest_valid_segment_length = longest_valid_segment_length;
  cost.collision_coeff_data = trajopt_common::CollisionCoeffData(10.0);
  // Use a relatively large safety margin to drive the robot away from collision
  cost.contact_manager_config.default_margin = std::max(0.025, min_contact_distance);
  // Use a small, fixed-size buffer beyond the defined margin to continue calculating collision avoidance gradients
  // even when the collision avoidance cost is zero
  cost.collision_margin_buffer = 0.025;
  cost.contact_manager_config.pair_margin_override_type = tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
  {
    cost.contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);
    cost.collision_coeff_data.setCollisionCoeff(pair.first, pair.second, 10.0);
  }

  // Collision constraint
  auto& constraint = profile.collision_constraint_config;
  constraint.enabled = true;
  constraint.collision_check_config.contact_request.type = tesseract::collision::ContactTestType::CLOSEST;
  constraint.collision_check_config.type = tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE;
  constraint.collision_check_config.longest_valid_segment_length = longest_valid_segment_length;
  constraint.collision_coeff_data = trajopt_common::CollisionCoeffData(10.0);
  // Use a minimal safety margin, such that the constraint is only violated (and therefore a cost produced) when the
  // robot is actually considered to be in collision
  constraint.contact_manager_config.default_margin = min_contact_distance;
  constraint.collision_margin_buffer = 0.025;
  constraint.contact_manager_config.pair_margin_override_type =
      tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
  {
    constraint.contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);
    constraint.collision_coeff_data.setCollisionCoeff(pair.first, pair.second, 10.0);
  }
}

std::shared_ptr<tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile> createCustomTrajOptProfile(
    double min_contact_distance, const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
    double longest_valid_segment_length)
{
  auto profile = std::make_shared<tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile>();
  configureTrajOptCollision(*profile, min_contact_distance, unique_collision_pairs, longest_valid_segment_length);
  return profile;
}

std::shared_ptr<tesseract::motion_planners::TrajOptDefaultCompositeProfile>
createTrajOptProfile(double min_contact_distance, const std::vector<ExplicitCollisionPair>& unique_collision_pairs,
                     double longest_valid_segment_length)
{
  auto profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
  configureTrajOptCollision(*profile, min_contact_distance, unique_collision_pairs, longest_valid_segment_length);
  return profile;
}

std::shared_ptr<tesseract::motion_planners::SimplePlannerLVSMoveProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract::motion_planners::SimplePlannerLVSMoveProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180,
                                                                                   1);
}

tesseract::task_composer::ContactCheckProfile::Ptr
createContactCheckProfile(double longest_valid_segment_distance, double min_contact_distance,
                          const std::vector<ExplicitCollisionPair>& collision_pairs)
{
  auto profile = std::make_shared<tesseract::task_composer::ContactCheckProfile>(longest_valid_segment_distance,
                                                                                 min_contact_distance);
  profile->collision_check_config.contact_request.type = tesseract::collision::ContactTestType::FIRST;
  profile->contact_manager_config.pair_margin_override_type =
      tesseract::common::CollisionMarginPairOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : collision_pairs)
    profile->contact_manager_config.pair_margin_data.setCollisionMargin(pair.first, pair.second, pair.distance);

  return profile;
}
