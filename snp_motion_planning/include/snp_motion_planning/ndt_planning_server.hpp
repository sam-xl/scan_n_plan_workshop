#include "planner_profiles.hpp"
#include "plugins/tasks/constant_tcp_speed_time_parameterization_profile.h"
#include "plugins/tasks/kinematic_limits_check_profile.h"
#include "plugins/tasks/tcp_speed_limiter_profile.h"

#include <rclcpp/rclcpp.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/srv/generate_freespace_motion_plan.hpp>
#include <snp_motion_planning/planning_server.hpp>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_environment/environment.h>

class NDTPlanningServer: public PlanningServer
{
public:
  NDTPlanningServer(rclcpp::Node::SharedPtr node)
    : PlanningServer(node), node_(node)
  {}

private:

  tesseract_planning::CompositeInstruction createRasterProgram(const tesseract_common::ManipulatorInfo& info,
                                                         const tesseract_common::Toolpath& raster_strips){
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    tesseract_planning::CompositeInstruction program(PROFILE, info);

    // Define the current state
    tesseract_planning::StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));
    program.push_back(tesseract_planning::MoveInstruction(current_state, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info));

    // Add the process raster motions
    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      // Add raster
      tesseract_planning::CompositeInstruction raster_segment(PROFILE);
      raster_segment.setDescription("Raster Index " + std::to_string(rs));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
        raster_segment.push_back(
            tesseract_planning::MoveInstruction(wp, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info));
      }
      program.push_back(raster_segment);

      // Add transition
      if (rs < raster_strips.size() - 1)
      {
        tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

        tesseract_planning::MoveInstruction transition_instruction1(
            twp, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info);
        transition_instruction1.setDescription("Transition #" + std::to_string(rs + 1));

        tesseract_planning::CompositeInstruction transition(PROFILE);
        transition.setDescription("Transition #" + std::to_string(rs + 1));
        transition.push_back(transition_instruction1);

        program.push_back(transition);
      }
    }

    program.print();

    return program;
  }

  void processMotionPlanCallback(const snp_msgs::srv::GenerateMotionPlan::Request::SharedPtr req,
                                 snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr res) override
  {
     try
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Received motion planning request");

      // Create a manipulator info and program from the service request
      const std::string& base_frame = req->tool_paths.at(0).segments.at(0).header.frame_id;
      if (base_frame.empty())
      {
        throw std::runtime_error("Base frame is empty!");
      }
      if (req->motion_group.empty())
      {
        throw std::runtime_error("Motion group is empty!");
      }
      if (req->tcp_frame.empty())
      {
        throw std::runtime_error("TCP frame is empty!");
      }
      tesseract_common::ManipulatorInfo manip_info(req->motion_group, base_frame, req->tcp_frame);

      // Set up composite instruction and environment
      tesseract_planning::CompositeInstruction program = createRasterProgram(manip_info, fromMsg(req->tool_paths));

      // Invoke the planner
      auto pd = createProfileDictionary();
      auto raster_task_name = get<std::string>(node_, RASTER_TASK_NAME_PARAM);
      tesseract_planning::CompositeInstruction program_results = plan(program, pd, raster_task_name);


      tesseract_planning::CompositeInstruction process_ci(program_results.begin(), program_results.end());
      res->process = tesseract_rosutils::toMsg(toJointTrajectory(process_ci), env_->getState());

      res->message = "Succesfully planned motion";
      res->success = true;
    }
    catch (const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), res->message);
  }

  rclcpp::Node::SharedPtr node_;

};
