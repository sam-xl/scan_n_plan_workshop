#include "tcp_speed_limiter_profile.h"
#include "tcp_speed_limiter.hpp"

#include <tesseract/common/profile_dictionary.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/environment/environment.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>

#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
#include <tesseract/task_composer/task_composer_task.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_plugin_factory_utils.h>
#include <tesseract/task_composer/task_composer_task_plugin_factory.h>

namespace snp_motion_planning
{
class TCPSpeedLimiterTask : public tesseract::task_composer::TaskComposerTask
{
public:
  // Required
  static const std::string INOUT_PROGRAM_PORT;
  static const std::string INPUT_ENVIRONMENT_PORT;
  static const std::string INPUT_PROFILES_PORT;

  using Ptr = std::shared_ptr<TCPSpeedLimiterTask>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterTask>;
  using UPtr = std::unique_ptr<TCPSpeedLimiterTask>;
  using ConstUPtr = std::unique_ptr<const TCPSpeedLimiterTask>;

  TCPSpeedLimiterTask() : tesseract::task_composer::TaskComposerTask(TCP_SPEED_LIMITER_TASK_NAME, ports(), true)
  {
  }

  explicit TCPSpeedLimiterTask(std::string name, std::string input_program_key, std::string input_environment_key,
                               std::string input_profiles_key, std::string output_program_key,
                               bool is_conditional = true)
    : tesseract::task_composer::TaskComposerTask(std::move(name), ports(), is_conditional)
  {
    input_keys_.add(INOUT_PROGRAM_PORT, std::move(input_program_key));
    input_keys_.add(INPUT_ENVIRONMENT_PORT, std::move(input_environment_key));
    input_keys_.add(INPUT_PROFILES_PORT, std::move(input_profiles_key));
    output_keys_.add(INOUT_PROGRAM_PORT, std::move(output_program_key));
    validatePorts();
  }

  explicit TCPSpeedLimiterTask(std::string name, const YAML::Node& config,
                               const tesseract::task_composer::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract::task_composer::TaskComposerTask(std::move(name), ports(), config)
  {
    validatePorts();
  }

  ~TCPSpeedLimiterTask() override = default;
  TCPSpeedLimiterTask(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask& operator=(const TCPSpeedLimiterTask&) = delete;
  TCPSpeedLimiterTask(TCPSpeedLimiterTask&&) = delete;
  TCPSpeedLimiterTask& operator=(TCPSpeedLimiterTask&&) = delete;

protected:
  static tesseract::task_composer::TaskComposerNodePorts ports()
  {
    tesseract::task_composer::TaskComposerNodePorts ports;
    ports.input_required[INOUT_PROGRAM_PORT] = tesseract::task_composer::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_ENVIRONMENT_PORT] = tesseract::task_composer::TaskComposerNodePorts::SINGLE;
    ports.input_required[INPUT_PROFILES_PORT] = tesseract::task_composer::TaskComposerNodePorts::SINGLE;
    ports.output_required[INOUT_PROGRAM_PORT] = tesseract::task_composer::TaskComposerNodePorts::SINGLE;
    return ports;
  }

  tesseract::task_composer::TaskComposerNodeInfo runImpl(tesseract::task_composer::TaskComposerContext& context,
                                                         OptionalTaskComposerExecutor /*executor*/) const override final
  {
    tesseract::task_composer::TaskComposerNodeInfo info(*this);
    info.return_value = 0;
    info.status_code = 0;

    if (context.isAborted())
    {
      info.status_message = "Aborted";
      return info;
    }

    // Get the environment input
    auto env_poly = getData(context, INPUT_ENVIRONMENT_PORT);
    if (env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract::environment::Environment>)))
    {
      info.color = "red";
      info.status_message =
          "Input data " + input_keys_.get(INPUT_ENVIRONMENT_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto env = env_poly.as<std::shared_ptr<const tesseract::environment::Environment>>();

    // Composite instruction input
    auto input_data_poly = getData(context, INOUT_PROGRAM_PORT);
    if (input_data_poly.getType() != std::type_index(typeid(tesseract::command_language::CompositeInstruction)))
    {
      info.color = "red";
      info.status_message = "Input data " + input_keys_.get(INOUT_PROGRAM_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto& ci = input_data_poly.as<tesseract::command_language::CompositeInstruction>();
    const tesseract::common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get the profile dictionary input
    auto profiles_poly = getData(context, INPUT_PROFILES_PORT);
    if (profiles_poly.isNull() ||
        profiles_poly.getType() != std::type_index(typeid(std::shared_ptr<tesseract::common::ProfileDictionary>)))
    {
      info.color = "red";
      info.status_message = "Input data " + input_keys_.get(INPUT_PROFILES_PORT) + " is missing or of incorrect type";
      return info;
    }
    auto profiles = profiles_poly.as<std::shared_ptr<tesseract::common::ProfileDictionary>>();

    // Get Composite Profile
    const std::string profile = ci.getProfile(ns_);
    auto cur_composite_profile =
        profiles->getProfile<TCPSpeedLimiterProfile>(name_, profile, std::make_shared<TCPSpeedLimiterProfile>());

    auto flattened = ci.flatten(tesseract::command_language::moveFilter);
    if (flattened.empty())
    {
      info.color = "yellow";
      info.status_message = "TCP speed limiter task found no MoveInstructions to process";
      info.status_code = 1;
      info.return_value = 1;
      return info;
    }

    tesseract::time_parameterization::InstructionsTrajectory trajectory(ci);
    const std::string joint_group = env->getJointGroup(manip_info.manipulator)->getName();

    try
    {
      limitTCPSpeed(env, trajectory, joint_group, manip_info.tcp_frame, cur_composite_profile->max_speed);
    }
    catch (const std::exception& ex)
    {
      info.status_message = ex.what();
      return info;
    }

    setData(context, INOUT_PROGRAM_PORT, input_data_poly);

    info.color = "green";
    info.status_message = "TCP speed limiter task succeeded";
    info.status_code = 1;
    info.return_value = 1;
    return info;
  }
};

const std::string TCPSpeedLimiterTask::INOUT_PROGRAM_PORT = "program";
const std::string TCPSpeedLimiterTask::INPUT_ENVIRONMENT_PORT = "environment";
const std::string TCPSpeedLimiterTask::INPUT_PROFILES_PORT = "profiles";

using TCPSpeedLimiterTaskFactory = tesseract::task_composer::TaskComposerTaskFactory<TCPSpeedLimiterTask>;

}  // namespace snp_motion_planning

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(snp_motion_planning::TCPSpeedLimiterTaskFactory, TCPSpeedLimiterTaskFactory)
