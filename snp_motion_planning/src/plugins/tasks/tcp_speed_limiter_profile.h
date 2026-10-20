#pragma once

#include <memory>
#include <string>
#include <tesseract/common/profile.h>

static const std::string TCP_SPEED_LIMITER_TASK_NAME = "TCPSpeedLimiterTask";

namespace snp_motion_planning
{
struct TCPSpeedLimiterProfile : public tesseract::common::Profile
{
  using Ptr = std::shared_ptr<TCPSpeedLimiterProfile>;
  using ConstPtr = std::shared_ptr<const TCPSpeedLimiterProfile>;

  TCPSpeedLimiterProfile();
  TCPSpeedLimiterProfile(double max_speed_);

  double max_speed{ 1.0 };

  bool operator==(const TCPSpeedLimiterProfile& rhs) const;
  bool operator!=(const TCPSpeedLimiterProfile& rhs) const;
};

}  // namespace snp_motion_planning
