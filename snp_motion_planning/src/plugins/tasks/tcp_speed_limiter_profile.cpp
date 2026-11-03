#include "tcp_speed_limiter_profile.h"

#include <tesseract/common/cereal_serialization.h>
#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

namespace snp_motion_planning
{

TCPSpeedLimiterProfile::TCPSpeedLimiterProfile() : Profile(createKey<TCPSpeedLimiterProfile>())
{
}

TCPSpeedLimiterProfile::TCPSpeedLimiterProfile(double max_speed_)
  : Profile(createKey<TCPSpeedLimiterProfile>()), max_speed(max_speed_)
{
}

bool TCPSpeedLimiterProfile::operator==(const TCPSpeedLimiterProfile& rhs) const
{
  return max_speed == rhs.max_speed;
}

bool TCPSpeedLimiterProfile::operator!=(const TCPSpeedLimiterProfile& rhs) const
{
  return !operator==(rhs);
}

template <class Archive>
void serialize(Archive& ar, TCPSpeedLimiterProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("max_speed", obj.max_speed));
}

}  // namespace snp_motion_planning

CEREAL_REGISTER_TYPE(snp_motion_planning::TCPSpeedLimiterProfile)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::Profile, snp_motion_planning::TCPSpeedLimiterProfile)
