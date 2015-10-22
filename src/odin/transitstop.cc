#include "odin/transitstop.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

TransitStop::TransitStop(TripPath_TransitStopInfo_Type type, uint32_t id,
                         std::string onestop_id, std::string name,
                         std::string arrival_date_time,
                         std::string departure_date_time, uint32_t parent_id,
                         std::string parent_onestop_id, bool is_parent_stop)
    : id(id),
      onestop_id(onestop_id),
      name(name),
      arrival_date_time(arrival_date_time),
      departure_date_time(departure_date_time),
      parent_id(id),
      parent_onestop_id(onestop_id),
      is_parent_stop(is_parent_stop) {
  set_type(type);
}

void TransitStop::set_type(TripPath_TransitStopInfo_Type in_type) {
  switch (in_type) {
    case TripPath_TransitStopInfo_Type_kStation: {
      type = TripDirections_TransitStop_Type_kStation;
      break;
    }
    default: {
      type = TripDirections_TransitStop_Type_kStop;
    }
  }
}

// TODO: do we need?
std::string TransitStop::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";
  //str += GetQuotedString(text_);

  str += delim;
  //str += GetQuotedString(std::to_string(consecutive_count_));

  str += " }";

  return str;
}

}
}
