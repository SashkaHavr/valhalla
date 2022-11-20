#include "valhalla/thor/worker.h"
#include <boost/format.hpp>

namespace valhalla {
namespace thor {

std::string thor_worker_t::gbfs_route(Api& request) {
  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);
  // auto costing = options.costing_type();
  // auto costing_str = Costing_Enum_Name(costing);

  // sif::mode_costing_t costings;
  // costings[1] = factory.Create(options.costings().find(Costing::pedestrian)->second);
  // costings[2] = factory.Create(options.costings().find(Costing::bicycle)->second);

  // LOG_INFO((boost::format("GBFS ----- Costing original: %1%") % static_cast<int>(mode_costing[2]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Consting own 1: %1%") % static_cast<int>(costings[1]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own 2: %1%") % static_cast<int>(costings[2]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own create 1: %1%") % static_cast<int>(costings_[1]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own create 2: %1%") % static_cast<int>(costings_[2]->travel_mode())).str());
  LOG_INFO((boost::format("GBFS ----- Travel mode: %1%") % static_cast<int>(mode)).str());
  

  gbfs_router.Expand(ExpansionType::forward, request, *reader, mode_costing, mode);
  gbfs_router.Clear();
  return "Entered kek";
}

} // namespace thor
} // namespace valhalla