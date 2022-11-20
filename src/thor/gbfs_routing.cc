#include "valhalla/thor/gbfs_routing.h"
#include <boost/format.hpp>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

constexpr uint32_t kInitialEdgeLabelCount = 500000;

gbfs_routing::gbfs_routing(const boost::property_tree::ptree& config) :  Dijkstras(config) {

}

void gbfs_routing::ExpandingNode(baldr::GraphReader& graphreader, graph_tile_ptr tile, const baldr::NodeInfo* node, const sif::EdgeLabel& current, const sif::EdgeLabel* previous) {

}

void gbfs_routing::BeforeExpandInner(baldr::GraphReader& graphreader, graph_tile_ptr tile, const baldr::NodeInfo* node) {
  auto old_costing_mode = costing_->travel_mode();
  if((node->access() & kPedestrianAccess) == kPedestrianAccess) {
    mode_ = sif::travel_mode_t::kPedestrian;
    access_mode_ = kPedestrianAccess;
  }
  else if((node->access() & kBicycleAccess) == kBicycleAccess) {
    mode_ = sif::travel_mode_t::kBicycle;
    access_mode_ = kBicycleAccess;
  }
  costing_ = costings_[static_cast<uint32_t>(mode_)];
  
  if(old_costing_mode != costing_->travel_mode()) {
    // LOG_INFO("GBFS ----- Mode changed");
  }
}

ExpansionRecommendation gbfs_routing::ShouldExpand(baldr::GraphReader& graphreader, const sif::EdgeLabel& pred, const ExpansionType route_type) {
  float time = pred.predecessor() == kInvalidLabel ? 0.f : bdedgelabels_[pred.predecessor()].cost().secs;
  float distance = pred.predecessor() == kInvalidLabel ? 0.f : bdedgelabels_[pred.predecessor()].path_distance();
  // prune the edge if its start is above max contour
  if (time > 900) {
    // LOG_INFO((boost::format("GBFS ----- Prune expansion, time: %1%; distance: %2%") % time % distance).str());
    return ExpansionRecommendation::prune_expansion;
  }

  return ExpansionRecommendation::continue_expansion;
}


void gbfs_routing::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const  {
  bucket_count = 20000;
  edge_label_reservation = kInitialEdgeLabelCount;
}

std::vector<PointLL> gbfs_routing::Expand(ExpansionType expansion_type, valhalla::Api& api, baldr::GraphReader& reader, const sif::mode_costing_t& costings, const sif::TravelMode mode) {
  costings_ = costings;
  std::vector<PointLL> res;


  Dijkstras::Expand(expansion_type, api, reader, costings, mode);

  return res;
}


} // namespace thor
} // namespace valhalla