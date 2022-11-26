#include "valhalla/thor/gbfs_routing.h"
#include <boost/format.hpp>
#include "valhalla/sif/dynamiccost.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

constexpr uint32_t kInitialEdgeLabelCount = 500000;

gbfs_routing::gbfs_routing(const boost::property_tree::ptree& config) :  Dijkstras(config) {

}

void gbfs_routing::ExpandingNode(baldr::GraphReader& graphreader, graph_tile_ptr tile, const baldr::NodeInfo* node, const sif::EdgeLabel& current, const sif::EdgeLabel* previous) {
  // auto it = target_edges_.find(current.edgeid().value);
  // // LOG_INFO((boost::format("GBFS ----- Cost: %1%") % current.cost().secs).str());
  // if(it != target_edges_.end()) {
  //   if((node->access() & kPedestrianAccess) != kPedestrianAccess) {
  //     LOG_ERROR("Possible");
  //   }
  //   auto& stations = *it;
  //   for(const auto& station : stations.second) {
  //     std::string station_id = station.name();
  //     auto it_res = result_.find(station_id);
  //     if(it_res == result_.end() || it_res->second.time_total > current.cost().secs) {
  //       auto r = gbfs_route_result();
  //       r.label = current;
  //       r.time_total = current.cost().secs;
  //       result_[station_id] = r;
  //     }
  //   }
  // }
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

std::vector<std::pair<uint64_t, gbfs_route_result>> gbfs_routing::Expand(ExpansionType expansion_type, valhalla::Api& api, baldr::GraphReader& reader, const sif::mode_costing_t& costings, const sif::TravelMode mode, std::unordered_map<uint64_t, std::vector<valhalla::Location>> target_edges) {
  costings_ = costings;
  target_edges_ = target_edges;
  result_.clear();

  Dijkstras::Expand(expansion_type, api, reader, costings, mode);

  for(const auto& edge_label : bdedgelabels_) {
    auto it = target_edges_.find(edge_label.edgeid().value);
    if(it != target_edges_.end()) {
      // if((node->access() & kPedestrianAccess) != kPedestrianAccess) {
      //   LOG_ERROR("Possible");
      // }
      auto& stations = *it;
      for(const auto& station : stations.second) {
        add_result(reader, station, edge_label);
      }
    }
  }

  // LOG_INFO((boost::format("GBFS ----- Result found: %1%") % result_.size()).str());
  // for(auto& r : result_) {
  //   LOG_INFO((boost::format("GBFS ----- Result for station %1% with time %2%") % r.first % r.second.time_total).str());
  //   fill_result(reader, r.second);
  // }
  
  std::vector<std::pair<uint64_t, gbfs_route_result>> result;
  std::copy_if(result_.begin(), result_.end(), std::back_inserter(result), [&](auto ie) { return ie.second.start.type != 0; });
  // LOG_INFO((boost::format("GBFS ----- Result left: %1%") % result.size()).str());
  return result;
}

void gbfs_routing::add_result(baldr::GraphReader& graphreader, const valhalla::Location& location, const sif::EdgeLabel& last_label) {
  int pedestrian_nodes = 0;
  int bicycle_nodes = 0;

  uint64_t station_id = location.gbfs_transport_station_id();
  auto it_res = result_.find(station_id);
  if(it_res != result_.end() && it_res->second.time_total < last_label.cost().secs) {
    return;
  }

  auto result = gbfs_route_result();
  result.label = last_label;
  result.time_total = last_label.cost().secs;

  // LOG_INFO((boost::format("GBFS ----- Result for station %1% with time %2%") % station_id % result.time_total).str());

  for (auto edgelabel_index = result.label.predecessor(); edgelabel_index != kInvalidLabel; edgelabel_index = bdedgelabels_[edgelabel_index].predecessor()) {
    const sif::EdgeLabel edgelabel = bdedgelabels_[edgelabel_index];
    graph_tile_ptr tile = graphreader.GetGraphTile(edgelabel.edgeid());
    const NodeInfo* node = tile->node(edgelabel.endnode());
    if(node->access() == kPedestrianAccess) {
      pedestrian_nodes++;
    }
    else {
      bicycle_nodes++;
    }

    if(node->access() != kPedestrianAccess || node->transition_count() == 0) {
      continue;
    }
    const auto& gbfs_locations = tile->gbfs_locations();
    auto it = std::find_if(gbfs_locations.begin(), gbfs_locations.end(), [&](const gbfs_location_node& l) {
      return l.node_id == edgelabel.endnode().id();
    });
    if(it == gbfs_locations.end()) {
      LOG_ERROR("GBFS ----- gbfs location was not found");
      throw std::exception();
    }
    const gbfs_location_node& l = *it;
    // LOG_INFO((boost::format("GBFS ----- Found location: %1%, id: %2%") % static_cast<uint32_t>(l.type) % std::string(l.id.data())).str());
    PointLL node_ll = node->latlng(tile->header()->base_ll());
    if(result.start.type == 0 && l.type == static_cast<uint8_t>(gbfs_location_node_type::kFreeBike)) {
      result.start = l;
      result.start_ll = node_ll;
      result.time_pedestrian = edgelabel.cost().secs;
      result.time_bicycle = result.time_total - result.time_pedestrian;
      break;
    }
    if(l.type == static_cast<uint8_t>(gbfs_location_node_type::kBicycleStation)) {
      if(result.end_station.type == 0) {
        result.end_station = l;
        result.end_station_ll = node_ll;
        result.time_bicycle = edgelabel.cost().secs;
        result.time_pedestrian_end = result.time_total - result.time_bicycle;
      }
      else{
        result.start = l;
        result.start_ll = node_ll;
        result.time_pedestrian = edgelabel.cost().secs;
        result.time_bicycle -= result.time_pedestrian;
        break;
      }
    }
  }

  result_[station_id] = result;

  // LOG_INFO((boost::format("GBFS ----- Nodes in result: pedestrian: %1%; bicycle: %2%") % pedestrian_nodes % bicycle_nodes).str());
}


} // namespace thor
} // namespace valhalla