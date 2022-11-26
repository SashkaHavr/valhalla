#pragma once
#include "valhalla/sif/dynamiccost.h"
#include "valhalla/baldr/graphconstants.h"
#include "valhalla/thor/dijkstras.h"
#include "valhalla/midgard/pointll.h"

namespace valhalla {
namespace thor {

struct gbfs_route_result {
  sif::EdgeLabel label;
  float time_total;

  float time_pedestrian;
  float time_bicycle;
  gbfs_location_node start;
  PointLL start_ll;

  // Additional fields for stations
  float time_pedestrian_end;
  gbfs_location_node end_station;
  PointLL end_station_ll;

  // gbfs_route_result() : gbfs_route_result(sif::EdgeLabel(), 0, 0, 0, gbfs_location_node(), 0, gbfs_location_node()) {

  // }

  // gbfs_route_result(sif::EdgeLabel label, float time_total, float time_pedestrian, float time_bicycle, gbfs_location_node start) 
  //   : gbfs_route_result(label, time_total, time_pedestrian, time_bicycle, start, 0, gbfs_location_node()) {

  // }

  // gbfs_route_result(sif::EdgeLabel label, float time_total, float time_pedestrian, float time_bicycle, gbfs_location_node start, float time_pedestrian_end, gbfs_location_node end_station) 
  //   : label(label), time_total(time_total), time_pedestrian(time_pedestrian), time_bicycle(time_bicycle), start(start), time_pedestrian_end(time_pedestrian_end), end_station(end_station) {

  // }
};


class gbfs_routing : public Dijkstras {

public:
/**
 * Constructor.
 * @param config A config object of key, value pairs
 */
explicit gbfs_routing(const boost::property_tree::ptree& config = {});

std::vector<std::pair<uint64_t, gbfs_route_result>> Expand(ExpansionType expansion_type, valhalla::Api& api, baldr::GraphReader& reader,
                    const sif::mode_costing_t& costings, const sif::TravelMode mode, std::unordered_map<uint64_t, std::vector<valhalla::Location>> target_edges);


protected:
  // when we expand up to a node we color the cells of the grid that the edge that ends at the
  // node touches
  virtual void ExpandingNode(baldr::GraphReader& graphreader,
                             graph_tile_ptr tile,
                             const baldr::NodeInfo* node,
                             const sif::EdgeLabel& current,
                             const sif::EdgeLabel* previous) override;

  // when the main loop is looking to continue expanding we tell it to terminate here
  virtual ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                               const sif::EdgeLabel& pred,
                                               const ExpansionType route_type) override;

  // tell the expansion how many labels to expect and how many buckets to use
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  virtual void BeforeExpandInner(baldr::GraphReader& graphreader,
                             graph_tile_ptr tile,
                             const baldr::NodeInfo* node) override;
  
  sif::mode_costing_t costings_;
  std::unordered_map<uint64_t, std::vector<valhalla::Location>> target_edges_;

  std::unordered_map<uint64_t, gbfs_route_result> result_;
  void add_result(baldr::GraphReader& graphreader, const valhalla::Location& location, const sif::EdgeLabel& last_label);

};

} // namespace thor
} // namespace valhalla