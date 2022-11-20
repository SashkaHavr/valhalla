#pragma once
#include "valhalla/sif/dynamiccost.h"
#include "valhalla/baldr/graphconstants.h"
#include "valhalla/thor/dijkstras.h"
#include "valhalla/midgard/pointll.h"

namespace valhalla {
namespace thor {


class gbfs_routing : public Dijkstras {

public:
/**
 * Constructor.
 * @param config A config object of key, value pairs
 */
explicit gbfs_routing(const boost::property_tree::ptree& config = {});

std::vector<PointLL> Expand(ExpansionType expansion_type, valhalla::Api& api, baldr::GraphReader& reader,
                    const sif::mode_costing_t& costings, const sif::TravelMode mode);


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

};

} // namespace thor
} // namespace valhalla