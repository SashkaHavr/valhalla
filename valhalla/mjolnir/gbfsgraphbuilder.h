#include <vector>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "valhalla/midgard/logging.h"
#include "valhalla/baldr/graphid.h"
#include "valhalla/baldr/graphtileptr.h"
#include "valhalla/baldr/directededge.h"
#include "valhalla/baldr/graphconstants.h"
#include "valhalla/baldr/graphreader.h"
#include "valhalla/baldr/graphtile.h"
#include "valhalla/baldr/edgeinfo.h"
#include "valhalla/mjolnir/util.h"
#include "valhalla/mjolnir/graphtilebuilder.h"
#include "valhalla/mjolnir/gbfsoperatorgetter.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {
namespace gbfs {

struct bicycle_edge {
  GraphId edge_id;
  int access_count;

  bicycle_edge(GraphId edge_id, int access_count)
    : edge_id(edge_id), access_count(access_count) {}
};

struct gbfs_graph_builder {
  boost::property_tree::ptree& config;
  const std::vector<std::string>& input_files;
  std::string tile_dir;

  gbfs_graph_builder(boost::property_tree::ptree& config,
                   const std::vector<std::string>& input_files) :
                   config(config),
                   input_files(input_files) {
    config.get_child("mjolnir").erase("tile_extract");
    config.get_child("mjolnir").erase("tile_url");
    config.get_child("mjolnir").erase("traffic_extract");
    tile_dir = config.get<std::string>("mjolnir.tile_dir");
  }

  bool build(bool parse_osm_first);

private:
  /**
   * Construct bike and foot networks and connect them
  */
  void construct_full_graph(std::unordered_map<baldr::GraphId, std::vector<bicycle_edge>>& nodes_to_bicycle_edges);

  /**
   * Iterates through tiles and reads nodes and edges
   * @param edge_callback takes DirectedEdge as parameter
   * @param node_callback takes NodeInfo as parameter
  */
  template <typename E, typename N>
  void iterate_to_read(E edge_callback, N node_callback);

  /**
   * Iterates through tiles and updates nodes and edges
   * @param edge_callback takes DirectedEdge&, GraphId& of an edge and GraphId& of an outbound node as parameter to update it's fields
   * @param node_callback takes NodeInfo& as parameter to update it's fields
  */
  template <typename E, typename N>
  void iterate_to_update(E edge_callback, N node_callback);

  DirectedEdge& copy_edge(const DirectedEdge* directededge, const GraphId& edgeid, graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, const GraphId& nodeid);
  NodeInfo& copy_node(const GraphId& nodeid, const NodeInfo* nodeinfo, graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, uint32_t edge_count, uint32_t edge_index);

  void add_station_network(gbfs_operator* gbfs_op);
};




} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
