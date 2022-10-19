#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <string>
#include "valhalla/mjolnir/osmdata.h"
#include "valhalla/baldr/graphid.h"
#include "valhalla/baldr/graphtileptr.h"
#include "valhalla/mjolnir/util.h"

#include "baldr/curler.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "valhalla/midgard/logging.h"
#include "valhalla/mjolnir/graphbuilder.h"
#include "valhalla/mjolnir/pbfgraphparser.h"
#include "rapidjson/document.h"

#include "valhalla/baldr/directededge.h"
#include "valhalla/baldr/graphconstants.h"
#include "valhalla/baldr/graphreader.h"
#include "valhalla/baldr/graphtile.h"
#include "valhalla/midgard/pointll.h"
#include "valhalla/mjolnir/graphtilebuilder.h"
#include "valhalla/baldr/edgeinfo.h"
#include "valhalla/midgard/sequence.h"
#include "valhalla/mjolnir/graphenhancer.h"
#include "valhalla/mjolnir/graphfilter.h"
#include "valhalla/mjolnir/elevationbuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {
namespace gbfs {


struct gbfs_urls {
  
};

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

  gbfs_graph_builder(boost::property_tree::ptree& original_config,
                   const std::vector<std::string>& input_files) :
                   config(original_config),
                   input_files(input_files) {
    config.get_child("mjolnir").erase("tile_extract");
    config.get_child("mjolnir").erase("tile_url");
    config.get_child("mjolnir").erase("traffic_extract");
    tile_dir = config.get<std::string>("mjolnir.tile_dir");
  }

  bool build(bool parse_osm_first);

private:
  void fetch_gbfs_data();

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

  DirectedEdge& copy_edge(const DirectedEdge* directededge, GraphId& edgeid, graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, GraphId& nodeid);
  NodeInfo& copy_node(GraphId& nodeid, const NodeInfo* nodeinfo, graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, uint32_t edge_count, uint32_t edge_index);
};




} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
