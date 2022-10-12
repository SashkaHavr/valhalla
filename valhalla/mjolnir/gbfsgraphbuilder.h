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

struct new_to_old_node {
  GraphId base_node;
  GraphId new_node;
  int access;

  new_to_old_node(GraphId base_node, GraphId new_node, uint16_t access) 
    : base_node(base_node), new_node(new_node), access(access) {

  }
};

struct old_to_new_node {
  GraphId base_node;
  GraphId pedestrian_node;
  GraphId bike_node;
  uint32_t density;

  old_to_new_node(GraphId base_node, GraphId pedestrian_node, GraphId bike_node, uint32_t density)
    : base_node(base_node), pedestrian_node(pedestrian_node), bike_node(bike_node), density(density) {

    }
};

struct gbfs_graph_builder {
  boost::property_tree::ptree& config;
  const std::vector<std::string>& input_files;
  OSMData osm_data{0};
  std::string tile_dir;
  std::map<baldr::GraphId, size_t> tiles;

  // Temporary files used during tile building
  const std::string new_to_old_file = "new_nodes_to_old_nodes.bin";
  const std::string old_to_new_file = "old_nodes_to_new_nodes.bin";

  std::string new_to_old_bin;
  std::string old_to_new_bin;

  gbfs_graph_builder(boost::property_tree::ptree& original_config,
                   const std::vector<std::string>& input_files) :
                   config(original_config),
                   input_files(input_files) {}

  bool build(bool parse_osm_first);


private:
  void init();
  void parse_ways();
  void parse_relations();
  void parse_nodes();
  void construct_edges();
  void build_graph();
  void enhance();
  void filter();
  void elevation();
  void cleanup();

  void fetch_gbfs_data();

  void create_new_nodes();

  void construct_full_graph();

  old_to_new_node find_nodes(sequence<old_to_new_node>& old_to_new, const GraphId& node);
  DirectedEdge make_network_connection_edge(GraphId start_node, GraphId end_node, GraphTileBuilder* tile_builder);
  bool OpposingEdgeInfoMatches(const graph_tile_ptr& tile, const DirectedEdge* edge);
};






} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
