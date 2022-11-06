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

struct station_inbound_edge {
  GraphId start_node;
  GraphId end_node;
  GraphId closest_edge;
  std::tuple<PointLL, double, int> best_projection;
  uint32_t access;
  station_information station;
  
  station_inbound_edge(GraphId start_node, GraphId end_node, GraphId closest_edge, std::tuple<PointLL, double, int> best_projection, uint32_t access, const station_information& station)
    : start_node(start_node), end_node(end_node), closest_edge(closest_edge), best_projection(best_projection), access(access), station(station) {

  }
};

struct gbfs_graph_builder {
  boost::property_tree::ptree& config;
  const std::vector<std::string>& input_files;
  std::string tile_dir;
  std::unordered_map<GraphId, std::unordered_map<GraphId, GraphId>> pedestrian_to_bicycle_nodes;
  std::unordered_map<GraphId, std::vector<GraphId>> stations_old;
  std::unordered_map<GraphId, std::vector<GraphId>> transitions_to_add;

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
  void construct_full_graph(std::unordered_map<baldr::GraphId, std::vector<bicycle_edge>>& nodes_to_bicycle_edges, std::unordered_map<GraphId, std::vector<station_inbound_edge>> inbound_edges);

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

  void add_station_network(gbfs_operator* gbfs_op, std::unordered_map<GraphId, std::vector<station_inbound_edge>>& inbound_edges);
  NodeInfo& create_station_node(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, const station_information& station);
  DirectedEdge& create_station_edge(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, const DirectedEdge* closest_edge, GraphId start_node, GraphId end_node, std::vector<PointLL> shape, uint32_t access);
  std::pair<std::vector<PointLL>, std::vector<PointLL>> create_shapes_to_edge_nodes(PointLL start_location, std::tuple<PointLL, double, int> best_projection, std::vector<PointLL> shape);
  DirectedEdge& create_inbound_station_edge(GraphReader& reader, GraphTileBuilder& tilebuilder, station_inbound_edge inbound_edge);

  inline bool is_access_equal(const DirectedEdge* edge, uint32_t access);

  void update_free_bike_info();

  void add_transitions();
};




} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
