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
  PointLL location;
  std::string id;
  
  station_inbound_edge(GraphId start_node, GraphId end_node, GraphId closest_edge, std::tuple<PointLL, double, int> best_projection, uint32_t access, PointLL location)
    : start_node(start_node), end_node(end_node), closest_edge(closest_edge), best_projection(best_projection), access(access), location(location){

  }
};

struct id_location_object {
  std::string id;
  PointLL location;
  uint8_t type;


  id_location_object(std::string id, PointLL location, uint8_t type) : id(id), location(location), type(type) { 
    if(id.length() > 63) {
      LOG_ERROR("GBFS ----- Location ID is too long");
      throw std::exception();
    }
  }

  std::array<char, 64> id_array() {
     std::array<char, 64> res;
     std::copy(id.begin(), id.end(), res.data());
     res[id.length()] = '\0';
     return res;
  }
};

struct gbfs_graph_builder {
  boost::property_tree::ptree& config;
  std::string tile_dir;
  std::unordered_map<GraphId, std::vector<gbfs_location_node>> stations_old;
  std::unordered_map<GraphId, std::vector<station_inbound_edge>> inbound_edges;
  std::unordered_map<GraphId, std::vector<uint32_t>> nodes_to_remove;

  gbfs_graph_builder(boost::property_tree::ptree& config) :
                   config(config) {
    config.get_child("mjolnir").erase("tile_extract");
    config.get_child("mjolnir").erase("tile_url");
    config.get_child("mjolnir").erase("traffic_extract");
    tile_dir = config.get<std::string>("mjolnir.tile_dir");
  }

  bool build(bool parse_osm_first, const std::vector<std::string>& input_files);
  void reload_free_bike_nodes();

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

  void add_gbfs_locations(std::unordered_map<GraphId, std::vector<id_location_object>> tileid_to_locations);
  NodeInfo& create_station_node(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, PointLL location);
  DirectedEdge& create_station_edge(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, const DirectedEdge* closest_edge, GraphId start_node, GraphId end_node, std::vector<PointLL> shape, uint32_t access);
  std::pair<std::vector<PointLL>, std::vector<PointLL>> create_shapes_to_edge_nodes(PointLL start_location, std::tuple<PointLL, double, int> best_projection, std::vector<PointLL> shape);
  DirectedEdge& create_inbound_station_edge(GraphReader& reader, GraphTileBuilder& tilebuilder, station_inbound_edge inbound_edge);
  inline bool is_access_equal(const DirectedEdge* edge, uint32_t access);
  void create_node_and_edges_in_location(GraphTileBuilder& tilebuilder, GraphId tile_id, graph_tile_ptr& tile, id_location_object location);
  void create_transition(NodeInfo& from, GraphId to, GraphTileBuilder& tilebuilder, bool up);
  std::unordered_map<GraphId, std::vector<id_location_object>> collect_gbgs_locations(bool only_free_bikes);
  void save_public_transport_stations();
  void clear_bins();
};




} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
