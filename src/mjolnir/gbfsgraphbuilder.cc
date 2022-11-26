#include "valhalla/mjolnir/gbfsgraphbuilder.h"
#include <boost/format.hpp>
#include <tuple>
#include <chrono>

namespace valhalla {
namespace mjolnir {
namespace gbfs {

bool gbfs_graph_builder::build(bool parse_osm_first, const std::vector<std::string>& input_files) {
  LOG_INFO("GBFS ----- Build started");
  std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
  if(parse_osm_first) {
    LOG_INFO("GBFS ----- First standard building stages: start - Initialize, end - Filter");
    build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kInitialize, valhalla::mjolnir::BuildStage::kFilter);
  }
  else {
    LOG_INFO("GBFS ----- Skipped first standard building stages");
  }

  LOG_INFO("GBFS ----- Creating station networks");
  
  int nodes_total = 0;
  iterate_to_update([&](DirectedEdge& edge, GraphId& edge_id, GraphId& node_id) {}, [&](NodeInfo& node) { nodes_total++;});
  LOG_INFO((boost::format("GBFS ----- Nodes total before: %1%") % nodes_total).str());
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  add_gbfs_locations(collect_gbgs_locations(false));
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  LOG_INFO((boost::format("GBFS ----- Time to save networks: %1%") % std::chrono::duration_cast<std::chrono::seconds>(end - begin).count()).str());
  nodes_total = 0;
  iterate_to_update([&](DirectedEdge& edge, GraphId& edge_id, GraphId& node_id) {}, [&](NodeInfo& node) { nodes_total++;});
  LOG_INFO((boost::format("GBFS ----- Nodes total after: %1%") % nodes_total).str());


  LOG_INFO("GBFS ----- Dividing pedestrian and bicycle edges");
  std::unordered_map<baldr::GraphId, std::vector<bicycle_edge>> bicycle_edges;
  int bicycle_edges_count = 0;
  int pedestrian_edges_count = 0;
  iterate_to_update([&](DirectedEdge& edge, GraphId& edge_id, GraphId& node_id) {
    int access_count = 0;
    if((edge.forwardaccess() & kBicycleAccess) == kBicycleAccess) {
      access_count += 1;
    }
    if((edge.forwardaccess() & kPedestrianAccess) == kPedestrianAccess) {
      edge.set_forwardaccess(kPedestrianAccess);
    }
    else {
      edge.set_forwardaccess(0);
    }

    if((edge.reverseaccess() & kBicycleAccess) == kBicycleAccess) {
      access_count += 2;
    }
    if((edge.reverseaccess() & kPedestrianAccess) == kPedestrianAccess) {
      edge.set_reverseaccess(kPedestrianAccess);
    }
    else {
      edge.set_reverseaccess(0);
    }

    if(access_count > 0) {
      bicycle_edges[node_id].push_back(bicycle_edge(edge_id, access_count));
      bicycle_edges_count++;
    }

    if((edge.forwardaccess() & kPedestrianAccess) == kPedestrianAccess || (edge.reverseaccess() & kPedestrianAccess) == kPedestrianAccess) {
      pedestrian_edges_count++;
    }
  }, [](NodeInfo& node) {

  });

  LOG_INFO((boost::format("GBFS ----- Pedestrian edges found: %1%") % pedestrian_edges_count).str());
  LOG_INFO((boost::format("GBFS ----- Bicycle edges found: %1%") % bicycle_edges_count).str());

  LOG_INFO("GBFS ----- Constructing a full graph");
  construct_full_graph(bicycle_edges);

  LOG_INFO("GBFS ----- Validating full graph");
  int total_pedestrian = 0;
  int total_bicycle = 0;
  iterate_to_read([&](const DirectedEdge& edge) {
    if((edge.forwardaccess() & kBicycleAccess) == kBicycleAccess || (edge.reverseaccess() & kBicycleAccess) == kBicycleAccess) {
      total_bicycle++;
    }
    if((edge.forwardaccess() & kPedestrianAccess) == kPedestrianAccess || (edge.reverseaccess() & kPedestrianAccess) == kPedestrianAccess) {
      total_pedestrian++;
    }
  }, [](const NodeInfo& node) {
    
  });

  LOG_INFO((boost::format("GBFS ----- Pedestrian edges found: %1%") % total_pedestrian).str());
  LOG_INFO((boost::format("GBFS ----- Bicycle edges found: %1%") % total_bicycle).str());

  int inbound_edges_count = 0;
  for(const auto& p : inbound_edges) {
    inbound_edges_count += p.second.size();
  }

  if(total_pedestrian != (pedestrian_edges_count + inbound_edges_count / 2)) {
    LOG_ERROR("GBFS ----- Pedestrian edges does not match");
    throw std::exception();
  }
  if(total_bicycle != (bicycle_edges_count + inbound_edges_count / 2)) {
    LOG_ERROR("GBFS ----- Bicycle edges does not match");
    throw std::exception();
  }

  int total = 0;
  LOG_INFO("GBFS ----- Loading free bikes");
  GraphReader reader2(config.get_child("mjolnir"));
  auto local_tiles = reader2.GetTileSet();
  for (const auto& tile_id : local_tiles) {
    graph_tile_ptr tile = reader2.GetGraphTile(tile_id);
    for (const auto& location : tile->gbfs_locations()) {
      GraphId node_id(tile_id.tileid(), tile_id.level(), location.node_id);
      LOG_INFO((boost::format("GBFS ----- Location id: %1%; node id: %2%; type: %3%") % std::string(location.id.data()) % node_id % static_cast<int>(location.type)).str());
      total++;
    }
  }
  LOG_INFO((boost::format("GBFS ----- Total locations loaded: %1%") % total).str());

  LOG_INFO("GBFS ----- Validating transitions");
  total = 0;
  int total_ped = 0;
  iterate_to_read([&](const DirectedEdge& edge) {
  }, [&](const NodeInfo& node) {
    if(node.access() == kPedestrianAccess) {
      total_ped += node.transition_count();
    }
    total += node.transition_count();
    if(node.transition_count() > 1) {
      LOG_ERROR("GBFS ----- There must be maximum 1 transition");
      throw std::exception();
    }
  });
  LOG_INFO((boost::format("GBFS ----- Transitions loaded: %1%") % total).str());
  LOG_INFO((boost::format("GBFS ----- Transitions pedestrian loaded: %1%") % total_ped).str());
  int total_pedestrian_should_be = 0;
  for(const auto& s : stations_old) {
    total_pedestrian_should_be += s.second.size();
  }
  if(total_pedestrian_should_be != total_ped) {
    LOG_ERROR("GBFS ----- Transitions does not match");
    throw std::exception();
  }

  LOG_INFO("GBFS ----- Last standard building stages: start - Elevation, end - Cleanup");
  build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kElevation, valhalla::mjolnir::BuildStage::kCleanup);

  std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
  LOG_INFO((boost::format("GBFS ----- Time total: %1%") % std::chrono::duration_cast<std::chrono::seconds>(end_total - begin_total).count()).str());
  
  stations_old.clear();
  inbound_edges.clear();
  nodes_to_remove.clear();
  return true;
}


// Build foot + bike networks

void gbfs_graph_builder::construct_full_graph(std::unordered_map<baldr::GraphId, std::vector<bicycle_edge>>& nodes_to_bicycle_edges) {
  std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new_pedestrian;
  std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new_bicycle;
  GraphReader reader(config.get_child("mjolnir"));

  uint32_t total_bicycle = 0;
  uint32_t total_pedestrian = 0;
  uint32_t total_transitions = 0;

  auto local_tiles = reader.GetTileSet();
  for (const auto& tile_id : local_tiles) {
    // Create a new tilebuilder - should copy header information
    GraphTileBuilder tilebuilder(tile_dir, tile_id, false);

    // Get the graph tile. Read from this tile to create the new tile.
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    std::vector<station_inbound_edge> tile_inbound_edges = inbound_edges[tile_id];
    auto& stations = stations_old[tile_id];

    GraphId nodeid(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++nodeid) {

      // Count of edges added for this node
      uint32_t edge_count = 0;
      // Current edge index for first edge from this node
      uint32_t edge_index = tilebuilder.directededges().size();

      // Iterate through directed edges outbound from this node, search for pedestrian edges and copy them
      const NodeInfo* nodeinfo = tile->node(nodeid);
      GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
        // Check if the directed edge should be included
        const DirectedEdge* directededge = tile->directededge(edgeid);
        if(directededge->forwardaccess() == 0 && directededge->reverseaccess() == 0) {
          continue;
        }

        copy_edge(directededge, edgeid, tile, tilebuilder, nodeid);
        edge_count++;
      }


      std::vector<station_inbound_edge> node_inbound_edges;
      std::copy_if(tile_inbound_edges.begin(), tile_inbound_edges.end(), std::back_inserter(node_inbound_edges), [&](station_inbound_edge ie) { return ie.start_node == nodeid; });
      for(const auto& inbound_edge : node_inbound_edges) {
        if(inbound_edge.access == kPedestrianAccess) {
          create_inbound_station_edge(reader, tilebuilder, inbound_edge);
          edge_count++;
        }
      }

      GraphId new_pedestrian_node_id;
      NodeInfo* new_pedestrian_node = nullptr;
      // Add the node to the tilebuilder unless no edges remain
      if (edge_count > 0) {
        new_pedestrian_node = &copy_node(nodeid, nodeinfo, tile, tilebuilder, edge_count, edge_index);
        new_pedestrian_node->set_access(kPedestrianAccess);
        new_pedestrian_node_id = GraphId(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size() - 1);
        old_to_new_pedestrian[nodeid] = new_pedestrian_node_id;
      }
      total_pedestrian += edge_count;

      // Count of edges added for this node
      edge_count = 0;
      edge_index = tilebuilder.directededges().size();
      auto edges = nodes_to_bicycle_edges[nodeid];
      //Create nodes and edges in bicycle network
      for (const bicycle_edge& bicycle_edge : edges) {
        // Check if the directed edge should be included
        const DirectedEdge* directededge = tile->directededge(bicycle_edge.edge_id);
        DirectedEdge& new_edge = copy_edge(directededge, bicycle_edge.edge_id, tile, tilebuilder, nodeid);
        edge_count++;
        switch(bicycle_edge.access_count) {
          case 1:
            new_edge.set_forwardaccess(kBicycleAccess);
            new_edge.set_reverseaccess(0);
            break;
          case 2:
            new_edge.set_forwardaccess(0);
            new_edge.set_reverseaccess(kBicycleAccess);
            break;
          case 3:
            new_edge.set_forwardaccess(kBicycleAccess);
            new_edge.set_reverseaccess(kBicycleAccess);
            break;
          default:
            LOG_ERROR("GBFS ----- unsupported access count");
            throw new std::exception();
            break;
        }
      }

      for(const auto& inbound_edge : node_inbound_edges) {
        if(inbound_edge.access == kBicycleAccess) {
          create_inbound_station_edge(reader, tilebuilder, inbound_edge);
          edge_count++;
        }
      }

      GraphId new_bicycle_node_id;
      NodeInfo* new_bicycle_node = nullptr;
      // Add the node to the tilebuilder unless no edges remain
      if (edge_count > 0) {
        new_bicycle_node = &copy_node(nodeid, nodeinfo, tile, tilebuilder, edge_count, edge_index);
        new_bicycle_node->set_access(kBicycleAccess);
        new_bicycle_node_id = GraphId(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size() - 1);
        old_to_new_bicycle[nodeid] = new_bicycle_node_id;
      }
      total_bicycle += edge_count;
      
      // Add transitions from all bicycle nodes to corresponding pedestrian nodes
      if(new_pedestrian_node_id.Is_Valid() && new_bicycle_node_id.Is_Valid()) {
        create_transition(*new_bicycle_node, new_pedestrian_node_id, tilebuilder, false);
        total_transitions++;

        auto station_it = std::find_if(stations.begin(), stations.end(), [&](gbfs_location_node n) {
          return n.node_id == nodeid.id();
        });
        if(station_it != stations.end()) {
          create_transition(*new_pedestrian_node, new_bicycle_node_id, tilebuilder, true);
          tilebuilder.gbfs_locations_builder().push_back({new_pedestrian_node_id.id(), station_it->type, station_it->id});
        }
      }
    }

    // Store the updated tile data (or remove tile if all edges are filtered)
    if (tilebuilder.nodes().size() > 0) {
      tilebuilder.StoreTileData();
    } else {
      // Remove the tile - all nodes and edges were filtered
      std::string file_location =
          tile_dir + filesystem::path::preferred_separator + GraphTile::FileSuffix(tile_id);
      remove(file_location.c_str());
      LOG_INFO("GBFS ----- Remove file: " + file_location + " all edges were filtered");
    }

    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }


  LOG_INFO((boost::format("GBFS ----- Created pedestrian edges: %1%") % total_pedestrian).str());
  LOG_INFO((boost::format("GBFS ----- Created bicycle edges: %1%") % total_bicycle).str());
  LOG_INFO((boost::format("GBFS ----- Created transitions: %1%") % total_transitions).str());

  total_pedestrian = 0;
  total_bicycle = 0;
  total_transitions = 0;

  LOG_INFO("GBFS ----- Validating edge creation");
  iterate_to_read([&](const DirectedEdge& edge) {
    if((edge.forwardaccess() & kBicycleAccess) == kBicycleAccess || (edge.reverseaccess() & kBicycleAccess) == kBicycleAccess) {
      total_bicycle++;
    }
    if((edge.forwardaccess() & kPedestrianAccess) == kPedestrianAccess || (edge.reverseaccess() & kPedestrianAccess) == kPedestrianAccess) {
      total_pedestrian++;
    }
  }, [&](const NodeInfo& node) {
    total_transitions += node.transition_count();
  });

  LOG_INFO((boost::format("GBFS ----- Pedestrian edges found: %1%") % total_pedestrian).str());
  LOG_INFO((boost::format("GBFS ----- Bicycle edges found: %1%") % total_bicycle).str());
  LOG_INFO((boost::format("GBFS ----- Transitions found: %1%") % total_transitions).str());


  LOG_INFO("GBFS ----- Update end nodes of new directed edges");
  // Update end nodes in edges
  iterate_to_update([&](DirectedEdge& edge, GraphId& edge_id, GraphId& node_id) {
    GraphId end_node;
    if((edge.forwardaccess() & kPedestrianAccess) == kPedestrianAccess || (edge.reverseaccess() & kPedestrianAccess) == kPedestrianAccess) {
      auto iter = old_to_new_pedestrian.find(edge.endnode());
      if (iter != old_to_new_pedestrian.end()) {
        end_node = iter->second;
      }
    }
    else if((edge.forwardaccess() & kBicycleAccess) == kBicycleAccess || (edge.reverseaccess() & kBicycleAccess) == kBicycleAccess) {
      auto iter = old_to_new_bicycle.find(edge.endnode());
      if (iter != old_to_new_bicycle.end()) {
        end_node = iter->second;
      }
    }

    if (!end_node.Is_Valid()) {
      LOG_ERROR("UpdateEndNodes - failed to find associated node");
      throw std::exception();
    } else {
      edge.set_endnode(end_node);
    }
  }, [](NodeInfo& node) {

  });
}

template <typename E, typename N>
void gbfs_graph_builder::iterate_to_read(E edge_callback, N node_callback) {
  GraphReader reader(config.get_child("mjolnir"));
  int count = 0;
  auto local_tiles = reader.GetTileSet();
  for (const auto& tile_id : local_tiles) {
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);

    for (uint32_t i = 0; i < tile->header()->nodecount(); ++i) {
      const NodeInfo* nodeinfo = tile->node(i);
      uint32_t idx = nodeinfo->edge_index();
      for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++idx) {
        const DirectedEdge* edge = tile->directededge(idx);
        edge_callback(*edge);
        count++;
      }
      node_callback(*nodeinfo);
    }
  }
  LOG_INFO((boost::format("GBFS ----- Edges checked: %1%") % count).str());
}

template <typename E, typename N>
void gbfs_graph_builder::iterate_to_update(E edge_callback, N node_callback) {
  GraphReader graph_reader(config.get_child("mjolnir"));
  int count = 0;
  auto local_tiles = graph_reader.GetTileSet();
  for (const auto& tile_id : local_tiles) {
    graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);

    // Create a new tile builder
    GraphTileBuilder tilebuilder(tile_dir, tile_id, false);

    // Update end nodes of transit connection directed edges
    std::vector<NodeInfo> nodes;
    std::vector<DirectedEdge> directededges;
    GraphId node_id(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); ++i, ++node_id) {
      NodeInfo& nodeinfo = tilebuilder.node(i);
      uint32_t idx = nodeinfo.edge_index();
      GraphId edge_id(tile_id.tileid(), tile_id.level(), idx);
      for (uint32_t j = 0; j < nodeinfo.edge_count(); ++j, ++idx, ++edge_id) {
        DirectedEdge& directededge = tilebuilder.directededge(idx);
        edge_callback(directededge, edge_id, node_id);
        // Add the directed edge to the local list
        directededges.emplace_back(std::move(directededge));
        count++;
      }

      node_callback(nodeinfo);
      // Add the node to the local list
      nodes.emplace_back(std::move(nodeinfo));
    }
    tilebuilder.Update(nodes, directededges);
  }
  LOG_INFO((boost::format("GBFS ----- Edges updated: %1%") % count).str());
}


DirectedEdge& gbfs_graph_builder::copy_edge(const DirectedEdge* directededge, const GraphId& edgeid, 
                                              graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, const GraphId& nodeid) {
  // Copy the directed edge information
  DirectedEdge newedge = *directededge;

  // Set opposing edge indexes to 0 (gets set in graph validator).
  newedge.set_opp_index(0);

  // Get signs from the base directed edge
  if (directededge->sign()) {
    std::vector<SignInfo> signs = tile->GetSigns(edgeid.id());
    if (signs.size() == 0) {
      LOG_ERROR("Base edge should have signs, but none found");
    }
    tilebuilder.AddSigns(tilebuilder.directededges().size(), signs);
  }

  // Get turn lanes from the base directed edge
  if (directededge->turnlanes()) {
    uint32_t offset = tile->turnlanes_offset(edgeid.id());
    tilebuilder.AddTurnLanes(tilebuilder.directededges().size(), tile->GetName(offset));
  }

  // Get access restrictions from the base directed edge. Add these to
  // the list of access restrictions in the new tile. Update the
  // edge index in the restriction to be the current directed edge Id
  if (directededge->access_restriction()) {
    auto restrictions = tile->GetAccessRestrictions(edgeid.id(), kAllAccess);
    for (const auto& res : restrictions) {
      tilebuilder.AddAccessRestriction(AccessRestriction(tilebuilder.directededges().size(),
                                                          res.type(), res.modes(), res.value()));
    }
  }

  // Copy lane connectivity
  if (directededge->laneconnectivity()) {
    auto laneconnectivity = tile->GetLaneConnectivity(edgeid.id());
    if (laneconnectivity.size() == 0) {
      LOG_ERROR("Base edge should have lane connectivity, but none found");
    }
    for (auto& lc : laneconnectivity) {
      lc.set_to(tilebuilder.directededges().size());
    }
    tilebuilder.AddLaneConnectivity(laneconnectivity);
  }

  // Get edge info, shape, and names from the old tile and add to the
  // new. Cannot use edge info offset since edges in arterial and
  // highway hierarchy can cross base tiles! Use a hash based on the
  // encoded shape plus way Id.
  bool added;
  auto edgeinfo = tile->edgeinfo(directededge);
  std::string encoded_shape = edgeinfo.encoded_shape();
  std::hash<std::string> hasher;
  uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
  uint32_t edge_info_offset =
      tilebuilder.AddEdgeInfo(w, nodeid, directededge->endnode(), edgeinfo.wayid(),
                              edgeinfo.mean_elevation(), edgeinfo.bike_network(),
                              edgeinfo.speed_limit(), encoded_shape, edgeinfo.GetNames(),
                              edgeinfo.GetTaggedValues(), edgeinfo.GetTaggedValues(true),
                              edgeinfo.GetTypes(), added);
  newedge.set_edgeinfo_offset(edge_info_offset);
  // wayid.push_back(edgeinfo.wayid()); //REMOVE
  // endnode.push_back(directededge->endnode());

  // Add directed edge
  tilebuilder.directededges().emplace_back(std::move(newedge));
  return tilebuilder.directededges().back();
}

NodeInfo& gbfs_graph_builder::copy_node(const GraphId& nodeid, const NodeInfo* nodeinfo, graph_tile_ptr& tile, 
                                        GraphTileBuilder& tilebuilder, uint32_t edge_count, uint32_t edge_index) {
  // Add a node builder to the tile. Update the edge count and edgeindex
  GraphId new_node(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size());
  tilebuilder.nodes().push_back(*nodeinfo);
  NodeInfo& node = tilebuilder.nodes().back();
  node.set_edge_count(edge_count);
  node.set_edge_index(edge_index);
  node.set_transition_count(0);
  node.set_transition_index(0);
  const auto& admin = tile->admininfo(nodeinfo->admin_index());
  node.set_admin_index(tilebuilder.AddAdmin(admin.country_text(), admin.state_text(),
                                            admin.country_iso(), admin.state_iso()));

  // Get named signs from the base node
  if (nodeinfo->named_intersection()) {
    std::vector<SignInfo> signs = tile->GetSigns(nodeid.id(), true);
    if (signs.size() == 0) {
      LOG_ERROR("Base node should have signs, but none found");
    }
    node.set_named_intersection(true);
    tilebuilder.AddSigns(tilebuilder.nodes().size() - 1, signs);
  }
  return tilebuilder.nodes().back();
}

void gbfs_graph_builder::add_gbfs_locations(std::unordered_map<GraphId, std::vector<id_location_object>> tileid_to_locations) {
  GraphReader reader(config.get_child("mjolnir"));
  auto local_tiles = reader.GetTileSet();
  for(const auto& tile_stations : tileid_to_locations) {
    GraphId tile_id = tile_stations.first;
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    if(tile == nullptr) {
      LOG_INFO("GBFS ----- Tile is null, locations aren't created");
      continue;
    }
    GraphTileBuilder tilebuilder(tile_dir, tile_id, true);
    for(const auto& station : tile_stations.second) {
      create_node_and_edges_in_location(tilebuilder, tile_id, tile, station);
    }
    tilebuilder.StoreTileData();
  }
}

std::unordered_map<GraphId, std::vector<id_location_object>> gbfs_graph_builder::collect_gbgs_locations(bool only_free_bikes) {
  gbfs_operator_getter operator_getter(config);
  auto operators = operator_getter.operators();
  std::unordered_map<GraphId, std::vector<id_location_object>> tileid_to_locations;
  for(const auto& o : operators) {
    const auto& stations = o->station_information().stations();
    std::string op_name = o->system_information().operator_name();
    if(!only_free_bikes) {
      if(stations.size() == 0) {
        LOG_INFO("GBFS ----- There aren't any stations in " + op_name);
      }
      else {
        LOG_INFO((boost::format("GBFS ----- Creating %1% station nodes for %2%") % stations.size() % op_name).str());
      }
      for(const station_information& station : stations) {
        GraphId tile_id = TileHierarchy::GetGraphId(station.location, TileHierarchy::levels().back().level);
        tileid_to_locations[tile_id].push_back({station.id, station.location, static_cast<uint8_t>(gbfs_location_node_type::kBicycleStation)});
      }
    }

    const auto& free_bikes = o->free_bike_status().free_bikes();
    if(free_bikes.size() == 0) {
      LOG_INFO("GBFS ----- There aren't any free bikes in " + op_name);
    }
    else {
      LOG_INFO((boost::format("GBFS ----- Creating %1% free bike nodes for %2%") % free_bikes.size() % op_name).str());
    }
    for(const free_bike& free_bike : free_bikes) {
      GraphId tile_id = TileHierarchy::GetGraphId(free_bike.location, TileHierarchy::levels().back().level);
      tileid_to_locations[tile_id].push_back({free_bike.id, free_bike.location, static_cast<uint8_t>(gbfs_location_node_type::kFreeBike)});
    }
  }
  return tileid_to_locations;
}

void gbfs_graph_builder::create_node_and_edges_in_location(GraphTileBuilder& tilebuilder, GraphId tile_id, graph_tile_ptr& tile, id_location_object location) {
    // Find best projection for station
  double min_distance_pedestrian = DBL_MAX;
  double min_distnace_bicycle = DBL_MAX;
  std::pair<GraphId, GraphId> best_node_and_edge_pedestrian;
  std::pair<GraphId, GraphId> best_node_and_edge_bicycle;
  std::tuple<PointLL, double, int> best_projection_pedestrian;
  std::tuple<PointLL, double, int> best_projection_bicycle;
  GraphId node_id(tile_id.tileid(), tile_id.level(), 0);
  for (uint32_t i = 0; i < tile->header()->nodecount(); ++i, ++node_id) {
    const NodeInfo* nodeinfo = tile->node(i);
    PointLL nodell = nodeinfo->latlng(tile->header()->base_ll());
    if(!nodell.ApproximatelyEqual(location.location, 0.025)) {
      continue;
    }

    uint32_t idx = nodeinfo->edge_index();
    GraphId edge_id(tile_id.tileid(), tile_id.level(), idx);
    for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++idx, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(idx);
      auto edge_info = tile->edgeinfo(edge);
      auto projection = location.location.Project(edge_info.shape());
      double distance = std::get<1>(projection);
      if(is_access_equal(edge, kPedestrianAccess) && distance < min_distance_pedestrian) {
        min_distance_pedestrian = distance;
        best_projection_pedestrian = projection;
        best_node_and_edge_pedestrian = std::pair<GraphId, GraphId>(node_id, edge_id);
      }
      if(is_access_equal(edge, kBicycleAccess) && distance < min_distnace_bicycle) {
        min_distnace_bicycle = distance;
        best_projection_bicycle = projection;
        best_node_and_edge_bicycle = std::pair<GraphId, GraphId>(node_id, edge_id);
      }
    }
  }
  if(!(best_node_and_edge_pedestrian.first.Is_Valid() && best_node_and_edge_pedestrian.second.Is_Valid() 
        && best_node_and_edge_bicycle.first.Is_Valid() && best_node_and_edge_bicycle.second.Is_Valid())) {
    LOG_ERROR((boost::format("GBFS ----- No nodes for this station found. Id: %1%") % location.id).str());
    return;
  }


  // Create node for station
  GraphId new_node_id(tile_id.tileid(), tile_id.level(), tilebuilder.nodes().size());
  create_station_node(tilebuilder, tile, location.location);
  stations_old[tile_id].push_back({new_node_id.id(), location.type, location.id_array()});

  // Create outbound edges
  const DirectedEdge* edge_pedestrian = tile->directededge(best_node_and_edge_pedestrian.second);
  EdgeInfo edge_info_pedestrian = tile->edgeinfo(edge_pedestrian);
  auto shapes_pedestrian = create_shapes_to_edge_nodes(location.location, best_projection_pedestrian, edge_info_pedestrian.shape());
  const DirectedEdge* edge_bicycle = tile->directededge(best_node_and_edge_bicycle.second);
  EdgeInfo edge_info_bicycle = tile->edgeinfo(edge_bicycle);
  auto shapes_bicycle = create_shapes_to_edge_nodes(location.location, best_projection_bicycle, edge_info_bicycle.shape());

  create_station_edge(tilebuilder, tile, edge_pedestrian, new_node_id, best_node_and_edge_pedestrian.first, shapes_pedestrian.first, kPedestrianAccess);
  create_station_edge(tilebuilder, tile, edge_pedestrian, new_node_id, edge_pedestrian->endnode(), shapes_pedestrian.second, kPedestrianAccess);
  create_station_edge(tilebuilder, tile, edge_bicycle, new_node_id, best_node_and_edge_bicycle.first, shapes_bicycle.first, kBicycleAccess);
  create_station_edge(tilebuilder, tile, edge_bicycle, new_node_id, edge_bicycle->endnode(), shapes_bicycle.second, kBicycleAccess);

  inbound_edges[tile_id].push_back(station_inbound_edge(best_node_and_edge_pedestrian.first, new_node_id, best_node_and_edge_pedestrian.second, best_projection_pedestrian, kPedestrianAccess, location.location));
  inbound_edges[edge_pedestrian->endnode().Tile_Base()].push_back(station_inbound_edge(edge_pedestrian->endnode(), new_node_id, best_node_and_edge_pedestrian.second, best_projection_pedestrian, kPedestrianAccess, location.location));
  inbound_edges[tile_id].push_back(station_inbound_edge(best_node_and_edge_bicycle.first, new_node_id, best_node_and_edge_bicycle.second, best_projection_bicycle, kBicycleAccess, location.location));
  inbound_edges[edge_bicycle->endnode().Tile_Base()].push_back(station_inbound_edge(edge_bicycle->endnode(), new_node_id, best_node_and_edge_bicycle.second, best_projection_bicycle, kBicycleAccess, location.location));
}

NodeInfo& gbfs_graph_builder::create_station_node(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, PointLL location) {
  NodeInfo new_node(tile->header()->base_ll(), location, (kPedestrianAccess | kBicycleAccess), NodeType::kBikeShare,
                          false, true, false, false);
  new_node.set_mode_change(true);
  new_node.set_edge_index(tilebuilder.directededges().size());
  new_node.set_edge_count(4);
  tilebuilder.nodes().emplace_back(std::move(new_node));
  return tilebuilder.nodes().back();
}

DirectedEdge& gbfs_graph_builder::create_station_edge(GraphTileBuilder& tilebuilder, graph_tile_ptr& tile, const DirectedEdge* closest_edge, GraphId start_node,
                                                      GraphId end_node, std::vector<PointLL> shape, uint32_t access) {
  DirectedEdge new_directededge;
  new_directededge.set_endnode(end_node);

  new_directededge.set_length(valhalla::midgard::length(shape));
  new_directededge.set_use(closest_edge->use());
  new_directededge.set_speed(closest_edge->speed());
  new_directededge.set_surface(closest_edge->surface());
  new_directededge.set_cyclelane(closest_edge->cyclelane());
  new_directededge.set_classification(closest_edge->classification());
  // directededge.set_localedgeidx(tilebuilder.directededges().size()); //????

  new_directededge.set_forwardaccess(access);
  new_directededge.set_reverseaccess(access);
  new_directededge.set_forward(true);
  // new_directededge.set_bss_connection(true); //????
  new_directededge.set_opp_index(0);

  bool added = false;
  EdgeInfo edge_info = tile->edgeinfo(closest_edge);
  uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(tilebuilder.directededges().size(),
                                    start_node, end_node, edge_info.wayid(),
                                    edge_info.mean_elevation(), edge_info.bike_network(),
                                    edge_info.speed_limit(), shape, edge_info.GetNames(),
                                    edge_info.GetTaggedValues(), edge_info.GetTaggedValues(true),
                                    edge_info.GetTypes(), added);

  new_directededge.set_edgeinfo_offset(edge_info_offset);
  tilebuilder.directededges().emplace_back(std::move(new_directededge));
  return tilebuilder.directededges().back();
}

std::pair<std::vector<PointLL>, std::vector<PointLL>> gbfs_graph_builder::create_shapes_to_edge_nodes(PointLL start_location,
                                                        std::tuple<PointLL, double, int> best_projection, std::vector<PointLL> shape) {
  const auto& closest_point = std::get<0>(best_projection);
  auto cloest_index = std::get<2>(best_projection);
  
  std::vector<PointLL> shape_start;
  std::vector<PointLL> shape_end;

  shape_start.push_back(start_location);
  shape_start.push_back(closest_point);
  std::copy(shape.begin() + cloest_index + 1, shape.end(), std::back_inserter(shape_start));

  shape_end.push_back(start_location);
  shape_end.push_back(closest_point);
  std::copy(shape.begin(), shape.begin() + cloest_index + 1, std::back_inserter(shape_end));

  return std::pair<std::vector<PointLL>, std::vector<PointLL>>(shape_start, shape_end);
}

inline bool gbfs_graph_builder::is_access_equal(const DirectedEdge* edge, uint32_t access) {
  return (edge->forwardaccess() & access) == access || (edge->reverseaccess() & access) == access;
}

DirectedEdge& gbfs_graph_builder::create_inbound_station_edge(GraphReader& reader, GraphTileBuilder& tilebuilder, station_inbound_edge inbound_edge) {
  graph_tile_ptr tile = reader.GetGraphTile(inbound_edge.closest_edge.Tile_Base());
  const DirectedEdge* edge = tile->directededge(inbound_edge.closest_edge);
  EdgeInfo edge_info = tile->edgeinfo(edge);
  
  auto shapes = create_shapes_to_edge_nodes(inbound_edge.location, inbound_edge.best_projection, edge_info.shape());
  std::vector<PointLL> shape;
  if(edge->endnode() == inbound_edge.start_node) {
    shape = shapes.second;
  }
  else {
    shape = shapes.first;
  }
  std::reverse(shape.begin(), shape.end());
  return create_station_edge(tilebuilder, tile, edge, inbound_edge.start_node, inbound_edge.end_node, shape, inbound_edge.access);
}

void gbfs_graph_builder::create_transition(NodeInfo& from, GraphId to, GraphTileBuilder& tilebuilder, bool up) {
  if(from.transition_count() > 0) {
    LOG_INFO("GBFS ----- Skipped node - transition exists");
    return;
  }
  from.set_transition_index(tilebuilder.transitions().size());
  tilebuilder.transitions().emplace_back(to, up);
  from.set_transition_count(1);
}

void gbfs_graph_builder::reload_free_bike_nodes() {
  std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
  GraphReader reader(config.get_child("mjolnir"));
  std::unordered_map<GraphId, std::vector<id_location_object>> locations = collect_gbgs_locations(true);
  std::unordered_map<GraphId, std::vector<id_location_object>> locations_to_add;
  int total_to_update = 0;
  for(const auto& l : locations) {
    graph_tile_ptr tile = reader.GetGraphTile(l.first);
    if(tile == nullptr) {
      LOG_INFO("GBFS ----- Tile is null, locations aren't created");
      continue;
    }
    std::unordered_map<std::string, gbfs_location_node> old_locations;
    const auto& nodes = tile->gbfs_locations();
    for(const auto& node : nodes) {
      old_locations[std::string(node.id.data())] = node;
    }
    for(const auto& v : l.second) {
      auto gbfs_node_it = old_locations.find(v.id);
      if(gbfs_node_it == old_locations.end()) {
        locations_to_add.insert(l);
      }
      auto gbfs_node = gbfs_node_it->second;
      const NodeInfo* node = tile->node(gbfs_node.node_id);
      PointLL nodell = node->latlng(tile->header()->base_ll());
      LOG_INFO((boost::format("GBFS ----- Locations: (%1%; %2%) (%3%; %4%)") % v.location.lat() % v.location.lng() % nodell.lat() % nodell.lng()).str());
      if(!v.location.ApproximatelyEqual(nodell)) {
        locations_to_add.insert(l);
        nodes_to_remove[l.first].push_back(gbfs_node.node_id);
        total_to_update++;
      }
      old_locations.erase(v.id);
    }
    for(const auto& v : old_locations) {
      nodes_to_remove[l.first].push_back(v.second.node_id);
    }
  }

  LOG_INFO((boost::format("GBFS ----- Free bikes to update: %1%") % total_to_update).str());
  LOG_INFO((boost::format("GBFS ----- Free bikes to add: %1%") % locations_to_add.size()).str());
  // add_gbfs_locations(locations_to_add);
  // LOG_INFO("GBFS ----- Dividing pedestrian and bicycle edges");
  // auto bicycle_edges = divide_pedestrian_and_bicycle_edges();
  // LOG_INFO("GBFS ----- Constructing a full graph");
  // construct_full_graph(bicycle_edges);

    std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
  LOG_INFO((boost::format("GBFS ----- Time total: %1%") % std::chrono::duration_cast<std::chrono::seconds>(end_total - begin_total).count()).str());
}

} // namespace gbfs
} // namespace mjolnir
} // namespace valhalla