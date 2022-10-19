#include "valhalla/mjolnir/gbfsgraphbuilder.h"
#include <boost/format.hpp>

namespace valhalla {
namespace mjolnir {
namespace gbfs {

bool gbfs_graph_builder::build(bool parse_osm_first) {
  LOG_INFO("GBFS ----- Build started");
  if(parse_osm_first) {
    LOG_INFO("GBFS ----- First standard building stages: start - Initialize, end - Filter");
    build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kInitialize, valhalla::mjolnir::BuildStage::kFilter);
  }
  else {
    LOG_INFO("GBFS ----- Skipped first standard building stages");
  }


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

  LOG_INFO("GBFS ----- Validating");
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

  LOG_INFO("GBFS ----- Last standard building stages: start - Elevation, end - Cleanup");
  build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kElevation, valhalla::mjolnir::BuildStage::kCleanup);

  return true;
}


// Build foot + bike networks

  void gbfs_graph_builder::construct_full_graph(std::unordered_map<baldr::GraphId, std::vector<bicycle_edge>>& nodes_to_bicycle_edges) {
    std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new_pedestrian;
    std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new_bicycle;
    GraphReader reader(config.get_child("mjolnir"));

    int total_bicycle = 0;
    int total_pedestrian = 0;
    int total_transitions = 0;

    auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Create a new tilebuilder - should copy header information
      GraphTileBuilder tilebuilder(tile_dir, tile_id, false);

      // Get the graph tile. Read from this tile to create the new tile.
      graph_tile_ptr tile = reader.GetGraphTile(tile_id);
      assert(tile);
      
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

        GraphId new_pedestrian_node_id;
        // Add the node to the tilebuilder unless no edges remain
        if (edge_count > 0) {
          copy_node(nodeid, nodeinfo, tile, tilebuilder, edge_count, edge_index);
          new_pedestrian_node_id = GraphId(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size() - 1);
          old_to_new_pedestrian[nodeid] = new_pedestrian_node_id;
        }
        total_pedestrian += edge_count;

        // Count of edges added for this node
        edge_count = 0;
        edge_index = tilebuilder.directededges().size();
        auto edges = nodes_to_bicycle_edges[nodeid];
        //Create nodes and edges in bicycle network
        for (uint32_t j = 0; j < edges.size(); ++j) {
          // Check if the directed edge should be included
          bicycle_edge bicycle_edge = edges[j];
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

        GraphId new_bicycle_node_id;
        NodeInfo* new_bicycle_node;
        // Add the node to the tilebuilder unless no edges remain
        if (edge_count > 0) {
          new_bicycle_node = &copy_node(nodeid, nodeinfo, tile, tilebuilder, edge_count, edge_index);
          new_bicycle_node_id = GraphId(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size() - 1);
          old_to_new_bicycle[nodeid] = new_bicycle_node_id;
        }
        total_bicycle += edge_count;
        
        // Add transitions from all bicycle nodes to corresponding pedestrian nodes
        if(new_pedestrian_node_id.Is_Valid() && new_bicycle_node_id.Is_Valid()) {
          new_bicycle_node->set_transition_index(tilebuilder.transitions().size());
          tilebuilder.transitions().emplace_back(new_pedestrian_node_id, false);
          new_bicycle_node->set_transition_count(1);
          total_transitions++;
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
        throw new std::exception();
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
    auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Get the graph tile. Skip if no tile exists (should not happen!?)
      graph_tile_ptr tile = reader.GetGraphTile(tile_id);
      assert(tile);

      for (uint32_t i = 0; i < tile->header()->nodecount(); i++) {
        const NodeInfo* nodeinfo = tile->node(i);
        uint32_t idx = nodeinfo->edge_index();
        for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, idx++) {
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
    auto local_tiles = graph_reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Skip if no nodes exist in the tile
      graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);
      assert(tile);
  
      // Create a new tile builder
      GraphTileBuilder tilebuilder(tile_dir, tile_id, false);
  
      // Update end nodes of transit connection directed edges
      std::vector<NodeInfo> nodes;
      std::vector<DirectedEdge> directededges;
      GraphId node_id(tile_id.tileid(), tile_id.level(), 0);
      for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++, ++node_id) {
        NodeInfo& nodeinfo = tilebuilder.node(i);
        uint32_t idx = nodeinfo.edge_index();
        GraphId edge_id(tile_id.tileid(), tile_id.level(), idx);
        for (uint32_t j = 0; j < nodeinfo.edge_count(); j++, idx++, ++edge_id) {
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


  DirectedEdge& gbfs_graph_builder::copy_edge(const DirectedEdge* directededge, GraphId& edgeid, 
                                                graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, GraphId& nodeid) {
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

  NodeInfo& gbfs_graph_builder::copy_node(GraphId& nodeid, const NodeInfo* nodeinfo, graph_tile_ptr& tile, 
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





  // GBFS data processing

  void gbfs_graph_builder::fetch_gbfs_data() {
    valhalla::baldr::curler_pool_t curlers(1, "");
    valhalla::baldr::scoped_curler_t curler(curlers);
    long http_code = 0;
    const std::function<void()>* interrupt = nullptr;
    std::string url = "https://gbfs.nextbike.net/maps/gbfs/v2/nextbike_nm/gbfs.json";
    boost::property_tree::ptree gbfs_json;
    char* response = &(curler.get()(url, http_code, false, interrupt)[0]);

    rapidjson::Document gbfs_data;
    gbfs_data.Parse(response);
    auto& urls = gbfs_data["data"]["en"]["feeds"];
    for (auto& v : urls.GetArray()) {
      LOG_INFO(v["name"].GetString());
    }
  }

} // namespace gbfs
} // namespace mjolnir
} // namespace valhalla