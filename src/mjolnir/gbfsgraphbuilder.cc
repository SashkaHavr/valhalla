#include "valhalla/mjolnir/gbfsgraphbuilder.h"
#include <boost/format.hpp>

namespace valhalla {
namespace mjolnir {
namespace gbfs {

bool gbfs_graph_builder::build(bool parse_osm_first) {
  LOG_INFO("GBFS ----- Build started");

  tile_dir = config.get<std::string>("mjolnir.tile_dir");
  new_to_old_bin = tile_dir + new_to_old_file;
  old_to_new_bin = tile_dir + old_to_new_file;

  if(parse_osm_first) {
    LOG_INFO("GBFS ----- Parse OSM Data");
    build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kInitialize, valhalla::mjolnir::BuildStage::kFilter);
  }
  else {
    LOG_INFO("GBFS ----- Skipped OSM Data parse");
  }


  // LOG_INFO("GBFS ----- Create new nodes");
  // // // // // create_new_nodes(); //REMOVE
  // construct_full_graph();

  LOG_INFO("GBFS ----- Updating access");
  iterate_to_update();

  LOG_INFO("GBFS ----- Validating");
  iterate();

  // LOG_INFO("GBFS ----- Start last build stages");
  build_tile_set(config, input_files, valhalla::mjolnir::BuildStage::kElevation, valhalla::mjolnir::BuildStage::kCleanup);

  // LOG_INFO("GBFS ----- Validating");
  // iterate();

  return true;
}

// void gbfs_graph_builder::init() {
//   // Take out tile_extract and tile_url from property tree as tiles must only use the tile_dir
//   config.get_child("mjolnir").erase("tile_extract");
//   config.get_child("mjolnir").erase("tile_url");
//   config.get_child("mjolnir").erase("traffic_extract");

//   // Get the tile directory (make sure it ends with the preferred separator
//   tile_dir = config.get<std::string>("mjolnir.tile_dir");
//   if (tile_dir.back() != filesystem::path::preferred_separator) {
//     tile_dir.push_back(filesystem::path::preferred_separator);
//   }

//   for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
//     auto level_dir = tile_dir + std::to_string(level.level);
//     if (filesystem::exists(level_dir) && !filesystem::is_empty(level_dir)) {
//       LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
//       filesystem::remove_all(level_dir);
//     }
//   }

//   // check for transit level.
//   auto level_dir = tile_dir + std::to_string(valhalla::baldr::TileHierarchy::GetTransitLevel().level);
//   if (filesystem::exists(level_dir) && !filesystem::is_empty(level_dir)) {
//     LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
//     filesystem::remove_all(level_dir);
//   }

//   // Create the directory if it does not exist
//   filesystem::create_directories(tile_dir);

//   // Set file names
//   ways_bin = tile_dir + ways_file;
//   way_nodes_bin = tile_dir + way_nodes_file;
//   nodes_bin = tile_dir + nodes_file;
//   edges_bin = tile_dir + edges_file;
//   tile_manifest = tile_dir + tile_manifest_file;
//   access_bin = tile_dir + access_file;
//   pronunciation_bin = tile_dir + pronunciation_file;
//   bss_nodes_bin = tile_dir + bss_nodes_file;
//   cr_from_bin = tile_dir + cr_from_file;
//   cr_to_bin = tile_dir + cr_to_file;
//   new_to_old_bin = tile_dir + new_to_old_file;
//   old_to_new_bin = tile_dir + old_to_new_file;
// }

// // OSM data processing

// void gbfs_graph_builder::parse_ways() {
//   // Read the OSM protocol buffer file. Callbacks for ways are defined within the PBFParser class
//   osm_data = PBFGraphParser::ParseWays(config.get_child("mjolnir"), input_files, ways_bin,
//                                        way_nodes_bin, access_bin, pronunciation_bin);
// }

// void gbfs_graph_builder::parse_relations() {
//   // Read the OSM protocol buffer file. Callbacks for relations are defined within the PBFParser
//   // class
//   PBFGraphParser::ParseRelations(config.get_child("mjolnir"), input_files, cr_from_bin, cr_to_bin,
//                                  osm_data);
// }

// void gbfs_graph_builder::parse_nodes() {
//   // Read the OSM protocol buffer file. Callbacks for nodes
//   // are defined within the PBFParser class
//   PBFGraphParser::ParseNodes(config.get_child("mjolnir"), input_files, way_nodes_bin, bss_nodes_bin,
//                              osm_data);
// }

// void gbfs_graph_builder::construct_edges() {
//   tiles = GraphBuilder::BuildEdges(config, ways_bin, way_nodes_bin, nodes_bin, edges_bin);
//   // Output manifest
//   TileManifest manifest{tiles};
//   manifest.LogToFile(tile_manifest);
// }

// void gbfs_graph_builder::build_graph() {
//   // Build the graph using the OSMNodes and OSMWays from the parser
//   GraphBuilder::Build(config, osm_data, ways_bin, way_nodes_bin, nodes_bin, edges_bin, cr_from_bin,
//                       cr_to_bin, pronunciation_bin, tiles);
// }

// void gbfs_graph_builder::enhance() {
//   GraphEnhancer::Enhance(config, osm_data, access_bin);
// }

// void gbfs_graph_builder::filter() {
//   GraphFilter::Filter(config);
// }

// void gbfs_graph_builder::elevation() {
//   ElevationBuilder::Build(config);
// }

// void gbfs_graph_builder::cleanup() {
//   auto remove_temp_file = [](const std::string& fname) {
//     if (filesystem::exists(fname)) {
//       filesystem::remove(fname);
//     }
//   };

//   LOG_INFO("Cleaning up temporary *.bin files within " + tile_dir);
//   remove_temp_file(ways_bin);
//   remove_temp_file(way_nodes_bin);
//   remove_temp_file(nodes_bin);
//   remove_temp_file(edges_bin);
//   remove_temp_file(access_bin);
//   remove_temp_file(pronunciation_bin);
//   remove_temp_file(bss_nodes_bin);
//   remove_temp_file(cr_from_bin);
//   remove_temp_file(cr_to_bin);
//   remove_temp_file(new_to_old_bin);
//   remove_temp_file(old_to_new_bin);
//   remove_temp_file(tile_manifest);
//   OSMData::cleanup_temp_files(tile_dir);
// }



// build foot + bike networks

// void construct_base_networks() {
//   GraphReader reader(pt.get_child("mjolnir"));
//   auto local_tiles = reader.GetTileSet();
//   for (const auto& base_tile_id : local_tiles) {
//     // Get the graph tile. Skip if no tile exists or no nodes exist in the tile.
//     graph_tile_ptr tile = reader.GetGraphTile(base_tile_id);
//     if (!tile) {
//       continue;
//     }

//     // Iterate through the nodes. Add nodes to the new level when
//     // best road class <= the new level classification cutoff
//     // 0 - foot, 1 - bike
//     uint32_t nodecount = tile->header()->nodecount();
//     GraphId basenode = base_tile_id;
//     GraphId edgeid = base_tile_id;
//     PointLL base_ll = tile->header()->base_ll();
//     const NodeInfo* nodeinfo = tile->node(basenode);
//     for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, ++basenode) {
//       // Iterate through the edges to see which levels this node exists.


//       for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, ++edgeid) {
//         // Update the flag for the level of this edge (skip transit
//         // connection edges)
//         const DirectedEdge* directed_edge = tile->directededge(edgeid);
//         const uint32_t new_edge_id = tile_builder.directededges().size();

//         if (directededge->forwardaccess() && kBicycleAccess == kBicycleAccess) {
//           create_new_edge(edge, tile->edgeinfo(directed_edge)
//         }
//         if (directededge->forwardaccess() && kPedestrianAccess != kPedestrianAccess) {}
//       }
//     }
//   }

//   DirectedEdge copy_edge(DirectedEdge source_edge, const GraphId endnode, const uint32_t new_edge_id) {
//     DirectedEdge new_edge;
//     directededge.set_endnode(endnode);
//     new_edge.set_localedgeidx(new_edge_id);

//     return new_edge;
//   }

//   void set_edge_mode(DirectedEdge* edge, uint32_t mode) {
//     new_edge.set_forwardaccess(mode);
//     new_edge.set_reverseaccess(mode);
//   }

//   void add_edge() {

//   }

//   void copy_node() {

//   }

//   void add_node() {

//   }

  void gbfs_graph_builder::create_new_nodes() {
    GraphReader reader(config.get_child("mjolnir"));
    std::unordered_map<GraphId, uint32_t> new_nodes;

    // lambda to get the next "new" node Id in a given tile
    auto get_new_node = [&new_nodes](const GraphId& tile) -> GraphId {
      auto itr = new_nodes.find(tile);
      if (itr == new_nodes.end()) {
        GraphId new_node(tile.tileid(), tile.level(), 0);
        new_nodes[tile] = 1;
        return new_node;
      } else {
        GraphId new_node(tile.tileid(), tile.level(), itr->second);
        itr->second++;
        return new_node;
      }
    };

    // Create a sequence to associate new nodes to old nodes
    // sequence<std::pair<GraphId, GraphId>> new_to_old(new_to_old_file, true);

    // Create a sequence to associate new nodes to old nodes
    // sequence<OldToNewNodes> old_to_new(old_to_new_file, true);

    sequence<new_to_old_node> new_to_old(new_to_old_bin, true); //TODO create new to old for foot and bike (create struct with foot/bike field)
    sequence<old_to_new_node> old_to_new(old_to_new_bin, true);

    // Iterate through all tiles in the local level
    auto local_tiles = reader.GetTileSet();
    for (const auto& base_tile_id : local_tiles) {

      // Get the graph tile. Skip if no tile exists or no nodes exist in the tile.
      graph_tile_ptr tile = reader.GetGraphTile(base_tile_id);
      if (!tile) {
        continue;
      }

      // Iterate through the nodes.
      // Add to bike if bike edge exists, to pedestrian if pedestrian exists
      bool levels[2];
      uint32_t nodecount = tile->header()->nodecount();
      GraphId basenode = base_tile_id;
      GraphId edgeid = base_tile_id;
      PointLL base_ll = tile->header()->base_ll();
      const NodeInfo* nodeinfo = tile->node(basenode);
      for (uint32_t i = 0; i < nodecount; i++, nodeinfo++, ++basenode) {
        // Iterate through the edges to see which levels this node exists.
        levels[0] = levels[1] = false;
        for (uint32_t j = 0; j < nodeinfo->edge_count(); j++, ++edgeid) {
          // Update the flag for the level of this edge (skip transit
          // connection edges)
          const DirectedEdge* directededge = tile->directededge(edgeid);
          levels[0] = levels[0] || ((directededge->forwardaccess() & kPedestrianAccess) == kPedestrianAccess);
          levels[1] = levels[1] || ((directededge->forwardaccess() & kBicycleAccess) == kBicycleAccess);
        }

        if (!levels[0] && !levels[1]) {
          // LOG_ERROR("No valid level for this node!");
          // continue;
        }

        // Associate new nodes to base nodes and base node to new nodes
        GraphId pedestrian_node;
        GraphId bike_node;
        if (levels[0]) {
          // New node is on the local level. Associate back to base/local node
          pedestrian_node = get_new_node(base_tile_id);
          new_to_old_node n(basenode, pedestrian_node, kPedestrianAccess);
          new_to_old.push_back(n);
        }
        if (levels[1]) {
          // New node is on the local level. Associate back to base/local node
          bike_node = get_new_node(base_tile_id);
          new_to_old_node n(basenode, bike_node, kBicycleAccess);
          new_to_old.push_back(n);
        }



        // Associate the old node to the new node(s). Entries in the tuple
        // that are invalid nodes indicate no node exists in the new level.
        old_to_new_node assoc(basenode, pedestrian_node, bike_node, nodeinfo->density());
        old_to_new.push_back(assoc);
      }

      // Sort old_to_new by node id
      old_to_new.sort([](const old_to_new_node& a, const old_to_new_node& b) { return a.base_node < b.base_node; });

      // Check if we need to clear the tile cache
      if (reader.OverCommitted()) {
        reader.Trim();
      }
    }












    //Form new graph




    reader.Clear();
    bool added = false;
    uint8_t current_level = std::numeric_limits<uint8_t>::max();
    GraphId tile_id;
    std::hash<std::string> hasher;
    PointLL base_ll;
    GraphTileBuilder* tilebuilder = nullptr;
    for (auto new_node = new_to_old.begin(); new_node != new_to_old.end(); new_node++) {

      if((*new_node).access == 0) {
        continue;
      }

      // Get the node - check if a new tile
      GraphId nodea = (*new_node).new_node;
      if (nodea.Tile_Base() != tile_id) {
        // Store the prior tile
        if (tilebuilder != nullptr) {
          tilebuilder->StoreTileData();
          delete tilebuilder;
        }

        // New tilebuilder for the next tile. Update current level.
        tile_id = nodea.Tile_Base();
        tilebuilder = new GraphTileBuilder(tile_dir, tile_id, false);
        current_level = nodea.level();

        // Set the base ll for this tile
        base_ll = TileHierarchy::get_tiling(current_level).Base(tile_id.tileid());
        tilebuilder->header_builder().set_base_ll(base_ll);

        // Check if we need to clear the base/local tile cache
        if (reader.OverCommitted()) {
          reader.Trim();
        }
      }

      // Get the node in the base level
      GraphId base_node = (*new_node).base_node;
      graph_tile_ptr tile = reader.GetGraphTile(base_node);
      if (tile == nullptr) {
        LOG_ERROR("Base tile is null? ");
        continue;
      }

      // Copy the data version
      tilebuilder->header_builder().set_dataset_id(tile->header()->dataset_id());

      // Copy node information and set the node lat,lon offsets within the new tile
      NodeInfo baseni = *(tile->node(base_node.id()));
      tilebuilder->nodes().push_back(baseni);
      const auto& admin = tile->admininfo(baseni.admin_index());
      NodeInfo& node = tilebuilder->nodes().back();
      node.set_latlng(base_ll, baseni.latlng(tile->header()->base_ll()));
      node.set_edge_index(tilebuilder->directededges().size());
      node.set_timezone(baseni.timezone());
      node.set_admin_index(tilebuilder->AddAdmin(admin.country_text(), admin.state_text(),
                                                 admin.country_iso(), admin.state_iso()));

      // Update node LL based on tile base
      // Density at this node
      uint32_t density1 = baseni.density();

      // Current edge count
      size_t edge_count = tilebuilder->directededges().size();

      // Iterate through directed edges of the base node to get remaining
      // directed edges (based on classification/importance cutoff)
      GraphId base_edge_id(base_node.tileid(), base_node.level(), baseni.edge_index());
      for (uint32_t i = 0; i < baseni.edge_count(); i++, ++base_edge_id) {
        // Check if the directed edge should exist on this level
        const DirectedEdge* directededge = tile->directededge(base_edge_id);

        // Copy the directed edge information
        DirectedEdge newedge = *directededge;

        // Set the end node for this edge. Transit connection edges
        // remain connected to the same node on the transit level.
        // Need to set nodeb for use in AddEdgeInfo
        uint32_t density2 = 32;
        GraphId nodeb;
        auto new_nodes = find_nodes(old_to_new, directededge->endnode()); //TODO copy find method
        if((*new_node).access == kPedestrianAccess) {
          nodeb = new_nodes.pedestrian_node;
        }
        else if((*new_node).access == kBicycleAccess) {
          nodeb = new_nodes.bike_node;
        }
        density2 = new_nodes.density;
        if (!nodeb.Is_Valid()) {
          LOG_ERROR("Invalid end node - not found in old_to_new map");
        }
        newedge.set_endnode(nodeb);

        //TODO set forward backward access according to the network
        newedge.set_forwardaccess((*new_node).access);
        newedge.set_reverseaccess((*new_node).access);

        // Set the edge density  to the average of the relative density at the
        // end nodes.
        uint32_t edge_density = (density2 == 32) ? density1 : (density1 + density2) / 2;
        newedge.set_density(edge_density);

        // Set opposing edge indexes to 0 (gets set in graph validator).
        newedge.set_opp_index(0);

        // Get signs from the base directed edge
        if (directededge->sign()) {
          std::vector<SignInfo> signs = tile->GetSigns(base_edge_id.id());
          if (signs.size() == 0) {
            LOG_ERROR("Base edge should have signs, but none found");
          }
          tilebuilder->AddSigns(tilebuilder->directededges().size(), signs);
        }

        // Get turn lanes from the base directed edge
        if (directededge->turnlanes()) {
          uint32_t offset = tile->turnlanes_offset(base_edge_id.id());
          tilebuilder->AddTurnLanes(tilebuilder->directededges().size(), tile->GetName(offset));
        }

        // Get access restrictions from the base directed edge. Add these to
        // the list of access restrictions in the new tile. Update the
        // edge index in the restriction to be the current directed edge Id
        if (directededge->access_restriction()) {
          auto restrictions = tile->GetAccessRestrictions(base_edge_id.id(), kAllAccess);
          for (const auto& res : restrictions) {
            tilebuilder->AddAccessRestriction(AccessRestriction(tilebuilder->directededges().size(),
                                                                res.type(), res.modes(), res.value()));
          }
        }

        // Copy lane connectivity
        if (directededge->laneconnectivity()) {
          auto laneconnectivity = tile->GetLaneConnectivity(base_edge_id.id());
          if (laneconnectivity.size() == 0) {
            LOG_ERROR("Base edge should have lane connectivity, but none found");
          }
          for (auto& lc : laneconnectivity) {
            lc.set_to(tilebuilder->directededges().size());
          }
          tilebuilder->AddLaneConnectivity(laneconnectivity);
        }

        // Do we need to force adding edgeinfo (opposing edge could have diff names)?
        // If end node is in the same tile and there is no opposing edge with matching
        // edge_info_offset).
        bool diff_names = directededge->endnode().tileid() == base_edge_id.tileid() &&
                          !OpposingEdgeInfoMatches(tile, directededge);

        // Get edge info, shape, and names from the old tile and add to the
        // new. Cannot use edge info offset since edges in arterial and
        // highway hierarchy can cross base tiles! Use a hash based on the
        // encoded shape plus way Id.
        auto edgeinfo = tile->edgeinfo(directededge);
        std::string encoded_shape = edgeinfo.encoded_shape();
        uint32_t w = hasher(encoded_shape + std::to_string(edgeinfo.wayid()));
        uint32_t edge_info_offset =
            tilebuilder->AddEdgeInfo(w, nodea, nodeb, edgeinfo.wayid(), edgeinfo.mean_elevation(),
                                     edgeinfo.bike_network(), edgeinfo.speed_limit(), encoded_shape,
                                     edgeinfo.GetNames(), edgeinfo.GetTaggedValues(),
                                     edgeinfo.GetTaggedValues(true), edgeinfo.GetTypes(), added,
                                     diff_names);

        newedge.set_edgeinfo_offset(edge_info_offset);

        // Add directed edge
        tilebuilder->directededges().emplace_back(std::move(newedge));
      }

      // Add node transitions  //TODO replace this with creating edges from/to bike/pedestrian networks
      // auto new_nodes = find_nodes(old_to_new, base_node);
      // if(new_nodes.bike_node.Is_Valid() && new_nodes.pedestrian_node.Is_Valid()) {
      //   make_network_connection_edge(new_nodes.pedestrian_node, new_nodes.bike_node, tilebuilder);
      //   make_network_connection_edge(new_nodes.bike_node, new_nodes.pedestrian_node, tilebuilder);
      // }

      // Set the edge count for the new node
      node.set_edge_count(tilebuilder->directededges().size() - edge_count);

      // Get named signs from the base node
      if (baseni.named_intersection()) {
        std::vector<SignInfo> signs = tile->GetSigns(base_node.id(), true);
        if (signs.size() == 0) {
          LOG_ERROR("Base node should have signs, but none found");
        }
        node.set_named_intersection(true);
        tilebuilder->AddSigns(tilebuilder->nodes().size() - 1, signs);
      }
    }

    // Delete the tile builder
    if (tilebuilder != nullptr) {
      tilebuilder->StoreTileData();
      delete tilebuilder;
    }
  }

  old_to_new_node gbfs_graph_builder::find_nodes(sequence<old_to_new_node>& old_to_new, const GraphId& node) {
    GraphId dmy;
    old_to_new_node target(node, dmy, dmy, 0);
    auto iter = old_to_new.find(target, [](const old_to_new_node& a, const old_to_new_node& b) {
      return a.base_node < b.base_node;
    });
    if (iter == old_to_new.end()) {
      throw std::runtime_error("Didn't find node!");
    } else {
      return *iter;
    }
  }

  DirectedEdge gbfs_graph_builder::make_network_connection_edge(GraphId start_node, GraphId end_node, GraphTileBuilder* tile_builder) {
    DirectedEdge directed_edge;
    directed_edge.set_endnode(end_node);
    directed_edge.set_length(0);
    directed_edge.set_use(Use::kNetworkConnection);
    directed_edge.set_speed(0);
    directed_edge.set_surface(Surface::kPavedSmooth);
    directed_edge.set_cyclelane(CycleLane::kNone);
    directed_edge.set_classification(valhalla::baldr::RoadClass::kUnclassified);
    directed_edge.set_localedgeidx(tile_builder->directededges().size());
  
    directed_edge.set_forwardaccess(kPedestrian);
    directed_edge.set_reverseaccess(kPedestrian);
  
    directed_edge.set_named(false);
    directed_edge.set_forward(true);

    bool b = true;
    std::string s = "";
    uint32_t edge_info_offset =
        tile_builder->AddEdgeInfo(tile_builder->directededges().size(), start_node,
                                      end_node, 0, 0, 0, 0, s,
                                      {}, {}, {}, 0,
                                      b);

    directed_edge.set_edgeinfo_offset(edge_info_offset);

    tile_builder->directededges().emplace_back(std::move(directed_edge));

    return directed_edge;
  }

  /**
 * Is there an opposing edge with matching edgeinfo offset. The end node of the directed edge
 * must be in the same tile as the directed edge.
 * @param  tile          Graph tile of the edge
 * @param  directededge  Directed edge to match.
 */
bool gbfs_graph_builder::OpposingEdgeInfoMatches(const graph_tile_ptr& tile, const DirectedEdge* edge) {
  // Get the nodeinfo at the end of the edge. Iterate through the directed edges and return
  // true if a matching edgeinfo offset if found.
  const NodeInfo* nodeinfo = tile->node(edge->endnode().id());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // Return true if the edge info matches (same name, shape, etc.)
    if (directededge->edgeinfo_offset() == edge->edgeinfo_offset()) {
      return true;
    }
  }
  return false;
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





































  void gbfs_graph_builder::construct_full_graph() {
    std::unordered_map<baldr::GraphId, baldr::GraphId> old_to_new;
    GraphReader reader(config.get_child("mjolnir"));


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

        // Iterate through directed edges outbound from this node
        // std::vector<uint64_t> wayid; //REMOVE
        // std::vector<GraphId> endnode;
        const NodeInfo* nodeinfo = tile->node(nodeid);
        GraphId edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
        for (uint32_t j = 0; j < nodeinfo->edge_count(); ++j, ++edgeid) {
          // Check if the directed edge should be included
          const DirectedEdge* directededge = tile->directededge(edgeid);
          // if (!include_edge(directededge)) { //COUNTS //TODO
          //   ++n_filtered_edges;
          //   continue;
          // }

          copy_edge(directededge, edgeid, tile, tilebuilder, edge_count, nodeid);

        }

        // Add the node to the tilebuilder unless no edges remain
        if (edge_count > 0) {
          GraphId new_node = copy_node(nodeid, nodeinfo, tile, tilebuilder, edge_count, edge_index);

          // Associate the old node to the new node.
          old_to_new[nodeid] = new_node;

          // Check if edges at this node can be aggregated. Only 2 edges, same way Id (so that
          // edge attributes should match), don't end at same node (no loops).
          // if (edge_count == 2 && wayid[0] == wayid[1] && endnode[0] != endnode[1]) { //REMOVE
          //   ++can_aggregate;
          // }
        } else {
          // ++n_filtered_nodes; //COUNTS
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
        LOG_INFO("Remove file: " + file_location + " all edges were filtered");
      }

      if (reader.OverCommitted()) {
        reader.Trim();
      }
    }






    //update end nodes in edges

    reader.Clear();
    LOG_INFO("Update end nodes of directed edges");
    int found = 0;

    // Iterate through all tiles in the local level
    local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Get the graph tile. Skip if no tile exists (should not happen!?)
      graph_tile_ptr tile = reader.GetGraphTile(tile_id);
      assert(tile);

      // Create a new tilebuilder - should copy header information
      GraphTileBuilder tilebuilder(tile_dir, tile_id, false);

      // Copy nodes (they do not change)
      std::vector<NodeInfo> nodes;
      size_t n = tile->header()->nodecount();
      nodes.reserve(n);
      const NodeInfo* orig_nodes = tile->node(0);
      std::copy(orig_nodes, orig_nodes + n, std::back_inserter(nodes));

      // Iterate through all directed edges - update end nodes
      std::vector<DirectedEdge> directededges;
      GraphId edgeid(tile_id.tileid(), tile_id.level(), 0);
      for (uint32_t j = 0; j < tile->header()->directededgecount(); ++j, ++edgeid) {
        const DirectedEdge* edge = tile->directededge(j);

        // Find the end node in the old_to_new mapping
        GraphId end_node;
        auto iter = old_to_new.find(edge->endnode());
        if (iter == old_to_new.end()) {
          LOG_ERROR("UpdateEndNodes - failed to find associated node");
        } else {
          end_node = iter->second;
          found++;
        }

        // Copy the edge to the directededges vector and update the end node
        directededges.push_back(*edge);
        DirectedEdge& new_edge = directededges.back();
        new_edge.set_endnode(end_node);
      }

      // Update the tile with new directededges.
      tilebuilder.Update(nodes, directededges);

      if (reader.OverCommitted()) {
        reader.Trim();
      }
    }
  }

  void gbfs_graph_builder::iterate() {
    GraphReader reader(config.get_child("mjolnir"));
    int count = 0;
    auto local_tiles = reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Get the graph tile. Skip if no tile exists (should not happen!?)
      graph_tile_ptr tile = reader.GetGraphTile(tile_id);
      assert(tile);

      GraphId edgeid(tile_id.tileid(), tile_id.level(), 0);
      for (uint32_t j = 0; j < tile->header()->directededgecount(); ++j, ++edgeid) {
        const DirectedEdge* edge = tile->directededge(j);

          if(edge->forwardaccess() != kPedestrianAccess || edge->reverseaccess() != kPedestrianAccess) {
            LOG_INFO((boost::format("Access: %1%, %2%") % edge->forwardaccess() % edge->reverseaccess()).str());
            throw std::exception();
          }

          count++;
      }
    }
    LOG_INFO((boost::format("GBFS ----- Edges checked: %1%") % count).str());
  }

  void gbfs_graph_builder::iterate_to_update() {
    GraphReader graph_reader(config.get_child("mjolnir"));
    int count = 0;
    auto local_tiles = graph_reader.GetTileSet(TileHierarchy::levels().back().level);
    for (const auto& tile_id : local_tiles) {
      // Skip if no nodes exist in the tile
      graph_tile_ptr tile = graph_reader.GetGraphTile(tile_id);
      if (!tile) {
        continue;
      }
  
      // Create a new tile builder
      GraphTileBuilder tilebuilder(tile_dir, tile_id, false);
  
      // Update end nodes of transit connection directed edges
      std::vector<NodeInfo> nodes;
      std::vector<DirectedEdge> directededges;
      for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
        NodeInfo nodeinfo = tilebuilder.node(i);
        uint32_t idx = nodeinfo.edge_index();
        for (uint32_t j = 0; j < nodeinfo.edge_count(); j++, idx++) {
          DirectedEdge directededge = tilebuilder.directededge(idx);
  
          directededge.set_forwardaccess(kPedestrianAccess);
          directededge.set_reverseaccess(kPedestrianAccess);
  
          // Add the directed edge to the local list
          directededges.emplace_back(std::move(directededge));
          count++;
        }
  
        // Add the node to the local list
        nodes.emplace_back(std::move(nodeinfo));
      }
      tilebuilder.Update(nodes, directededges);
    }
    LOG_INFO((boost::format("GBFS ----- Edges updated: %1%") % count).str());
  }


  DirectedEdge& gbfs_graph_builder::copy_edge(const DirectedEdge* directededge, GraphId& edgeid, 
                                                graph_tile_ptr& tile, GraphTileBuilder& tilebuilder, uint32_t& edge_count, GraphId& nodeid) {
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
    ++edge_count;
    return tilebuilder.directededges().back();
  }

  GraphId gbfs_graph_builder::copy_node(GraphId& nodeid, const NodeInfo* nodeinfo, graph_tile_ptr& tile, 
                                          GraphTileBuilder& tilebuilder, uint32_t edge_count, uint32_t edge_index) {
    // Add a node builder to the tile. Update the edge count and edgeindex
    GraphId new_node(nodeid.tileid(), nodeid.level(), tilebuilder.nodes().size());
    tilebuilder.nodes().push_back(*nodeinfo);
    NodeInfo& node = tilebuilder.nodes().back();
    node.set_edge_count(edge_count);
    node.set_edge_index(edge_index);
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
    return new_node;
  }



} // namespace gbfs
} // namespace mjolnir
} // namespace valhalla