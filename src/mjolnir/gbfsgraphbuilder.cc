#include "valhalla/mjolnir/gbfsgraphbuilder.h"
#include "valhalla/mjolnir/util.h"

#include "baldr/curler.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"
#include "rapidjson/document.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/graphbuilder.h"

namespace valhalla {
namespace mjolnir {
namespace gbfs {

bool gbfs_graph_builder::build() {
  LOG_INFO("Build started");

  // init();

  // fetch_gbfs_data();

  return true;
}

void gbfs_graph_builder::init() {
  // Take out tile_extract and tile_url from property tree as tiles must only use the tile_dir
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");
  config.get_child("mjolnir").erase("traffic_extract");

  // Get the tile directory (make sure it ends with the preferred separator
  tile_dir = config.get<std::string>("mjolnir.tile_dir");
  if (tile_dir.back() != filesystem::path::preferred_separator) {
    tile_dir.push_back(filesystem::path::preferred_separator);
  }

  for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
    auto level_dir = tile_dir + std::to_string(level.level);
    if (filesystem::exists(level_dir) && !filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      filesystem::remove_all(level_dir);
    }
  }

  // check for transit level.
  auto level_dir = tile_dir + std::to_string(valhalla::baldr::TileHierarchy::GetTransitLevel().level);
  if (filesystem::exists(level_dir) && !filesystem::is_empty(level_dir)) {
    LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
    filesystem::remove_all(level_dir);
  }

  // Create the directory if it does not exist
  filesystem::create_directories(tile_dir);

  // Set file names
  ways_bin = tile_dir + ways_file;
  way_nodes_bin = tile_dir + way_nodes_file;
  nodes_bin = tile_dir + nodes_file;
  edges_bin = tile_dir + edges_file;
  tile_manifest = tile_dir + tile_manifest_file;
  access_bin = tile_dir + access_file;
  pronunciation_bin = tile_dir + pronunciation_file;
  bss_nodes_bin = tile_dir + bss_nodes_file;
  cr_from_bin = tile_dir + cr_from_file;
  cr_to_bin = tile_dir + cr_to_file;
  new_to_old_bin = tile_dir + new_to_old_file;
  old_to_new_bin = tile_dir + old_to_new_file;
}

//OSM data processing

void gbfs_graph_builder::parse_ways() {
  // Read the OSM protocol buffer file. Callbacks for ways are defined within the PBFParser class
  osm_data = PBFGraphParser::ParseWays(config.get_child("mjolnir"), input_files, ways_bin,
                                       way_nodes_bin, access_bin, pronunciation_bin);
}

void gbfs_graph_builder::parse_relations() {
  // Read the OSM protocol buffer file. Callbacks for relations are defined within the PBFParser
  // class
  PBFGraphParser::ParseRelations(config.get_child("mjolnir"), input_files, cr_from_bin, cr_to_bin,
                                 osm_data);
}

void gbfs_graph_builder::parse_nodes() {
  // Read the OSM protocol buffer file. Callbacks for nodes
  // are defined within the PBFParser class
  PBFGraphParser::ParseNodes(config.get_child("mjolnir"), input_files, way_nodes_bin, bss_nodes_bin,
                             osm_data);
}

void gbfs_graph_builder::construct_edges() {
  tiles = GraphBuilder::BuildEdges(config, ways_bin, way_nodes_bin, nodes_bin, edges_bin);
  // Output manifest
  TileManifest manifest{tiles};
  manifest.LogToFile(tile_manifest);
}

//GBFS data processing

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