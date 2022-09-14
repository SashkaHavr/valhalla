#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <string>
#include "valhalla/mjolnir/osmdata.h"

namespace valhalla {
namespace mjolnir {
namespace gbfs {



struct gbfs_graph_builder {
  boost::property_tree::ptree& config;
  const std::vector<std::string>& input_files;
  OSMData osm_data{0};
  std::string tile_dir;
  std::map<baldr::GraphId, size_t> tiles;

  // Temporary files used during tile building
  const std::string ways_file = "ways.bin";
  const std::string way_nodes_file = "way_nodes.bin";
  const std::string nodes_file = "nodes.bin";
  const std::string edges_file = "edges.bin";
  const std::string tile_manifest_file = "tile_manifest.json";
  const std::string access_file = "access.bin";
  const std::string pronunciation_file = "pronunciation.bin";
  const std::string bss_nodes_file = "bss_nodes.bin";
  const std::string cr_from_file = "complex_from_restrictions.bin";
  const std::string cr_to_file = "complex_to_restrictions.bin";
  const std::string new_to_old_file = "new_nodes_to_old_nodes.bin";
  const std::string old_to_new_file = "old_nodes_to_new_nodes.bin";
  const std::string intersections_file = "intersections.bin";
  const std::string shapes_file = "shapes.bin";

  std::string ways_bin;
  std::string way_nodes_bin;
  std::string nodes_bin;
  std::string edges_bin;
  std::string tile_manifest;
  std::string access_bin;
  std::string pronunciation_bin;
  std::string bss_nodes_bin;
  std::string cr_from_bin;
  std::string cr_to_bin;
  std::string new_to_old_bin;
  std::string old_to_new_bin;

  gbfs_graph_builder(boost::property_tree::ptree& original_config,
                   const std::vector<std::string>& input_files) :
                   config(original_config),
                   input_files(input_files) {}

  bool build();

  void init();
  void parse_ways();
  void parse_relations();
  void parse_nodes();
  void construct_edges();

  void fetch_gbfs_data();
};



struct gbfs_urls {
  
};


} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs
