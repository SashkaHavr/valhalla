#include <string>
#include <vector>

#include "config.h"

#include "baldr/rapidjson_utils.h"
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>
#include <iostream>

#include "filesystem.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include "valhalla/mjolnir/gbfsgraphbuilder.h"
using namespace valhalla::mjolnir::gbfs;


int main(int argc, char** argv) {
  // args
  filesystem::path config_file_path;
  std::vector<std::string> input_files;
  boost::property_tree::ptree pt;

  try {

    // ref:
    // https://github.com/jarro2783/cxxopts/blob/302302b30839505703d37fb82f536c53cf9172fa/src/example.cpp
    cxxopts::Options options(
        "valhalla_build_tiles",
        "valhalla_build_tiles " VALHALLA_VERSION
        "\n\nvalhalla_build_tiles is a program that creates the route graph\nfrom one or multiple osm.pbf extract(s)\n");

    // clang-format off
    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version","Print the version of this software.")
      ("c,config", "Path to the configuration file", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files));
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("OSM PBF file(s)");
    auto result = options.parse(argc, argv);

    if (result.count("version")) {
      std::cout << "valhalla_build_gbfs_tiles " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    // Read the config file
    if (result.count("inline-config")) {
      std::stringstream ss;
      ss << result["inline-config"].as<std::string>();
      rapidjson::read_json(ss, pt);
    } else if (result.count("config") &&
               filesystem::is_regular_file(
                   config_file_path = filesystem::path(result["config"].as<std::string>()))) {
      rapidjson::read_json(config_file_path.string(), pt);
    } else {
      std::cerr << "Configuration is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }

    // configure logging
    boost::optional<boost::property_tree::ptree&> logging_subtree =
        pt.get_child_optional("mjolnir.logging");
    if (logging_subtree) {
      auto subtree = logging_subtree.get();
      auto logging_config =
          valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                   std::unordered_map<std::string, std::string>>(subtree);
      valhalla::midgard::logging::Configure(logging_config);
    }

    LOG_INFO("Building tiles for gbfs search...");

    if (!result.count("input_files")) {
      std::cerr << "Input file is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
  }

  // Build some tiles!
  gbfs_graph_builder g(pt, input_files);
  if (g.build()) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}
