#include "valhalla/loki/worker.h"
#include "valhalla/loki/search.h"
#include "valhalla/baldr/pathlocation.h"
#include "valhalla/mjolnir/gbfsgraphbuilder.h"

using namespace valhalla::baldr;


namespace valhalla {
namespace loki {


void loki_worker_t::gbfs_route(Api& request) {
  auto& options = *request.mutable_options();
  parse_locations(options.mutable_locations());
  request.mutable_options()->set_costing_type(Costing::multimodal);
  parse_costing(request);
  try {
    // correlate the various locations to the underlying graph
    auto locations = PathLocation::fromPBF(options.locations());
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& projection = projections.at(locations[i]);
      PathLocation::toPBF(projection, options.mutable_locations(i), *reader);
    }
  } 
  catch (const std::exception& ex) {
    LOG_ERROR(ex.what());
    throw valhalla_exception_t{171}; 
  }
}

void loki_worker_t::parse_stations(Api& request) {
  auto& options = *request.mutable_options();
  parse_locations(options.mutable_locations());
  request.mutable_options()->set_costing_type(Costing::pedestrian);
  parse_costing(request);
  // correlate the various locations to the underlying graph
  LOG_INFO("Parse");
  auto locations = PathLocation::fromPBF(options.locations());
  LOG_INFO("Search");
  const auto projections = loki::Search(locations, *reader, costing);
  LOG_INFO("Fill");
  for (size_t i = 0; i < locations.size(); ++i) {
    auto projection = projections.find(locations[i]);
    if(projection == projections.end()) {
      continue;
    }
    PathLocation::toPBF((*projection).second, options.mutable_locations(i), *reader);
  }
}


} // namespace loki
} // namespace valhalla