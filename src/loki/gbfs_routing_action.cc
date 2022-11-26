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
      LOG_INFO("Received location: " + options.mutable_locations(i)->gbfs_transport_station_id());
      const auto& projection = projections.at(locations[i]);
      PathLocation::toPBF(projection, options.mutable_locations(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }
}


} // namespace loki
} // namespace valhalla