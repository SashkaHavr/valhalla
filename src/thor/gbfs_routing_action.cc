#include "valhalla/thor/worker.h"
#include <boost/format.hpp>
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"

namespace valhalla {
namespace thor {

std::string empty_response = "{\"routes\":[]}";

std::string thor_worker_t::gbfs_route(Api& request) {
  auto& options = *request.mutable_options();
  if(options.locations().empty()) {
    return empty_response;
  }
  adjust_scores(options);
  auto costing = parse_costing(request);

  std::unordered_map<uint64_t, std::vector<valhalla::Location>> target_edges;
  std::unordered_map<baldr::GraphId, std::vector<valhalla::Location>> tileid_to_location;
  for(const auto& location : options.targets()) {
    baldr::GraphId graphid = baldr::TileHierarchy::GetGraphId(PointLL(location.ll().lng(), location.ll().lat()), baldr::TileHierarchy::levels().back().level);
    tileid_to_location[graphid].push_back(location);
  }

  for(const auto& tile_to_l : tileid_to_location) {
    graph_tile_ptr tile = reader->GetGraphTile(tile_to_l.first);
    if(tile == nullptr) {
      continue;
    }
    for(const auto& location : tile_to_l.second) {
      std::vector<public_transport_station_projection> projections;
      std::copy_if(tile->station_projections().begin(), tile->station_projections().end(), std::back_inserter(projections), [&](const public_transport_station_projection& p) {
        return p.station_id == location.gbfs_transport_station_id();
      });
      if(projections.empty()) {
        continue;
      }
      // LOG_INFO((boost::format("GBFS ----- Projections count: %1%") % projections.size()).str());
      for(const public_transport_station_projection& proj : projections) {
        target_edges[baldr::GraphId(tile_to_l.first.tileid(), tile_to_l.first.level(), proj.edge_id).value].push_back(location);
      }
    }
  }

  // LOG_INFO((boost::format("GBFS ----- Received start locations: %1%") % options.locations().size()).str());
  // LOG_INFO((boost::format("GBFS ----- Found target edges: %1%") % target_edges.size()).str());

  // LOG_INFO((boost::format("GBFS ----- Got dir: %1%") % options.is_forward()).str());
  // LOG_INFO((boost::format("GBFS ----- Gor max time: %1%") % options.gbfs_max_duration()).str());

  // std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
  auto res = gbfs_router.Expand(options.is_forward() ? ExpansionType::forward : ExpansionType::reverse, request, *reader, mode_costing, mode, target_edges);

  rapidjson::Document document;
  document.SetObject();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  rapidjson::Value routes(rapidjson::Type::kArrayType);

  for(const auto& gbfs_result : res) {
    rapidjson::Value route(rapidjson::Type::kObjectType);
    
    rapidjson::Value station(rapidjson::Type::kObjectType);
    station.AddMember("id", gbfs_result.first, allocator);
    route.AddMember("p_type", "Station", allocator);
    route.AddMember("p", station.Move(), allocator);

    auto r = gbfs_result.second;
    route.AddMember("total_duration", (uint32_t)(r.time_total / 60), allocator); // uint32 total_duration

    rapidjson::Value gbfs_route(rapidjson::Type::kObjectType);
    if(r.start.type == static_cast<uint8_t>(gbfs_location_node_type::kFreeBike)) {
      gbfs_route.AddMember("bike_id", std::string(r.start.id.data()), allocator);

      rapidjson::Value pos(rapidjson::Type::kObjectType);
      pos.AddMember("lat", r.start_ll.lat(), allocator);
      pos.AddMember("lng", r.start_ll.lng(), allocator);
      gbfs_route.AddMember("b", pos.Move(), allocator);
      
      gbfs_route.AddMember("walk_duration", (uint16_t)(r.time_pedestrian / 60), allocator); // uint16 walk_duration
      gbfs_route.AddMember("bike_duration", (uint16_t)(r.time_bicycle / 60), allocator); // uint16 bike_duration
      
      route.AddMember("vehicle_type", "dockless bike", allocator);
      route.AddMember("route_type", "FreeBikeRoute", allocator);
    }
    else if(r.start.type == static_cast<uint8_t>(gbfs_location_node_type::kBicycleStation)) {
      rapidjson::Value s_start(rapidjson::Type::kObjectType);
      s_start.AddMember("id", std::string(r.start.id.data()), allocator);
      rapidjson::Value pos_start(rapidjson::Type::kObjectType);
      pos_start.AddMember("lat", r.start_ll.lat(), allocator);
      pos_start.AddMember("lng", r.start_ll.lng(), allocator);
      s_start.AddMember("pos", pos_start.Move(), allocator);
      gbfs_route.AddMember("from", s_start.Move(), allocator);

      rapidjson::Value s_end(rapidjson::Type::kObjectType);
      s_end.AddMember("id", std::string(r.end_station.id.data()), allocator);
      rapidjson::Value pos_end(rapidjson::Type::kObjectType);
      pos_end.AddMember("lat", r.end_station_ll.lat(), allocator);
      pos_end.AddMember("lng", r.end_station_ll.lng(), allocator);
      s_end.AddMember("pos", pos_end.Move(), allocator);
      gbfs_route.AddMember("to", s_end.Move(), allocator);

      gbfs_route.AddMember("first_walk_duration", (uint16_t)r.time_pedestrian, allocator); // uint16 first_walk_duration
      gbfs_route.AddMember("bike_duration", (uint16_t)r.time_bicycle, allocator); // uint16 bike_duration
      gbfs_route.AddMember("second_walk_duration", (uint16_t)r.time_pedestrian_end, allocator); // uint16 second_walk_duration

      route.AddMember("vehicle_type", "station bike", allocator);
      route.AddMember("route_type", "StationBikeRoute", allocator);
    }
    else {
      route.AddMember("vehicle_type", "foot", allocator);
      route.AddMember("route_type", "FootRoute", allocator);
    }
    route.AddMember("route", gbfs_route.Move(), allocator);

    routes.PushBack(route.Move(), allocator);
  }
  document.AddMember("routes", routes.Move(), allocator);

  rapidjson::StringBuffer buffer;
  buffer.Clear();
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::string response_string = buffer.GetString();

  // std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
  // LOG_INFO((boost::format("GBFS ----- Time to expand: %1%") % std::chrono::duration_cast<std::chrono::milliseconds>(end_total - begin_total).count()).str());
  gbfs_router.Clear();
  return response_string;
}

} // namespace thor
} // namespace valhalla