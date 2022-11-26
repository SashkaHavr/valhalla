#include "valhalla/thor/worker.h"
#include <boost/format.hpp>
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"

namespace valhalla {
namespace thor {

std::string thor_worker_t::gbfs_route(Api& request) {
  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);
  // auto costing = options.costing_type();
  // auto costing_str = Costing_Enum_Name(costing);

  // sif::mode_costing_t costings;
  // costings[1] = factory.Create(options.costings().find(Costing::pedestrian)->second);
  // costings[2] = factory.Create(options.costings().find(Costing::bicycle)->second);

  // LOG_INFO((boost::format("GBFS ----- Costing original: %1%") % static_cast<int>(mode_costing[2]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Consting own 1: %1%") % static_cast<int>(costings[1]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own 2: %1%") % static_cast<int>(costings[2]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own create 1: %1%") % static_cast<int>(costings_[1]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Costings own create 2: %1%") % static_cast<int>(costings_[2]->travel_mode())).str());
  // LOG_INFO((boost::format("GBFS ----- Travel mode: %1%") % static_cast<int>(mode)).str());

  std::unordered_map<uint64_t, std::vector<valhalla::Location>> target_edges;
  for(auto it = options.locations().begin() + 1; it != options.locations().end(); ++it) {
    auto location = *it;
    for(const auto& edge : location.correlation().edges()) {
      target_edges[edge.graph_id()].push_back(location);
    }
    // LOG_INFO((boost::format("GBFS ----- Received location: %1%, Number of edges: %2%") % location.gbfs_transport_station_id() % location.correlation().edges().size()).str());
  }

  options.mutable_locations()->DeleteSubrange(1,  options.locations().size() - 1);
  // LOG_INFO((boost::format("GBFS ----- Received start locations: %1%") % options.locations().size()).str());
  // LOG_INFO((boost::format("GBFS ----- Found target edges: %1%") % target_edges.size()).str());

  // LOG_INFO((boost::format("GBFS ----- Got dir: %1%") % options.is_forward()).str());
  // LOG_INFO((boost::format("GBFS ----- Gor max time: %1%") % options.gbfs_max_duration()).str());

  std::chrono::steady_clock::time_point begin_total = std::chrono::steady_clock::now();
  auto res = gbfs_router.Expand(ExpansionType::forward, request, *reader, mode_costing, mode, target_edges);

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
    route.AddMember("total_duration", (uint32_t)r.time_total, allocator); // uint32 total_duration

    rapidjson::Value gbfs_route(rapidjson::Type::kObjectType);
    if(r.start.type == static_cast<uint8_t>(gbfs_location_node_type::kFreeBike)) {
      gbfs_route.AddMember("bike_id", std::string(r.start.id.data()), allocator);

      rapidjson::Value pos(rapidjson::Type::kObjectType);
      pos.AddMember("lat", r.start_ll.lat(), allocator);
      pos.AddMember("lng", r.start_ll.lng(), allocator);
      gbfs_route.AddMember("b", pos.Move(), allocator);
      
      gbfs_route.AddMember("walk_duration", (uint16_t)r.time_pedestrian, allocator); // uint16 walk_duration
      gbfs_route.AddMember("bike_duration", (uint16_t)r.time_bicycle, allocator); // uint16 bike_duration
      
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
    route.AddMember("route", gbfs_route.Move(), allocator);

    routes.PushBack(route.Move(), allocator);
  }
  document.AddMember("routes", routes.Move(), allocator);

  rapidjson::StringBuffer buffer;
  buffer.Clear();
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::string response_string = buffer.GetString();

  std::chrono::steady_clock::time_point end_total = std::chrono::steady_clock::now();
  LOG_INFO((boost::format("GBFS ----- Time to expand: %1%") % std::chrono::duration_cast<std::chrono::milliseconds>(end_total - begin_total).count()).str());
  gbfs_router.Clear();
  return response_string;
}

} // namespace thor
} // namespace valhalla