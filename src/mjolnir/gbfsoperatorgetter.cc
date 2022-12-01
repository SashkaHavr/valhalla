#include "valhalla/mjolnir/gbfsoperatorgetter.h"

namespace valhalla {
namespace mjolnir {
namespace gbfs {

std::vector<gbfs_operator*> gbfs_operator_getter::operators() {
  if(operators_.size() > 0) {
    return operators_;
  }
  std::vector<std::string> urls;
  for (const auto& kv : config.get_child("mjolnir.gbfs.operators_base_urls")) {
    urls.push_back(kv.second.get_value<std::string>());
  }
  for(const std::string& url : urls) {
    operators_.push_back(new gbfs_operator(url));
  }
  return operators_;
}

std::string gbfs_operator::fetch_json(std::string url) {
  long http_code = 0;
  const std::function<void()>* interrupt = nullptr;
  valhalla::baldr::curler_pool_t curlers(1, "");
  valhalla::baldr::scoped_curler_t curler(curlers);
  std::vector<char> response = curler.get()(url, http_code, false, interrupt);
  if(response.size() == 0) {
    LOG_ERROR("GBFS ----- Response is empty. URL: " + url);
    throw std::exception();
  }
  return std::string(response.begin(), response.end());
}

gbfs_urls& gbfs_operator::urls() {
  if(urls_.is_outdated()) {
    urls_ = gbfs_urls(fetch_json(url));
  }
  return urls_;
}

gbfs_system_information& gbfs_operator::system_information() {
  if(system_information_.is_outdated()) {
    system_information_ = gbfs_system_information(fetch_json(urls().system_information_url()));
  }
  return system_information_;
}

gbfs_station_information& gbfs_operator::station_information() {
  if(station_information_.is_outdated()) {
    station_information_ = gbfs_station_information(fetch_json(urls().station_information_url()));
  }
  return station_information_;
}

gbfs_free_bike_status& gbfs_operator::free_bike_status() {
  if(free_bike_status_.is_outdated()) {
    std::string url = urls().free_bike_status_url();
    if(url == "") {
      LOG_WARN("GBFS ----- free_bike_status file is not available for " + system_information().operator_name());
      return free_bike_status_;
    }
    free_bike_status_ = gbfs_free_bike_status(fetch_json(url));
  }
  return free_bike_status_;
}

std::string gbfs_urls::get_url(std::string key) {
  for(auto& lng : data().GetObject()) {
    auto urls = lng.value["feeds"].GetArray();
    auto res = std::find_if(urls.begin(), urls.end(), [&key](rapidjson::Value& val){return std::string(val["name"].GetString()) == key;});
    if(res == urls.end()) {
      continue;
    }
    return (*res)["url"].GetString();
  }
  return "";
}

const std::vector<station_information>& gbfs_station_information::stations() {
  if(stations_.size() > 0) {
    return stations_;
  }
  auto json_stations = data()["stations"].GetArray();
  for(const auto& json_station : json_stations) {
    stations_.push_back({json_station["station_id"].GetString(), json_station["name"].GetString(), {json_station["lon"].GetDouble(), json_station["lat"].GetDouble()}});
  }
  return stations_;
}

const std::vector<free_bike>& gbfs_free_bike_status::bikes() {
  if(bikes_.size() > 0) {
    return bikes_;
  }
  auto json_bikes = data()["bikes"].GetArray();
  for(const auto& json_bike : json_bikes) {
    bikes_.push_back({json_bike["bike_id"].GetString(), {json_bike["lon"].GetDouble(), json_bike["lat"].GetDouble()}});
    auto it = json_bike.FindMember("station_id");
    if(it != json_bike.MemberEnd()) {
      bikes_.back().station_id = it->value.GetString();
    }
  }
  return bikes_;
}

const std::vector<free_bike>& gbfs_free_bike_status::free_bikes() {
  if(free_bikes_.size() > 0) {
    return free_bikes_;
  }
  if(bikes_.size() == 0) {
    bikes();
  }
  std::copy_if(bikes_.begin(), bikes_.end(), std::back_inserter(free_bikes_), [](free_bike& bike) { return bike.station_id == kGBFSInvalidId;});
  return free_bikes_;
}

} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs