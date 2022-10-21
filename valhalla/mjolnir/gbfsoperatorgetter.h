#include <vector>
#include <string>
#include <chrono>

#include <boost/property_tree/ptree.hpp>
#include <boost/format.hpp>
#include "rapidjson/document.h"

#include "valhalla/midgard/logging.h"
#include "valhalla/baldr/curler.h"
#include "valhalla/midgard/pointll.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace std::literals;

namespace valhalla {
namespace mjolnir {
namespace gbfs {

// const rapidjson::Document empty_document;
// void gbfs_graph_builder::fetch_gbfs_data();



struct gbfs_base {
  gbfs_base() {

  }

  gbfs_base(std::string json) {
    time_created = std::chrono::steady_clock::now();
    document.Parse(json.c_str());
    ttl = document["ttl"].GetUint();
  }

  bool is_outdated() {
    if(ttl == 0) {
      return true;
    }
    auto time_now = std::chrono::steady_clock::now();
    // LOG_INFO((boost::format("GBFS ----- Time since creation: %1%, TTL: %2%") % ((time_now - time_created) / 1s) % ttl).str());
    return ((time_now - time_created) / 1s) > ttl;
  }

  rapidjson::Value& data() {
    return document["data"];
  }

protected:
  rapidjson::Document document;
private:
  std::chrono::time_point<std::chrono::steady_clock> time_created;
  unsigned int ttl = 0;
};


struct gbfs_urls : gbfs_base {
  using gbfs_base::gbfs_base;
  
  std::string system_information_url() {
    return get_url("system_information");
  }

  std::string station_information_url() {
    return get_url("station_information");
  }

  std::string station_status_url() {
    return get_url("station_status");
  }

  std::string free_bike_status_url() {
    return get_url("free_bike_status");
  }

private:
  std::string get_url(std::string key);
};

struct gbfs_system_information : gbfs_base {
  using gbfs_base::gbfs_base;

  std::string operator_name() {
    return data()["name"].GetString();
  }
};

struct station_information {
  std::string id;
  std::string name;
  PointLL location;
};

struct gbfs_station_information : gbfs_base {
  using gbfs_base::gbfs_base;

  const std::vector<station_information>&  stations();
private:
  std::vector<station_information> stations_;
};

struct gbfs_operator {

  gbfs_operator(std::string url)
        : url(url) {

  }

  // Getters
  gbfs_urls& urls();
  gbfs_system_information& system_information();
  gbfs_station_information& station_information();

private:
  // Data fields
  gbfs_system_information system_information_;
  gbfs_urls urls_;
  gbfs_station_information station_information_;

  // Network
  std::string url;
  std::string fetch_json(std::string url);
};




struct gbfs_operator_getter {

  gbfs_operator_getter(boost::property_tree::ptree& config) : config(config) {

  }
  ~gbfs_operator_getter() {
    for(gbfs_operator* o : operators_) {
      delete o;
    }
  }

  std::vector<gbfs_operator*> operators();
private:
  std::vector<gbfs_operator*> operators_;
  boost::property_tree::ptree& config;
};

} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs