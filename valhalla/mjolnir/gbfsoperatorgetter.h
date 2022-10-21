#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include "rapidjson/document.h"
#include "valhalla/midgard/logging.h"
#include "valhalla/baldr/curler.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {
namespace gbfs {

// const rapidjson::Document empty_document;
// void gbfs_graph_builder::fetch_gbfs_data();



struct gbfs_base {
  time_t time_stamp;

  gbfs_base() {

  }

  gbfs_base(std::string json) {
    time_stamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    data.Parse(json.c_str());
  }

  bool is_outdated() {
    return true;
  }

protected:
  rapidjson::Document data;
};


struct gbfs_urls : gbfs_base {
  using gbfs_base::gbfs_base;
  
  std::string system_information_url() {
    auto urls = data["data"]["en"]["feeds"].GetArray();
    auto res = std::find_if(urls.begin(), urls.end(), [](rapidjson::Value& val){return std::string(val["name"].GetString()) == "system_information";});
    if(res == urls.end()) {
      throw new std::exception();
    }
    return (*res)["url"].GetString();
  }
};

struct gbfs_system_information : gbfs_base {
  using gbfs_base::gbfs_base;

  std::string operator_name() {
    return data["data"]["name"].GetString();
  }
};



struct gbfs_operator {

  gbfs_operator(std::string url)
        : url(url) {

  }

  // Getters
  gbfs_urls& urls();
  gbfs_system_information& system_information();

private:
  // Data fields
  gbfs_system_information system_information_;
  gbfs_urls urls_;

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