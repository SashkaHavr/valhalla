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
  boost::property_tree::ptree gbfs_json;
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

gbfs_system_information& gbfs_operator::system_information() {
  if(system_information_.is_outdated()) {
    system_information_ = gbfs_system_information(fetch_json(urls().system_information_url()));
  }
  return system_information_;
}

gbfs_urls& gbfs_operator::urls() {
  if(urls_.is_outdated()) {
    urls_ = gbfs_urls(fetch_json(url));
  }
  return urls_;
}


} //namespace valhalla
} //namespace mjolnir
} //namespace gbfs