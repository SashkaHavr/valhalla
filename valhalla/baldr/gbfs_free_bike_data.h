#include <cstdint>
#include <array>

enum class gbfs_location_node_type : uint8_t {
  kNone = 0,
  kBicycleStation = 1,
  kFreeBike = 2
};

struct gbfs_location_node {
  uint32_t node_id;
  uint8_t type;
  std::array<char, 64> id;
};
