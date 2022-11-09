#include <cstdint>
#include <array>

struct gbfs_location_node {
  uint32_t node_id;
  uint8_t type;
  std::array<char, 64> id;
};
