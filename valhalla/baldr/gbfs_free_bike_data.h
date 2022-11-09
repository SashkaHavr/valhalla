#include <cstdint>
#include <array>

// struct fb_edge_map
// {
//     uint32_t edge_id;
//     uint32_t offset_to_shape_map;
// };

// struct fb_shape_map
// {
//     uint32_t shape_idx;
//     uint32_t offset_to_free_bikes;
// };

struct gbfs_location_node {
  uint32_t node_id;
  uint8_t type;
  std::array<char, 64> id;
};
