#include <cstdint>

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

struct fb_node {
  uint32_t node_id;
  uint32_t offset_to_free_bike_ids;
};
