#include "ros/ros.h"

#include "player_map/rosmap.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "player_map_test");

  ros::NodeHandle nh;

  map_t *full_map = rf::requestCSpaceMap("player_static_map");

  map_t *map = rf::requestMap("static_map");
  ROS_INFO("Building c-space for original map");
  map_update_cspace(map, full_map->max_occ_dist);
  
  if (map->size_y != full_map->size_y || map->size_x != full_map->size_x) {
    ROS_ERROR("map sizes differ");
    return -1;
  } else {
    ROS_INFO("map sizes are the same");
  }
  
  for (int j = 0; j < map->size_y; j++) {
    for (int i = 0; i < map->size_x; i++) {
      int ind = MAP_INDEX(map, i, j);
      bool same_state = map->cells[ind].occ_state == full_map->cells[ind].occ_state;
      bool same_dist = (map->cells[ind].occ_dist - full_map->cells[ind].occ_dist) < 1e-6;
      if (!same_state) {
        ROS_ERROR("States differ");
        goto err;
      } else if (!same_dist) {
        ROS_ERROR("Dists differ");
        ROS_ERROR("%f %f", map->cells[ind].occ_dist, full_map->cells[ind].occ_dist);        
        goto err;
      }
    }
  }
  ROS_INFO("maps are identical!");  
 err:
  
  map_free(full_map);
  map_free(map);
}
