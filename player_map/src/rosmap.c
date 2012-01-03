#include "player_map/rosmap.hpp"

map_t * requestMap(char *name) {
  ros::NodeHandle nh;
  // Taken from PAMCL
  map_t* map = map_alloc();
  ROS_ASSERT(map);
    
  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call(srv_name, req, resp)) {
    ROS_WARN("Request for map '%s' failed; trying again...", srv_name);
    ros::Duration d(0.5);
    d.sleep();
    if (!nh.ok())
      break;
  }
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           resp.map.info.width,
           resp.map.info.height,
           resp.map.info.resolution);
    
  map->size_x = resp.map.info.width;
  map->size_y = resp.map.info.height;
  map->scale = resp.map.info.resolution;
  map->origin_x = resp.map.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = resp.map.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++) {
    if(resp.map.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(resp.map.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }
  return map;
}
