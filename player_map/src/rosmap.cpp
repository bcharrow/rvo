#include "player_map/rosmap.hpp"

#include <cstdio>
#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/GetMap.h>

using namespace std;
namespace rf {
  map_t * requestMap(const char *srv_name) {
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

  LOSChecker::LOSChecker(map_t *map) : map_(map) {

  }

  bool LOSChecker::LineOfSight(double x1, double y1, double x2, double y2, double max_dist /* = 0.0 */) {
    // March along the line between (x1, y1) and (x2, y2) until the point passes
    // beyond (x2, y2).
    double step_size = map_->scale / 2.0;

    double dy = y2 - y1;
    double dx = x2 - x1;
    double t = atan2(dy, dx);
    double ct = cos(t);
    double st = sin(t);

    double mag_sq = dy * dy + dx * dx;
    // fprintf(stderr, "t: %f ct: %f st: %f\n", t, ct, st);
    double line_x = x1, line_y = y1;
    // Terminate when current point defines a vector longer than original vector
    while (pow(line_x - x1, 2) + pow(line_y - y1, 2) < mag_sq) {
      // fprintf(stderr, "%f %f\n", line_x, line_y);
      map_cell_t *cell = map_get_cell(map_, line_x, line_y, 0);
      if (cell == NULL) {
        ROS_WARN_THROTTLE(5, "LOSChecker::LineOfSight() Beyond map edge");
        return false;
      } else if (cell->occ_state != -1 || cell->occ_dist < max_dist) {
        return false;
      }
      line_x += step_size * ct;
      line_y += step_size * st;    
    }
    return true;
  }

  PointVector dijkstra(const Eigen::Vector2f &start, const Eigen::Vector2f &stop,
                       map_t *map, double max_occ_dist /* = 0.0 */) {
    int starti = MAP_GXWX(map, start(0)), startj = MAP_GYWY(map, start(1));
    if (!MAP_VALID(map, starti ,startj)) {
      ROS_ERROR("Invalid starting position");
      return PointVector();
    }
    int stopi = MAP_GXWX(map, stop(0)), stopj = MAP_GYWY(map, stop(1));
    if (!MAP_VALID(map, stopi ,stopj)) {
      ROS_ERROR("Invalid stopping position");
      return PointVector();
    }

    const int ncells = map->size_x * map->size_y;
    float *costs = new float[ncells];
    int *prev_i = new int[ncells];
    int *prev_j = new int[ncells];
    for (int k = 0; k < ncells; ++k) {
      costs[k] = std::numeric_limits<float>::infinity();
      prev_i[k] = -1;
      prev_j[k] = -1;
    }
    // fprintf(stderr, "Start: %i %i\n", starti, startj);
    // fprintf(stderr, "Stop:  %i %i\n", stopi, stopj);  
  
    int start_ind = MAP_INDEX(map, starti, startj);
    costs[start_ind] = 0.0;
    prev_i[starti] = starti;
    prev_j[startj] = startj;
  
    // Priority queue mapping cost to index
    typedef pair<int, int> coord_t;
    typedef pair<float, coord_t> cost_t;  
    set<cost_t> Q;
    Q.insert(make_pair(0.0, make_pair(starti, startj)));

    bool found = false;
    while (!Q.empty()) {
      cost_t node = *Q.begin();
      Q.erase(Q.begin());
      double cost = node.first;
      coord_t coord = node.second;
      int ci = coord.first;
      int cj = coord.second;
      // fprintf(stderr, "At %i %i (cost = %6.2f)  % 7.2f % 7.2f \n",
      //         ci, cj, cost, MAP_WXGX(map, ci), MAP_WYGY(map, cj));
    
      if (ci == stopi && cj == stopj) {
        found = true;
        break;
      }
    
      // Iterate over neighbors
      for (int newi = ci - 1; newi <= ci + 1; ++newi) {
        for (int newj = cj - 1; newj <= cj + 1; ++newj) {
          // Skip self edges
          if (newi == ci && newj == cj || !MAP_VALID(map, newi, newj)) {
            continue;
          }
          // fprintf(stderr, "  Examining %i %i ", newi, newj);
          int index = MAP_INDEX(map, newi, newj);
          map_cell_t *cell = map->cells + index;
          // If cell is occupied or too close to occupied cell, continue

          if (cell->occ_state != -1 || cell->occ_dist < max_occ_dist) {
            // fprintf(stderr, "occupado\n");
            continue;
          }
          // fprintf(stderr, "free\n");        
          double edge_cost = ci == newi || cj == newj ? 1 : sqrt(2);        
          if (edge_cost + cost < costs[index]) {
            // fprintf(stderr, "    Better path: new cost= % 6.2f\n", edge_cost + cost);
            costs[index] = edge_cost + cost;
            prev_i[index] = ci;
            prev_j[index] = cj;
            Q.insert(make_pair(costs[index], make_pair(newi, newj)));
          }
        }
      }
    }

    // Recreate path
    PointVector path;
    if (found) {
      int i = stopi, j = stopj;
      while (!(i == starti && j == startj)) {
        int index = MAP_INDEX(map, i, j);      
        float x = MAP_WXGX(map, i);
        float y = MAP_WYGY(map, j);
        path.push_back(Eigen::Vector2f(x, y));

        i = prev_i[index];
        j = prev_j[index];
      }
      float x = MAP_WXGX(map, i);
      float y = MAP_WYGY(map, j);
      path.push_back(Eigen::Vector2f(x, y));
    }

    delete[] costs;
    delete[] prev_i;
    delete[] prev_j;
  
    return PointVector(path.rbegin(), path.rend());
  }
}
