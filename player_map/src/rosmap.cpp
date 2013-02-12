#include "player_map/rosmap.hpp"

#include <cstdio>
#include <cmath>

#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <player_map/GetMap.h>

using namespace std;
namespace rf {

  map_t * requestCSpaceMap(const char *srv_name) {
    ros::NodeHandle nh;
    // Taken from PAMCL
    map_t* map = map_alloc();
    ROS_ASSERT(map);
    
    // get map via RPC
    player_map::GetMap::Request  req;
    player_map::GetMap::Response resp;
    ROS_INFO("Requesting c-space + map...");
    while(!ros::service::call(srv_name, req, resp)) {
      ROS_WARN("Request for map '%s' failed; trying again...",
               ros::names::resolve(string(srv_name)).c_str());
      ros::Duration d(4.0);
      d.sleep();
      if (!nh.ok())
        break;
    }
    ROS_INFO("Received a %d X %d map @ %.3f m/pix  max_occ_dist= %.3f\n",
             resp.map.info.width,
             resp.map.info.height,
             resp.map.info.resolution,
             resp.max_occ_dist);
    convertMap(resp.map, map);

    map->max_occ_dist = resp.max_occ_dist;
    for (int j = 0; j < map->size_y; j++) {
        for (int i = 0; i < map->size_x; i++) {
          int ind = MAP_INDEX(map, i, j);
          (map->cells + ind)->occ_dist = resp.occ_dist[ind];
        }
    }
    return map;
  }
  
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
      ROS_WARN("Request for map '%s' failed; trying again...",
               ros::names::resolve(string(srv_name)).c_str());
      ros::Duration d(4.0);
      d.sleep();
      if (!nh.ok())
        break;
    }
    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
             resp.map.info.width,
             resp.map.info.height,
             resp.map.info.resolution);
    
    convertMap(resp.map, map);
    return map;
  }

  void convertMap(const nav_msgs::OccupancyGrid &map, map_t *pmap) {
    pmap->size_x = map.info.width;
    pmap->size_y = map.info.height;
    pmap->scale = map.info.resolution;
    pmap->origin_x = map.info.origin.position.x + (pmap->size_x / 2) * pmap->scale;
    pmap->origin_y = map.info.origin.position.y + (pmap->size_y / 2) * pmap->scale;
    // Convert to player format
    pmap->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*pmap->size_x*pmap->size_y);
    ROS_ASSERT(pmap->cells);
    for(int i=0;i<pmap->size_x * pmap->size_y;i++) {
      if(map.data[i] == 0)
        pmap->cells[i].occ_state = -1;
      else if(map.data[i] == 100)
        pmap->cells[i].occ_state = +1;
      else
        pmap->cells[i].occ_state = 0;

      pmap->cells[i].occ_dist = 0;
    }
  }
  

  LOSChecker::LOSChecker(map_t *map) : map_(map) {

  }

  bool LOSChecker::LineOfSight(double x1, double y1, double x2, double y2, double max_dist /* = 0.0 */) {
    if (map_ == NULL) {
      return true;
    }
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
        // ROS_WARN_THROTTLE(5, "LOSChecker::LineOfSight() Beyond map edge");
        return false;
      } else if (cell->occ_state != -1 || cell->occ_dist < max_dist) {
        return false;
      }
      line_x += step_size * ct;
      line_y += step_size * st;    
    }
    return true;
  }


  struct Node {
    Node(const pair<int, int> &c, float d, float h) :
      coord(c), true_dist(d), heuristic(h) { }
    pair<int, int> coord;
    float true_dist;
    float heuristic;
  };

  struct NodeCompare {
    bool operator()(const Node &lnode, const Node &rnode) {
      return make_pair(lnode.heuristic, lnode.coord) <
        make_pair(rnode.heuristic, rnode.coord);
    }
  };
    
  PointVector astar(const Eigen::Vector2f &start, const Eigen::Vector2f &stop,
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
    // True cost to goal from cell
    float *costs = new float[ncells];
    // (i,j) coordinates of node with lowest cost to current node
    int *prev_i = new int[ncells];
    int *prev_j = new int[ncells];
    
    // Map is large and initializing costs takes a while.  To speedup,
    // partially initialize costs in a rectangle surrounding start and stop
    // positions + margin.  If you run up against boundary, initialize the rest.
    int margin = 120;
    pair<int, int> init_ul = make_pair(max(0, min(starti, stopi) - margin),
                                       max(0, min(startj, stopj) - margin));
    pair<int, int> init_lr =
      make_pair(min(map->size_x, max(starti, stopi) + margin),
                min(map->size_y, max(startj, stopj) + margin));
    for (int j = init_ul.second; j < init_lr.second; ++j) {
      for (int i = init_ul.first; i < init_lr.first; ++i) {
        int ind = MAP_INDEX(map, i, j);
        costs[ind] = std::numeric_limits<float>::infinity();
      }
    }

    // fprintf(stderr, "Start: %i %i\n", starti, startj);
    // fprintf(stderr, "Stop:  %i %i\n", stopi, stopj);  
  
    int start_ind = MAP_INDEX(map, starti, startj);
    costs[start_ind] = 0.0;
    prev_i[starti] = starti;
    prev_j[startj] = startj;
  
    // Priority queue mapping cost to index
    set<Node, NodeCompare> Q;
    Q.insert(Node(make_pair(starti, startj), 0.0, 0.0));
    bool found = false;
    bool full_init = false;
    while (!Q.empty()) {
      // Copy node and then erase it
      Node curr_node = *Q.begin();
      Q.erase(Q.begin());
      // float cost = curr_node.true_dist;
      int ci = curr_node.coord.first;
      int cj = curr_node.coord.second;
     
      costs[MAP_INDEX(map, ci, cj)] = curr_node.true_dist;
      // fprintf(stderr, "At %i %i (cost = %6.2f)  % 7.2f % 7.2f \n",
      //     ci, cj, curr_node.true_dist, MAP_WXGX(map, ci), MAP_WYGY(map, cj));
      if (ci == stopi && cj == stopj) {
        found = true;
        break;
      }

      // Check if we're neighboring nodes whose costs are uninitialized.
      if (!full_init &&
          ((ci + 1 >= init_lr.first) || (ci - 1 <= init_ul.first) ||
           (cj + 1 >= init_lr.second) || (cj - 1 <= init_ul.second))) {
        full_init = true;
        for (int j = 0; j < map->size_y; ++j) {
          for (int i = 0; i < map->size_x; ++i) {
            // Only initialize costs that are outside original rectangle
            if (!(init_ul.first <= i && i < init_lr.first &&
                  init_ul.second <= j && j < init_lr.second)) {
              int ind = MAP_INDEX(map, i, j);
              costs[ind] = std::numeric_limits<float>::infinity();
            }
          }
        }
      }

      // Iterate over neighbors
      for (int newj = cj - 1; newj <= cj + 1; ++newj) {
        for (int newi = ci - 1; newi <= ci + 1; ++newi) {
          // Skip self edges
          if ((newi == ci && newj == cj) || !MAP_VALID(map, newi, newj)) {
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
          double heur_cost = hypot(newi - stopi, newj - stopj);
          double ttl_cost = edge_cost + curr_node.true_dist + heur_cost;
          if (ttl_cost < costs[index]) {
            // fprintf(stderr, "    Better path: new cost= % 6.2f\n", ttl_cost);
            // If node has finite cost, it's in queue and needs to be removed
            if (!isinf(costs[index])) {
              Q.erase(Node(make_pair(newi, newj), costs[index], 0.0));
            }            
            costs[index] = ttl_cost;
            prev_i[index] = ci;
            prev_j[index] = cj;
            Q.insert(Node(make_pair(newi, newj),
                          edge_cost + curr_node.true_dist, ttl_cost));
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
