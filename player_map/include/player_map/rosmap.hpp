#ifndef ROSMAP_HPP
#define ROSMAP_HPP

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/OccupancyGrid.h>
#include <player_map/map.h>

namespace rf {
  map_t * requestCSpaceMap(const char *name);
  map_t * requestMap(const char *name);
  void convertMap(const nav_msgs::OccupancyGrid &map, map_t *pmap);
  class LOSChecker {
  public:
    LOSChecker(map_t *map);
    bool LineOfSight(double x1, double y1, double x2, double y2, double max_dist = 0.0);
  private:
    map_t *map_;
  };

  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > PointVector;
  PointVector astar(const Eigen::Vector2f &start, const Eigen::Vector2f &stop,
                    map_t *map, double max_occ_dist = 0.0);
}

#endif
