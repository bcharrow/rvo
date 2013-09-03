#ifndef ROSMAP_HPP
#define ROSMAP_HPP

#include <vector>
#include <set>

#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/OccupancyGrid.h>
#include <player_map/map.h>

namespace rf {
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > PointVector;

class OccupancyMap {
public:
  OccupancyMap(map_t *map);
  ~OccupancyMap();

  static OccupancyMap* FromMapServer(const char *srv_name);

  void setMap(const nav_msgs::OccupancyGrid &grid);
  void updateCSpace(double max_occ_dist);

  bool lineOfSight(double x1, double y1, double x2, double y2,
                   double max_occ_dist = 0.0) const;
  PointVector astar(double x1, double y1, double x2, double y2,
                    double max_occ_dist = 0.0);

  // PointVector prepareShortestPaths(double x, double y, double distance,
  //                                  double max_occ_dist);
  // PointVector buildShortestPath(int ind);

private:
  struct Node {
    Node() {}
    Node(const std::pair<int, int> &c, float d, float h) :
      coord(c), true_dist(d), heuristic(h) { }
    std::pair<int, int> coord;
    float true_dist;
    float heuristic;
  };

  struct NodeCompare {
    bool operator()(const Node &lnode, const Node &rnode) {
      return make_pair(lnode.heuristic, lnode.coord) <
        make_pair(rnode.heuristic, rnode.coord);
    }
  };

  void initializeSearch(double startx, double starty);
  bool nextNode(Node *curr_node, double max_occ_dist);
  void addNeighbors(const Node &node, double max_occ_dist);
  void buildPath(int i, int j, PointVector *path);

  map_t *map_;
  int ncells_;
  int starti_, startj_;
  int stopi_, stopj_;
  boost::scoped_array<float> costs_;
  boost::scoped_array<int> prev_i_;
  boost::scoped_array<int> prev_j_;
  // Priority queue mapping cost to index
  boost::scoped_ptr<std::set<Node, NodeCompare> > Q_;
};

}
#endif
