#include "rvo_wrapper.hpp"

#include <tf/tf.h>

using namespace rf;
using namespace std;
//================================ Utilities ================================//

RVO::Vector2 pose_to_rvo(const geometry_msgs::Pose& p) {
  return RVO::Vector2(p.position.x, p.position.y);
}

double quat_angle(const geometry_msgs::Quaternion& quat) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(quat, q);
  return copysign(q.getAngle(), q.getAxis().z());
}

RVO::Vector2 odom_to_rvo(const nav_msgs::Odometry& odom,
                         const geometry_msgs::Pose& p) {
  double vx = odom.twist.twist.linear.x;
  double vw = odom.twist.twist.linear.z;

  double theta = quat_angle(p.orientation);
  // Rotate to get velocity in world frame
  double xdot = vx * cos(theta) - vw * sin(theta);
  double ydot = vx * sin(theta) + vw * cos(theta);
  return RVO::Vector2(xdot, ydot);
}

geometry_msgs::Twist vel_to_twist(const RVO::Vector2 vel) {
  geometry_msgs::Twist t;
  t.linear.x = vel.x();
  t.angular.z = vel.y();
  return t;
}

RVO::Vector2 eig_to_rvo(const Eigen::Vector2f& vec) {
  return RVO::Vector2(vec(0), vec(1));
}

Eigen::Vector2f rvo_to_eig(const RVO::Vector2& vec) {
  return Eigen::Vector2f(vec.x(), vec.y());
}

//================================= Wrapper =================================//
namespace rf {
  RVOWrapper::RVOWrapper(vector<BotClient *> bots, size_t id, map_t *map) :
    bots_(bots), map_(map), id_(id), axel_width_(0.23), goal_tol_(0.1),
    path_margin_(0.2), timestep_(0.1), los_margin_(0.1) {
    if (id >= bots_.size()) {
      stringstream ss;
      ss << "RVOWrapper::RVOWrapper(): Bot id=" << id << " >= number of bots=" << bots_.size();
      throw std::runtime_error(ss.str());
    }
    
    RVO::Vector2 velocity(0.0, 0.0);
    float neighborDist = 3.0, timeHorizon = 2.0, timeHorizonObst = 2.0;
    float radius = 0.25, maxSpeed = 0.2;
    size_t maxNeighbors = 10;

    sim_ = new RVO::RVOSimulator();
    sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                           timeHorizonObst, radius, maxSpeed, velocity);
    sim_->setTimeStep(timestep_);

    std::vector<RVO::Vector2> obs1, obs2, obs3, obs4, obs5;
    // Lower square in Levine
    obs1.push_back(RVO::Vector2(-6.6750, 13.8700));
    obs1.push_back(RVO::Vector2(-6.6750, 5.4700));
    obs1.push_back(RVO::Vector2(0.6750, 5.4700));
    obs1.push_back(RVO::Vector2(0.5250, 13.8700));
    // Upper square in levine
    obs2.push_back(RVO::Vector2(0.5250, 18.5200));
    obs2.push_back(RVO::Vector2(0.5250, 27.2200));
    obs2.push_back(RVO::Vector2(-6.5250, 27.2200));
    obs2.push_back(RVO::Vector2(-6.5250, 18.6700));
    // Right boundary
    obs3.push_back(RVO::Vector2(1.90, 27.4));
    obs3.push_back(RVO::Vector2(1.90, 5.4700));
    obs3.push_back(RVO::Vector2(16.2250, 5.4700));
    obs3.push_back(RVO::Vector2(15.9250, 7.7200));
    obs3.push_back(RVO::Vector2(5.4250, 28.7200));
    obs3.push_back(RVO::Vector2(5.4250, 27.40));
    // Top boundary + left boundary
    obs4.push_back(RVO::Vector2(5.7750, 30.8200));
    obs4.push_back(RVO::Vector2(5.7750, 28.7200));
    obs4.push_back(RVO::Vector2(-8.3250, 28.8700));
    obs4.push_back(RVO::Vector2(-8.3250, 5.6200));
    obs4.push_back(RVO::Vector2(-10.8750, 5.6200));
    obs4.push_back(RVO::Vector2(-10.1250, 30.8200));
    // Bottom boundary
    obs5.push_back(RVO::Vector2(-11.0250, 6.0700));
    obs5.push_back(RVO::Vector2(-12.0750, 5.9200));
    obs5.push_back(RVO::Vector2(-11.9250, 2.6200));
    obs5.push_back(RVO::Vector2(18.0750, 2.1700));
    obs5.push_back(RVO::Vector2(18.0750, 3.3700));
    obs5.push_back(RVO::Vector2(7.8750, 3.0700));
    obs5.push_back(RVO::Vector2(7.8750, 4.0));
    obs5.push_back(RVO::Vector2(-11.0250, 4.0));

    sim_->addObstacle(obs1);
    sim_->addObstacle(obs2);
    sim_->addObstacle(obs3);
    sim_->addObstacle(obs4);
    sim_->addObstacle(obs5);
    sim_->processObstacles();

    for (size_t i = 0; i < bots_.size(); ++i) {
      sim_->addAgent(RVO::Vector2());
      goals_.push_back(RVO::Vector2());
    }

    // Shorten path to contains points that have visibility
    // TODO: change this to RVO's visibility test
    if (map_ != NULL) {
      checker_ = new LOSChecker(map_);
    } else {
      ROS_WARN("No map specified");
    }
  }

  RVOWrapper::~RVOWrapper() {
    delete sim_;
  }

  RVOWrapper* RVOWrapper::ROSInit(const ros::NodeHandle& nh, map_t *map, vector<BotClient*> bots) {
    // Setup simulation
    int id;
    double neighborDist, timeHorizon, timeHorizonObst, radius, maxSpeed, los_margin;
    double axel_width, waypoint_spacing, path_margin, timestep, goal_tol;
    int maxNeighbors;
    nh.param("neighborDist", neighborDist, 3.0);
    nh.param("maxNeighbors", maxNeighbors, 10);
    nh.param("timeHorizon", timeHorizon, 2.0);
    nh.param("timeHorizonObst", timeHorizonObst, 2.0);
    nh.param("radius", radius, 0.05);
    nh.param("maxSpeed", maxSpeed, 3.0);
    nh.param("id", id, 0);
    nh.param("timestep", timestep, 0.1);
    nh.param("axel_width", axel_width, 0.23);
    nh.param("waypoint_spacing", waypoint_spacing, 0.75);
    nh.param("path_margin", path_margin, 0.1);
    nh.param("goal_tolerance", goal_tol, 0.1);
    nh.param("los_margin", los_margin, 0.1);
    
    RVOWrapper *wrapper = new RVOWrapper(bots, id, map);
    wrapper->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                              timeHorizonObst, radius, maxSpeed);
    wrapper->setTimestep(timestep);
    wrapper->setAxelWidth(axel_width);
    wrapper->setWaypointSpacing(waypoint_spacing);
    wrapper->setPathMargin(path_margin);
    wrapper->setGoalTolerance(goal_tol);
    wrapper->setLOSMargin(los_margin);
    return wrapper;
  }
  
  void RVOWrapper::setAgentDefaults(float neighborDist, size_t maxNeighbors,
                                    float timeHorizon, float timeHorizonObst,
                                    float radius, float maxSpeed) {
    sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                           timeHorizonObst, radius, maxSpeed);
  }


  bool RVOWrapper::step() {
    // TODO: synchornize ROS clock and sim clock.
    sim_->doStep();
    RVO::Vector2 vel = sim_->getAgentVelocity(id_);

    // Solve for control inputs for standard kinematic model with feedback
    // linearization; see (eqn 2) in "Smooth and Collision-Free Navigation for
    // Multiple Robots Under Differential-Drive Constraints"
    double theta = quat_angle(bots_[id_]->getPose().orientation);

    double ct = cos(theta);
    double st = sin(theta);
    double L = sim_->getAgentRadius(id_) / 2.0;
    Eigen::Matrix2d m;
    m(0, 0) = ct / 2.0 + axel_width_ * st / L;
    m(0, 1) = ct / 2.0 - axel_width_ * st / L;
    m(1, 0) = st / 2.0 - axel_width_ * ct / L;
    m(1, 1) = st / 2.0 + axel_width_ * ct / L;

    Eigen::Vector2d v(vel.x(), vel.y());
    Eigen::Vector2d u = m.inverse() * v;

    bool at_dest = false;
    double vx;
    double vw;
    if (RVO::absSq(sim_->getAgentPosition(id_) - goals_[id_]) > goal_tol_ * goal_tol_) {
      vx = (u(0) + u(1)) / 2.0;
      vw = (u(1) - u(0)) / L;
    } else {
      vx = 0.0;
      vw = 0.0;
      at_dest = true;
    }
    bots_[id_]->pubVel(vx, vw);
    return at_dest;
    // ROS_INFO("Theta: % 6.2f", theta);
    // ROS_INFO("L: % 6.2f", L);
    // ROS_INFO_STREAM("m: " << m);
    // ROS_INFO_STREAM("m_inv: " << m.inverse());
    // ROS_INFO_STREAM("vel:       " << setprecision(2) << v.transpose());
    // ROS_INFO_STREAM("wheel vel: " << setprecision(2) << u.transpose());
    // ROS_INFO("sim pose:  % 6.2f % 6.2f", sim_->getAgentPosition(id_).x(),
    //          sim_->getAgentPosition(id_).y());
    // ROS_INFO("command:   % 6.2f % 6.2f\n", vx, vw);
  }

  bool RVOWrapper::setGoal(const geometry_msgs::Pose& p) {
    goals_[id_] = pose_to_rvo(p);
    waypoints_.clear();
    RVO::Vector2 start = sim_->getAgentPosition(id_);
    RVO::Vector2 stop = goals_[id_];
    // ROS_INFO("Current pose: % 6.2f % 6.2f", start.x(), start.y());
    // ROS_INFO("Got goal: % 6.2f % 6.2f", goals_[id_].x(), goals_[id_].y());
    
    PointVector path = dijkstra(rvo_to_eig(start), rvo_to_eig(stop), map_, path_margin_);
    if (path.size() == 0) {
      return false;
    }
  
    waypoints_.push_back(eig_to_rvo(path[0]));
    for (size_t i = 0; i < path.size() - 1; ++i) {
      const RVO::Vector2& prev = waypoints_.back();
      const Eigen::Vector2f& curr = path[i];
      if (!checker_->LineOfSight(prev.x(), prev.y(), curr(0), curr(1), los_margin_) ||
          hypot(prev.x() - curr(0), prev.y() - curr(1)) >= way_spacing_) {
        waypoints_.push_back(eig_to_rvo(path[i-1]));
      }
    }
    waypoints_.push_back(eig_to_rvo(path.back()));
    // ROS_INFO("Waypoint path has %zu vertices", waypoints_.size());
    // for (size_t i = 0; i < waypoints_.size(); ++i ) {
    //   ROS_INFO("  %3zu: % 6.1f % 6.1f    dist: % 6.1f",
    //            i, waypoints_[i].x(), waypoints_[i].y(),
    //            map_get_cell(map_, waypoints_[i].x(), waypoints_[i].y(), 0.0)->occ_dist);
    // }
    return true;
  }

  bool RVOWrapper::getLeadGoal(RVO::Vector2 *goal) {
    // Direct robot towards successor of point that it is closest to
    float min_dist = std::numeric_limits<float>::infinity();
    int min_ind = -1;
    RVO::Vector2 pos = sim_->getAgentPosition(id_);
    for (size_t wayind = 0; wayind < waypoints_.size(); ++wayind) {
      if (!checker_->LineOfSight(pos.x(), pos.y(),
                                 waypoints_[wayind].x(), waypoints_[wayind].y(),
                                 los_margin_)) {
        // ROS_INFO(" Skipping %zu, no LOS", wayind);
        continue;
      }
      float dist = RVO::absSq(pos - waypoints_[wayind]);
      if (dist < min_dist) {
        min_dist = dist;
        min_ind = wayind;
        // ROS_INFO(" %i is now closest % 7.1f", min_ind, min_dist);
      }
    }

    if (min_ind == -1) {
      *goal = RVO::Vector2();
      return false;
    } else {
      if (static_cast<unsigned>(min_ind + 1) < waypoints_.size() &&
          checker_->LineOfSight(pos.x(), pos.y(),
                                waypoints_[min_ind+1].x(), waypoints_[min_ind+1].y(),
                                los_margin_)) {
        ++min_ind;
      }
      *goal = waypoints_[min_ind] - pos;
      // ROS_INFO_THROTTLE(1.0, "Advancing towards goal");
      // ROS_INFO_THROTTLE(1.0, "occ dist: % 6.2f",
      //                   map_get_cell(map_, pos.x(), pos.y(), 0.0)->occ_dist);
      // ROS_INFO_THROTTLE(1.0, "min_ind: %i", min_ind);
      // ROS_INFO_THROTTLE(1.0, "position:   % 6.2f % 6.2f", pos.x(), pos.y());
      // ROS_INFO_THROTTLE(1.0, "waypoint:   % 6.2f % 6.2f",
      //                   waypoints_[min_ind].x(), waypoints_[min_ind].y());
      // ROS_INFO_THROTTLE(1.0, "goalVec:    % 6.2f % 6.2f",
      //                   goalVector.x(), goalVector.y());
      return true;      
    }
  }

  bool RVOWrapper::update() {
    /* Update position, speed, and goals of the agents */
    bool have_everything = true;
    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      BotClient *bot = bots_[i];
      if (bot->havePose()) {
        RVO::Vector2 position = pose_to_rvo(bot->getPose());
        // ROS_INFO_THROTTLE(1.0, "%zu position: % 6.2f % 6.2f", i, position.x(), position.y());
        sim_->setAgentPosition(i, position);

        if (bot->haveOdom()) {
          RVO::Vector2 vel = odom_to_rvo(bot->getOdom(), bot->getPose());
          sim_->setAgentVelocity(i, vel);
        } else {
          ROS_WARN("No odom info for %s (id: %zu)", bot->getName().c_str(), i);
          have_everything = false;
        }

        if (i != id_) {
          goals_[i] = position;
        }
      } else {
        ROS_WARN("No pose info for %s (id: %zu)", bot->getName().c_str(), i);
        have_everything = false;
      }
    }
    if (!have_everything) {
      ROS_WARN("Not performing update");
      return false;
    }

    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
    bool ok = true;
    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      RVO::Vector2 goalVector;
      if (i == id_) {
        ok = getLeadGoal(&goalVector);
      } else {
        goalVector = goals_[i] - sim_->getAgentPosition(i);
      }
      
      if (RVO::absSq(goalVector) > 1.0f) {
        goalVector = RVO::normalize(goalVector);
      }
      sim_->setAgentPrefVelocity(i, goalVector);

      /*
       * Perturb a little to avoid deadlocks due to perfect symmetry.
       */
      float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
      float dist = std::rand() * 0.0001f / RAND_MAX;

      sim_->setAgentPrefVelocity(i, sim_->getAgentPrefVelocity(i) +
                                 dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
    }
    if (!ok) {
      ROS_WARN("Couldn't get waypoint");
    }
    return ok;
  }

  //============================== MoveServer ===============================//

  MoveServer::MoveServer(const std::string &server_name) :
    pnh_("~"), as_(nh_, server_name, boost::bind(&MoveServer::executeCB, this, _1), false),
    action_name_(ros::names::resolve(server_name)) {
    
    ROS_INFO("Setting up action server '%s'", action_name_.c_str());
    // Get the map
    map_ = requestCSpaceMap("map");
    double max_occ_dist;
    pnh_.param("path_margin", max_occ_dist, 0.1);
    if (map_->max_occ_dist < max_occ_dist) {
      ROS_WARN("path_margin set to %0.2f but map's c-space max_occ_dist is %0.2f",
               max_occ_dist, map_->max_occ_dist);
    }
    // Get bots
    bots_ = BotClient::MakeBots(pnh_);
    ROS_INFO("Listening to %zu bots", bots_.size());
    for (size_t i = 0; i < bots_.size(); ++i) {
      ROS_INFO("  Bot %zu: %s", i, bots_[i]->getName().c_str());
    }

    wrapper_ = RVOWrapper::ROSInit(pnh_, map_, bots_);
    pnh_.param("timestep", timestep_, 0.1);
    wrapper_->setTimestep(timestep_);
    ROS_INFO("Done setting up %s",  action_name_.c_str());
  }

  MoveServer::~MoveServer() {
    delete wrapper_;
    map_free(map_);
    for (size_t i = 0; i < bots_.size(); ++i) {
      delete bots_[i];
    }
  }
  
  void MoveServer::executeCB(const rvo_move::MoveGoalConstPtr &goal) {
    bool success = true;
    std::string prefix = action_name_ + "::MoveServer::executeCB()";
    const char *pref = prefix.c_str();
    ROS_INFO("%s: got request: (% 7.2f, % 7.2f)", pref,
             goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);

    wrapper_->update();

    if (!wrapper_->setGoal(goal->target_pose.pose)) {
      ROS_WARN("No path found");
      as_.setAborted();
      return;
    }
    for (ros::Duration dur(timestep_); ; dur.sleep()) {
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      if (!wrapper_->update()) {
        ROS_WARN("%s Problem while updating RVOWrapper", pref);
        as_.setAborted();
        break;
      }

      if (wrapper_->step()) {
        ROS_INFO("%s Reached destination", pref);
        as_.setSucceeded();
        break;
      }
    }
    bots_[wrapper_->getID()]->pubVel(0.0, 0.0);
  }
  
  void MoveServer::start() {
    as_.start();
  }
}
