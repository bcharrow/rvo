#include "rvo_wrapper.hpp"
#include <fstream>
#include <tf/tf.h>


#include <nav_msgs/Path.h>


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
  RVOWrapper::RVOWrapper(vector<BotClient *> bots, size_t id, map_t *map,
                         const vector<vector<RVO::Vector2> > &obstacles) :
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

    // TODO: RVO should be able to use occupancy grid information.
    // TODO: Should publish obstacles to RViz
    for (size_t i = 0; i < obstacles.size(); ++i) {
      sim_->addObstacle(obstacles[i]);
    }
    sim_->processObstacles();
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

    vector<vector<RVO::Vector2> > obstacles;
    if (nh.hasParam("obstacle_file")) {
      std::string fname;
      nh.getParam("obstacle_file", fname);
      ifstream ifs(fname.c_str(), ios::in);
      std::string line;
      vector<RVO::Vector2> obstacle;

      while (getline(ifs, line)) {
        if (line == "===") {
          if (obstacle.size() > 0) {
            ROS_DEBUG("Adding obstacle with %zu vertices", obstacle.size());
            obstacles.push_back(obstacle);
            obstacle.clear();
          }
        } else {
          float x, y;
          sscanf(line.c_str(), "%f %f", &x, &y);
          obstacle.push_back(RVO::Vector2(x, y));
        }
      }
    }
    if (obstacles.size() == 0) {
      ROS_WARN("NO OBSTACLES LOADED");
    } else {
      ROS_INFO("Loaded %zu obstacles", obstacles.size());
    }
    RVOWrapper *wrapper = new RVOWrapper(bots, id, map, obstacles);
    wrapper->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                              timeHorizonObst, radius, maxSpeed);
    wrapper->setTimestep(timestep);
    wrapper->setAxelWidth(axel_width);
    wrapper->setWaypointSpacing(waypoint_spacing);
    wrapper->setPathMargin(path_margin);
    wrapper->setGoalTolerance(goal_tol);

    wrapper->addAgents();
    return wrapper;
  }
  
  void RVOWrapper::setAgentDefaults(float neighborDist, size_t maxNeighbors,
                                    float timeHorizon, float timeHorizonObst,
                                    float radius, float maxSpeed) {
    sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                           timeHorizonObst, radius, maxSpeed);
  }

  
  void RVOWrapper::addAgents() {
    for (size_t i = 0; i < bots_.size(); ++i) {
      sim_->addAgent(RVO::Vector2());
    }
  }

  bool RVOWrapper::step() {
    // TODO: synchornize ROS clock and sim clock.
    sim_->doStep();
    RVO::Vector2 vel = sim_->getAgentVelocity(id_);
    // ROS_INFO("%s's actual xy_vel:    % 6.2f % 6.2f",
    //          bots_[id_]->getName().c_str(), vel.x(), vel.y());

    // ROS_INFO("Sim vel: % 6.2f % 6.2f", vel.x(), vel.y());
    
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
    if (RVO::abs(sim_->getAgentPosition(id_) - goal_) > goal_tol_) {
      vx = (u(0) + u(1)) / 2.0;
      vw = (u(1) - u(0)) / L;
    } else {
      vx = 0.0;
      vw = 0.0;
      at_dest = true;
    }
    bots_[id_]->pubVel(vx, vw);
    ROS_DEBUG("Theta: % 6.2f", theta);
    ROS_DEBUG("L: % 6.2f", L);
    ROS_DEBUG_STREAM("m: " << m);
    ROS_DEBUG_STREAM("m_inv: " << m.inverse());
    ROS_DEBUG_STREAM("vel:       " << setprecision(2) << v.transpose());
    ROS_DEBUG_STREAM("wheel vel: " << setprecision(2) << u.transpose());
    ROS_DEBUG("sim pose:  % 6.2f % 6.2f", sim_->getAgentPosition(id_).x(),
             sim_->getAgentPosition(id_).y());
    ROS_DEBUG("command:   % 6.2f % 6.2f\n", vx, vw);
    return at_dest;
  }

  std::vector<geometry_msgs::Pose> RVOWrapper::setGoal(const geometry_msgs::Pose& p) {
    RVO::Vector2 start = pose_to_rvo(bots_[id_]->getPose());
    goal_ = pose_to_rvo(p);
    waypoints_.clear();

    PointVector path = astar(rvo_to_eig(start), rvo_to_eig(goal_), map_, path_margin_);
    if (path.size() != 0) {  
      waypoints_.push_back(eig_to_rvo(path[0]));
      for (size_t i = 0; i < path.size() - 1; ++i) {
        const RVO::Vector2& prev = waypoints_.back();
        const RVO::Vector2& curr = eig_to_rvo(path[i]);
        if (!sim_->queryVisibility(prev, curr, los_margin_) ||
            RVO::abs(prev - curr) > way_spacing_) {
          waypoints_.push_back(eig_to_rvo(path[i-1]));
        }
      }
      waypoints_.push_back(eig_to_rvo(path.back()));
      ROS_DEBUG("Waypoint path has %zu vertices", waypoints_.size());
    }

    std::vector<geometry_msgs::Pose> ros_path;
    ros_path.resize(waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      tf::poseTFToMsg(tf::Pose(tf::createIdentityQuaternion(),
                               tf::Vector3(waypoints_[i].x(), waypoints_[i].y(), 0)),
                      ros_path[i]);
    }
    return ros_path;
  }

  bool RVOWrapper::getLeadGoal(RVO::Vector2 *goal) {
    // Direct robot towards successor of point that it is closest to
    float min_dist = std::numeric_limits<float>::infinity();
    int min_ind = -1;
    RVO::Vector2 pos =  sim_->getAgentPosition(id_);
    for (size_t wayind = 0; wayind < waypoints_.size(); ++wayind) {
      if (!sim_->queryVisibility(pos, waypoints_[wayind], sim_->getAgentRadius(id_))) {
        continue;
      }
      float dist = RVO::absSq(pos - waypoints_[wayind]);
      if (dist < min_dist) {
        min_dist = dist;
        min_ind = wayind;
      }
    }

    if (min_ind == -1) {
      *goal = RVO::Vector2();
      return false;
    } else {
      while (static_cast<unsigned>(min_ind + 1) < waypoints_.size() &&
             sim_->queryVisibility(pos, waypoints_[min_ind+1], los_margin_)) {
        ++min_ind;
      }
      if (!sim_->queryVisibility(pos, waypoints_[min_ind])) {
        ROS_WARN("RVOWrapper::getLeadGoal() %s's next waypoint is not visible",
                 bots_[id_]->getName().c_str());
      }
      
      *goal = waypoints_[min_ind] - pos;
      if (RVO::absSq(*goal) > 1.0f) {
        *goal = RVO::normalize(*goal);
      }
      
      ROS_DEBUG("Advancing towards goal");
      ROS_DEBUG("occ dist: % 6.2f",
                map_get_cell(map_, pos.x(), pos.y(), 0.0)->occ_dist);
      ROS_DEBUG("min_ind: %i", min_ind);
      ROS_DEBUG("position:   % 6.2f % 6.2f", pos.x(), pos.y());
      ROS_DEBUG("waypoint:   % 6.2f % 6.2f",
                waypoints_[min_ind].x(), waypoints_[min_ind].y());
      ROS_DEBUG("%s's actual goalVec: % 6.2f % 6.2f",
                bots_[id_]->getName().c_str(), goal->x(), goal->y());
      ROS_DEBUG("Num obs: %zu", sim_->getAgentNumObstacleNeighbors(id_));
      return true;
    }
  }

  bool RVOWrapper::syncState() {
    // Update RVO simulator with latest position & velocitys from ROS
    bool have_everything = true;
    ros::Duration d(10.0);
    
    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      BotClient *bot = bots_[i];
      if (bot->havePose(d)) {
        RVO::Vector2 position = pose_to_rvo(bot->getPose());
        sim_->setAgentPosition(i, position);
      } else {
        ROS_WARN("No pose info for %s (id: %zu)", bot->getName().c_str(), i);
        have_everything = false;
      }
      
      if (bot->haveOdom(d)) {
        RVO::Vector2 vel = odom_to_rvo(bot->getOdom(), bot->getPose());
        sim_->setAgentVelocity(i, vel);
      } else {
        ROS_WARN_THROTTLE(1.0, "No odom info for %s (id: %zu)",
                          bot->getName().c_str(), i);
        have_everything = false;
      }
    }

    return have_everything;
  }

  bool RVOWrapper::setVelocities() {
    // Set preferred velocities
    bool ok = true;
    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      RVO::Vector2 goalVector;
      if (i == id_) {
        ok = getLeadGoal(&goalVector);
        if (!ok) {
          ROS_WARN("%s problem getting goal", bots_[id_]->getName().c_str());
        } else {
          // Perturb a little to avoid deadlocks due to perfect symmetry.
          float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
          float dist = std::rand() * 0.0001f / RAND_MAX;
          sim_->setAgentPrefVelocity(i, goalVector + 
                                     dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
        }
      } else {
        BotClient *bot = bots_[i];
        RVO::Vector2 goal = odom_to_rvo(bot->getOdom(), bot->getPose());
        ROS_DEBUG("%s thinks %s's xy_vel:  % 6.2f % 6.2f", bots_[id_]->getName().c_str(),
                  bot->getName().c_str(), goal.x(), goal.y());
        sim_->setAgentPrefVelocity(i, goalVector);
      }
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

    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5, true);
    pnh_.param("map_frame_id", tf_frame_, std::string("/map"));
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = tf_frame_; 
    path.poses.resize(0);
    path_pub_.publish(path);
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
    std::string prefix = action_name_ + "::MoveServer";
    const char *pref = prefix.c_str();
    ROS_INFO("%s got request: (% 7.2f, % 7.2f)", pref,
             goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
    
    std::vector<geometry_msgs::Pose> path = wrapper_->setGoal(goal->target_pose.pose);
    // Publish the path
    nav_msgs::Path nav_path;
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = tf_frame_;
    nav_path.poses.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      nav_path.poses[i].header.stamp = ros::Time::now();
      nav_path.poses[i].header.frame_id = tf_frame_;
      nav_path.poses[i].pose = path[i];
    }
    path_pub_.publish(nav_path);

    if (path.size() == 0) {
      ROS_WARN("No path found");
      as_.setAborted();
      return;
    }
    for (ros::Duration dur(timestep_); ; dur.sleep()) {      
      if (!ros::ok()) {
        ROS_INFO("%s Ros shutdown", action_name_.c_str());
        as_.setPreempted();
        break;
      }

      if (as_.isPreemptRequested()) {
        ROS_INFO("%s Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!wrapper_->syncState()) {
        ROS_WARN_THROTTLE(1.0, "%s Problem synchronizing state", pref);
      }
     
      if (!wrapper_->setVelocities()) {
        ROS_WARN("%s Problem while setting agent velocities", pref);
        as_.setAborted();
        break;
      }

      if (wrapper_->step()) {
        ROS_INFO("%s Reached destination", pref);
        as_.setSucceeded();
        break;
      }
    }
    nav_path.poses.resize(0);
    path_pub_.publish(nav_path);
    if (!as_.isNewGoalAvailable()) {
      bots_[wrapper_->getID()]->pubVel(0.0, 0.0);
    }
  }
  
  void MoveServer::start() {
    as_.start();
  }
}
