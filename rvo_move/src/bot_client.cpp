#include <rvo_move/bot_client.hpp>

#include <player_map/rosmap.hpp>

using namespace std;

namespace rf {
  BotClient::BotClient(const ros::NodeHandle &parent, string prefix) :
    got_odom_(false), got_pose_(false), got_vel_(false) {
    parent.param(prefix, name_, prefix);
    nh_ = new ros::NodeHandle(name_);
    pose_sub_ = nh_->subscribe("amcl_pose", 5, &BotClient::poseCallback, this);
    odom_sub_ = nh_->subscribe("laser_odom/lodom", 5, &BotClient::odomCallback, this);
    vel_sub_ = nh_->subscribe("motor/cmd_vel", 5, &BotClient::velCallback, this);

    vel_pub_ = nh_->advertise<geometry_msgs::Twist>("motor/cmd_vel", 5);
  }

  void BotClient::odomCallback(const nav_msgs::Odometry &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_odom_ = true;
    odom_ = msg;
  }

  void BotClient::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_pose_ = true;
    pose_ = msg.pose.pose;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": pose = " << pose_.position.x << ", " << pose_.position.y);
  }

  void BotClient::velCallback(const geometry_msgs::Twist &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_vel_ = true;
    vel_ = msg;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": vel = " << vel_.linear.x << ", " << vel_.angular.z);    
  }
  
  nav_msgs::Odometry BotClient::getOdom() {
    boost::mutex::scoped_lock lock(mutex_);    
    return odom_;
  }

  geometry_msgs::Pose BotClient::getPose() {
    boost::mutex::scoped_lock lock(mutex_);    
    return pose_;
  }

  geometry_msgs::Twist BotClient::getVel() {
    boost::mutex::scoped_lock lock(mutex_);    
    return vel_;
  }

  bool BotClient::haveOdom() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_odom_;
  }

  bool BotClient::havePose() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_pose_;
  }
  
  bool BotClient::haveVel() {
    boost::mutex::scoped_lock lock(mutex_);    
    return got_vel_;
  }

  vector<BotClient *> BotClient::MakeBots(const ros::NodeHandle& nh) {
    vector<BotClient *> bots;
    int nbots;
    nh.param("nbots", nbots, 0);
    // instantiate listeners
    for (int id = 0; id < nbots; id++) {
      stringstream ss;
      ss.str();
      ss << "robot" << id;
      bots.push_back(new BotClient(nh, ss.str()));
    }
    return bots;
  }

  void BotClient::FreeBots(vector<BotClient *> bots) {
    for (size_t i = 0; i < bots.size(); ++i) {
      delete bots[i];
    }
  }

  void BotClient::pubVel(double v, double w) {
    geometry_msgs::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    vel_pub_.publish(t);
  }
}
