#include <rvo_move/bot_client.hpp>

#include <player_map/rosmap.hpp>

#include <tf/tf.h>

using namespace std;

namespace rf {
  BotClient::BotClient(const ros::NodeHandle &parent, string prefix) :
    got_odom_(false), got_pose_(false) {
    parent.param(prefix, name_, prefix);
    nh_ = new ros::NodeHandle(name_);
    pose_sub_ = nh_->subscribe("amcl_pose", 5, &BotClient::poseCallback, this);
    odom_sub_ = nh_->subscribe("motor/odom", 5, &BotClient::odomCallback, this);

    bool init_pose = (nh_->hasParam("amcl/initial_pose_x") &&
                      nh_->hasParam("amcl/initial_pose_y") &&
                      nh_->hasParam("amcl/initial_pose_a"));

    if (init_pose) {
      got_pose_ = true;
      nh_->getParam("amcl/initial_pose_x", pose_.pose.position.x);
      nh_->getParam("amcl/initial_pose_y", pose_.pose.position.y);
      double yaw;
      nh_->getParam("amcl/initial_pose_a", yaw);
      ROS_INFO("Initializing pose to %0.2f %0.2f %0.2f for %s",
               pose_.pose.position.x, pose_.pose.position.y, yaw,
               nh_->getNamespace().c_str());
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),
                            pose_.pose.orientation);
      pose_.header.stamp = ros::Time::now();
    } else {
      ROS_INFO("Not initializing pose");
    }
    vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 5);
  }

  BotClient::~BotClient() {
    delete nh_;
  }


  void BotClient::odomCallback(const nav_msgs::Odometry &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_odom_ = true;
    odom_ = msg;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": odom = " << msg.twist.twist.linear.x <<
    //                          ", " << msg.twist.twist.angular.z);
  }

  void BotClient::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_pose_ = true;
    pose_.pose = msg.pose.pose;
    pose_.header = msg.header;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": pose = " <<
    //                          pose_.pose.position.x << ", " << pose_.pose.position.y);
  }

  nav_msgs::Odometry BotClient::getOdom() {
    boost::mutex::scoped_lock lock(mutex_);
    return odom_;
  }

  geometry_msgs::Pose BotClient::getPose() {
    boost::mutex::scoped_lock lock(mutex_);
    return pose_.pose;
  }

  bool BotClient::haveOdom() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_odom_;
  }

  bool BotClient::haveOdom(const ros::Duration &d) {
    boost::mutex::scoped_lock lock(mutex_);
    return got_odom_ && (ros::Time::now() - odom_.header.stamp) < d;
  }

  bool BotClient::havePose() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_pose_;
  }

  bool BotClient::havePose(const ros::Duration &d) {
    boost::mutex::scoped_lock lock(mutex_);
    return got_pose_ && (ros::Time::now() - pose_.header.stamp) < d;
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
