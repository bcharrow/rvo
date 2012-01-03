#ifndef BOT_COMM_HPP
#define BOT_COMM_HPP

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace rf {
  class BotClient {
  public:
    BotClient(const ros::NodeHandle &parent, std::string prefix);

    void odomCallback(const nav_msgs::Odometry &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void velCallback(const geometry_msgs::Twist &msg);

    bool haveOdom();
    bool havePose();
    bool haveVel();
    
    std::string getName() const { return name_; }
    nav_msgs::Odometry getOdom();
    geometry_msgs::Pose getPose();
    geometry_msgs::Twist getVel();

    void pubVel(double v, double w);
    
    static std::vector<BotClient *> MakeBots(const ros::NodeHandle& nh);
    static void FreeBots(std::vector<BotClient *> bots);
  protected:
    ros::NodeHandle *nh_;
    ros::Subscriber pose_sub_, odom_sub_, vel_sub_;
    ros::Publisher vel_pub_;
    bool got_odom_, got_pose_, got_vel_;
    std::string name_;
    
    boost::mutex mutex_;

    geometry_msgs::Pose pose_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Twist vel_;
  };
}
#endif
