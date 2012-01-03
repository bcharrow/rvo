#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <actionlib/client/simple_action_client.h>

#include <rvo_move/MoveAction.h>

using namespace std;

class MoveClient {
public:
  MoveClient(std::string name = "move_server") :
    ac_(name, true), action_name_(name), done_(false) {
    ROS_INFO("Waiting for action server '%s'", ros::names::resolve(name).c_str());
    ac_.waitForServer(ros::Duration(5.0));
    if (!ac_.isServerConnected()) {
      ROS_ERROR("Couldn't connect to server");
      ros::shutdown();
    } else {
      ROS_INFO("Action server started");
    }
  }

  void sendGoal(const geometry_msgs::PoseWithCovarianceStamped& dest) {
    rvo_move::MoveGoal goal;
    goal.target_pose.pose.position.x = dest.pose.pose.position.x;
    goal.target_pose.pose.position.y = dest.pose.pose.position.y;
    ac_.sendGoal(goal, boost::bind(&MoveClient::doneCb, this, _1, _2));
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const rvo_move::MoveResultConstPtr& result) {
    ROS_INFO("Finished");
    ROS_INFO_STREAM("" << state.toString());
    ROS_INFO_STREAM("" << *result);
    ros::shutdown();
  }

private:
  actionlib::SimpleActionClient<rvo_move::MoveAction> ac_;
  std::string action_name_;
  boost::mutex mut_;
  boost::condition_variable cond_;
  bool done_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rvo_move");
  ros::NodeHandle nh("~");

  double x, y;
  if (!nh.getParam("x", x) || !nh.getParam("y", y)) {
    ROS_ERROR("Need to specify x, y destination");
    return 1;
  }
  
  MoveClient mc("move_server");
  geometry_msgs::PoseWithCovarianceStamped goal;
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  
  mc.sendGoal(goal);
  ROS_INFO("Sent goal..");  
  ros::spin();
  return 0;
}
