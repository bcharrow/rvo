#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <actionlib/client/simple_action_client.h>

#include <rvo_move/MoveAction.h>

using namespace std;

class MoveClient {
public:
  MoveClient(std::string name = "move_server") :
    ac_(name, true), action_name_(name) {
    ROS_INFO("Waiting for action server '%s'",
             ros::names::resolve(name).c_str());
    ac_.waitForServer();
    if (!ac_.isServerConnected()) {
      ROS_ERROR("Couldn't connect to server");
      ros::shutdown();
    } else {
      ROS_INFO("Action server started");
    }

    sub_ = nh_.subscribe("goal", 1, &MoveClient::sendGoal, this);
  }

  void sendGoal(const geometry_msgs::PoseStamped& goal_msg) {
    rvo_move::MoveGoal goal;
    goal.target_pose.pose.position.x = goal_msg.pose.position.x;
    goal.target_pose.pose.position.y = goal_msg.pose.position.y;
    ac_.sendGoal(goal, boost::bind(&MoveClient::doneCb, this, _1, _2));
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const rvo_move::MoveResultConstPtr& result) {
    ROS_INFO("Finished");
    ROS_INFO_STREAM("" << state.toString());
    ROS_INFO_STREAM("" << *result);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  actionlib::SimpleActionClient<rvo_move::MoveAction> ac_;
  std::string action_name_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rvo_move");
  MoveClient mc("move_server");
  ros::spin();
  return 0;
}
