#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include "actionlite/action_server.h"
#include <signal.h>

using geometry_msgs::Twist;
using std_srvs::Trigger;
using std_srvs::TriggerRequest;
using std_srvs::TriggerResponse;

class KeyboardControlActionServer: private actionlite::ActionServer<TriggerRequest, TriggerResponse>
{
private:
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber cmd_vel_sub_;
  void cmdVelCb(const Twist cmd_vel);
  virtual bool executeSetup(const TriggerRequest& req);
  virtual bool preemptSetup(TriggerResponse& resp);
public:
  KeyboardControlActionServer();
  ~KeyboardControlActionServer(){};
};

KeyboardControlActionServer::KeyboardControlActionServer()
{
  ros::NodeHandle nh;
  cmd_vel_pub_ = nh.advertise<Twist>("/turtle1/cmd_vel", 1);
}

void KeyboardControlActionServer::cmdVelCb(const Twist cmd_vel)
{
  cmd_vel_pub_.publish(cmd_vel);
}

bool KeyboardControlActionServer::executeSetup(const TriggerRequest&)
{
  ROS_INFO("KEYBOARD CONTROL ACTION SERVER RECEIVE EXECUTE REQUEST!");
  ros::NodeHandle nh;
  cmd_vel_sub_ = nh.subscribe<Twist>("/cmd_vel", 1, &KeyboardControlActionServer::cmdVelCb, this);
  return true;
}

bool KeyboardControlActionServer::preemptSetup(TriggerResponse &resp)
{
  ROS_INFO("KEYBOARD CONTROL ACTION SERVER RECEIVE PREEMPT REQUEST!");
  cmd_vel_sub_.shutdown();
  resp.message = "ATTEMPT CANCEL KEYBORAD CONTROL!";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_control", ros::init_options::NoSigintHandler);
  KeyboardControlActionServer server;
  signal(SIGINT, actionlite::sigintHandler);
  ros::spin();
  return 0;
}