#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "mission_control/action_server.h"

using std_srvs::TriggerRequest;
using std_srvs::TriggerResponse;
using std_srvs::SetBoolRequest;
using std_srvs::SetBoolResponse;

class KeyboardControlActionServer: private ActionServer<SetBoolRequest, SetBoolResponse>
{
private:
  virtual bool executeCb(SetBoolRequest &req, SetBoolResponse &resp);
  virtual bool preemptCb(TriggerRequest &req, TriggerResponse &resp);
public:
  KeyboardControlActionServer(){};
  ~KeyboardControlActionServer(){};
};

bool KeyboardControlActionServer::executeCb(SetBoolRequest &req, SetBoolResponse &resp)
{
  ROS_INFO("GOT IT!");
  resp = waitForResponse();
  return true;
}

bool KeyboardControlActionServer::preemptCb(TriggerRequest &req, TriggerResponse &resp)
{
  ROS_INFO("GOT IT!");
  SetBoolResponse action_resp;
  action_resp.message = "ACTION CANCELED!";
  action_resp.success = true;
  setResponse(action_resp);
  resp.success = true;
  resp.message = "OK";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_control");
  KeyboardControlActionServer server;
  ROS_INFO("KEYBOARD CONTROL INITIATED!");
  ros::spin();
  return 0;
}