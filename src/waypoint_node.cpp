#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <mission_control/WaypointAction.h>
#include <signal.h>
#include <boost/circular_buffer.hpp>

#include "actionlite/action_server.h"

using geometry_msgs::Twist;
using turtlesim::Pose;
using mission_control::WaypointActionRequest;
using mission_control::WaypointActionResponse;

class WaypointActionServer: private actionlite::ActionServer<WaypointActionRequest, WaypointActionResponse>
{
private:
  ros::Time start_ts;
  boost::circular_buffer<float> x_err_, y_err_, yaw_err_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber pose_sub_;
  bool checkErrSS(float x_err_ss, float y_err_ss, float yaw_err_ss);
  void poseCb(const Pose pose);
  virtual bool executeSetup(const WaypointActionRequest& req);
  virtual bool preemptSetup(WaypointActionResponse& resp);
  virtual void cleanUp();
public:
  WaypointActionServer();
  ~WaypointActionServer(){};
};

WaypointActionServer::WaypointActionServer(): x_err_(5), y_err_(5), yaw_err_(5)
{
  ros::NodeHandle nh;
  cmd_vel_pub_ = nh.advertise<Twist>("/turtle1/cmd_vel", 1);
}

void WaypointActionServer::poseCb(const Pose pose)
{
  float duration_s = (ros::Time::now() - start_ts).toSec();
  WaypointActionRequest req = getRequest();
  float x_err = req.x - pose.x;
  float y_err = req.y - pose.y;
  float yaw_err = req.yaw - pose.theta;
  float vx = x_err * 0.8;
  float vy = y_err * 0.8;
  float yaw = pose.theta;
  Twist cmd;
  cmd.linear.x = cos(yaw) * vx + sin(yaw) * vy;
  cmd.linear.y = -sin(yaw) * vx + cos(yaw) * vy;
  cmd.angular.z = yaw_err * 1.5;
  cmd_vel_pub_.publish(cmd);

  x_err_.push_back(x_err);
  y_err_.push_back(y_err);
  yaw_err_.push_back(yaw_err);
  if (checkErrSS(req.x_err_ss, req.y_err_ss, req.yaw_err_ss))
  {
    WaypointActionResponse resp;
    resp.x = pose.x;
    resp.y = pose.y;
    resp.yaw = pose.theta;
    resp.duration_s = duration_s;
    resp.status = resp.SUCCEEDED;
    setResponse(resp);
    return;
  }
  if (duration_s > req.timeout_s)
  {
    WaypointActionResponse resp;
    resp.x = pose.x;
    resp.y = pose.y;
    resp.yaw = pose.theta;
    resp.duration_s = duration_s;
    resp.status = resp.TIMEOUT;
    setResponse(resp);
    return;
  }
}

bool WaypointActionServer::checkErrSS(float x_err_ss, float y_err_ss, float yaw_err_ss)
{
  for (int i = 0; i < x_err_.size(); i++)
  {
    if (abs(x_err_[i]) > x_err_ss)
      return false;
  }
  for (int i = 0; i < y_err_.size(); i++)
  {
    if (abs(y_err_[i]) > y_err_ss)
      return false;
  }
  for (int i = 0; i < yaw_err_.size(); i++)
  {
    if (abs(yaw_err_[i]) > yaw_err_ss)
      return false;
  }
  return true;
}

bool WaypointActionServer::executeSetup(const WaypointActionRequest&)
{
  ROS_INFO("WAYPOINT ACTION SERVER RECEIVE EXECUTE REQUEST!");
  start_ts = ros::Time::now();
  ros::NodeHandle nh;
  pose_sub_ = nh.subscribe<Pose>("/turtle1/pose", 1, &WaypointActionServer::poseCb, this);
  return true;
}

bool WaypointActionServer::preemptSetup(WaypointActionResponse &resp)
{
  ROS_INFO("WAYPOINT ACTION SERVER RECEIVE PREEMPT REQUEST!");
  resp.status = resp.PREEMPTED;
  return true;
}

void WaypointActionServer::cleanUp()
{
  pose_sub_.shutdown();
  x_err_.clear();
  y_err_.clear();
  yaw_err_.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint", ros::init_options::NoSigintHandler);
  WaypointActionServer server;
  signal(SIGINT, actionlite::sigintHandler);
  ros::spin();
  return 0;
}