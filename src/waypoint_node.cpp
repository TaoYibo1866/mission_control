#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <actionlite/action_server.h>
#include <actionlite/SetUInt8.h>
#include <signal.h>
#include <boost/circular_buffer.hpp>

#include "mission_control/WaypointAction.h"

using geometry_msgs::Twist;
using turtlesim::Pose;
using mission_control::WaypointActionRequest;
using mission_control::WaypointActionResponse;

class WaypointActionServer: private actionlite::ActionServer<WaypointActionRequest, WaypointActionResponse>
{
private:
  boost::circular_buffer<float> x_err_, y_err_, yaw_err_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber pose_sub_;
  ros::Timer timer_;
  bool checkErrSS(float x_err_ss, float y_err_ss, float yaw_err_ss);
  void poseCb(const Pose pose);
  void timerCb(const ros::TimerEvent&);
  virtual bool executeSetup(const WaypointActionRequest& req);
  virtual bool preemptSetup(int status, WaypointActionResponse& resp);
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
    resp.status = WaypointActionResponse::SUCCEEDED;
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

bool WaypointActionServer::executeSetup(const WaypointActionRequest& req)
{
  ROS_INFO("WAYPOINT ACTION SERVER RECEIVE EXECUTE REQUEST!");
  ros::NodeHandle nh;
  pose_sub_ = nh.subscribe<Pose>("/turtle1/pose", 1, &WaypointActionServer::poseCb, this);
  timer_ = nh.createTimer(ros::Duration(req.timeout_s), &WaypointActionServer::timerCb, this, true);
  return true;
}

bool WaypointActionServer::preemptSetup(int status, WaypointActionResponse& resp)
{
  ROS_INFO("WAYPOINT ACTION SERVER RECEIVE PREEMPT REQUEST!");
  resp.status = status;
  return true;
}

void WaypointActionServer::cleanUp()
{
  pose_sub_.shutdown();
  timer_.stop();
  x_err_.clear();
  y_err_.clear();
  yaw_err_.clear();
}

void WaypointActionServer::timerCb(const ros::TimerEvent&)
{
  WaypointActionResponse resp;
  resp.status = WaypointActionResponse::ABORTED_TIMEOUT;
  setResponse(resp);
  return;
}

void sigintHandler(int sig)
{
  actionlite::SetUInt8 srv;
  srv.request.data = WaypointActionResponse::PREEMPTED_CTRL_C;
  ros::service::call("~preempt", srv);
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint", ros::init_options::NoSigintHandler);
  WaypointActionServer server;
  signal(SIGINT, sigintHandler);
  ros::spin();
  return 0;
}