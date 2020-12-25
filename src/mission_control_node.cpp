#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <mission_control/WaypointAction.h>
#include <boost/circular_buffer.hpp>

using std::vector;
using std_srvs::Trigger;
using std_srvs::Empty;
using mission_control::WaypointAction;

int status;
boost::circular_buffer<int> status_history(5);
int cnt, max_cnt;
float timeout_s;

class MissionStatus
{
public:
  enum Status
  {
    MANUAL,
    RUSH_A,
    RUSH_B,
    GO_HOME,
    RETRY,
    TIMEOUT
  };
};

WaypointAction generateWaypointAction(std::string name)
{
  ros::NodeHandle nh("~");
  WaypointAction srv;
  ROS_ASSERT(nh.getParam(name + "/x", srv.request.x));
  ROS_ASSERT(nh.getParam(name + "/y", srv.request.y));
  ROS_ASSERT(nh.getParam(name + "/yaw_deg", srv.request.yaw));
  ROS_ASSERT(nh.getParam(name + "/x_err_ss", srv.request.x_err_ss));
  ROS_ASSERT(nh.getParam(name + "/y_err_ss", srv.request.y_err_ss));
  ROS_ASSERT(nh.getParam(name + "/yaw_err_ss_deg", srv.request.yaw_err_ss));
  ROS_ASSERT(nh.getParam(name + "/timeout_s", srv.request.timeout_s));
  srv.request.yaw *= M_PI / 180.0;
  srv.request.yaw_err_ss *= M_PI / 180.0;
  return srv;
}

void setStatus(int new_status)
{
  status = new_status;
  status_history.push_back(status);
}

void manual()
{
  ROS_INFO("MISSION: MANUAL");
  Trigger srv;
  ROS_ASSERT(ros::service::call("/keyboard_control/execute", srv));
  status = MissionStatus::RUSH_A;
}

void rushA()
{
  ROS_INFO("MISSION: RUSH_A");
  WaypointAction srv = generateWaypointAction("RUSH_A");
  ROS_ASSERT(ros::service::call("/waypoint/execute", srv));
  if (srv.response.status == srv.response.SUCCEEDED)
  {
    setStatus(MissionStatus::RUSH_B);
    return;
  }
  else if (srv.response.status == srv.response.PREEMPTED)
  {
    if (status == MissionStatus::TIMEOUT)
    {
      setStatus(MissionStatus::GO_HOME);
      return;
    }
    setStatus(MissionStatus::MANUAL);
    return;
  }
  else
  {
    ROS_WARN("WAYPOINT TIMEOUT!");
    setStatus(MissionStatus::RETRY);
    return;
  }
}

void rushB()
{
  ROS_INFO("MISSION: RUSH_B");
  WaypointAction srv = generateWaypointAction("RUSH_B");
  ROS_ASSERT(ros::service::call("/waypoint/execute", srv));
  if (srv.response.status == srv.response.SUCCEEDED)
  {
    if (++cnt >= max_cnt)
    {
      setStatus(MissionStatus::GO_HOME);
      return;
    }
    setStatus(MissionStatus::RUSH_A);
    return;
  }
  else if (srv.response.status == srv.response.PREEMPTED)
  {
    if (status == MissionStatus::TIMEOUT)
    {
      setStatus(MissionStatus::GO_HOME);
      return;
    }
    setStatus(MissionStatus::MANUAL);
    return;
  }
  else
  {
    ROS_WARN("WAYPOINT TIMEOUT!");
    setStatus(MissionStatus::RETRY);
    return;
  }
}

void goHome()
{
  ROS_INFO("MISSION: GO_HOME");
  WaypointAction srv = generateWaypointAction("GO_HOME");
  ROS_ASSERT(ros::service::call("/waypoint/execute", srv));
  if (srv.response.status == srv.response.SUCCEEDED)
  {
    setStatus(MissionStatus::MANUAL);
    return;
  }
  else if (srv.response.status == srv.response.PREEMPTED)
  {
    setStatus(MissionStatus::MANUAL);
    return;
  }
  else
  {
    ROS_WARN("WAYPOINT TIMEOUT!");
    setStatus(MissionStatus::RETRY);
    return;
  }
}

void retry()
{
  ROS_INFO("MISSION: RETRY");
  int len = status_history.size();
  ROS_ASSERT(len > 1);
  setStatus(status_history[len - 2]);
}

void timerCb(const ros::TimerEvent&)
{
  ROS_WARN("MISSION TIMEOUT! GO HOME!");
  setStatus(MissionStatus::TIMEOUT);
  Empty srv;
  ros::service::call("/waypoint/preempt", srv);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_control");
  ros::NodeHandle nh("~"); // can't be omitted, must use this to start this node.
  ros::service::waitForService("/keyboard_control/execute");
  ros::service::waitForService("/waypoint/execute");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration dur(1);
  setStatus(MissionStatus::MANUAL);
  cnt = 0;
  ROS_ASSERT(nh.getParam("max_cnt", max_cnt));
  ROS_ASSERT(nh.getParam("timeout_s", timeout_s));
  ros::Timer timer;
  while (ros::ok())
  {
    switch (status)
    {
    case MissionStatus::MANUAL:
      cnt = 0;
      manual();
      timer = nh.createTimer(ros::Duration(timeout_s), timerCb, true);
      break;
    case MissionStatus::RUSH_A:
      rushA();
      break;
    case MissionStatus::RUSH_B:
      rushB();
      break;
    case MissionStatus::GO_HOME:
      goHome();
      break;
    case MissionStatus::RETRY:
      retry();
      break;
    default:
      ROS_ERROR("Status %d undefined! Should never reach this place!", status);
      break;
    }
    dur.sleep(); //avoid ctrl-C stuck
  }
  return 0;
}