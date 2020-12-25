#include <ros/ros.h>
#include <std_srvs/Trigger.h>

using std::vector;
using std_srvs::Trigger;

class MissionStatus
{
public:
  enum Status
  {
    MANUAL,
    GO_HOME,
    RUSH_A,
    RUSH_B,
    RETRY
  };
};

int status = MissionStatus::MANUAL;
vector<int> status_history(5);

void manual()
{
  ROS_INFO("MISSION: MANUAL");
  Trigger srv;
  ROS_ASSERT(ros::service::call("/keyboard_control/execute", srv));
  status = MissionStatus::GO_HOME;
}

void goHome()
{
  ROS_INFO("MISSION: GO_HOME");
  ros::Rate r(2);
  r.sleep();
  status = MissionStatus::MANUAL;
}

void setStatus(int new_status)
{
  status = new_status;
  for (int i = 0; i < status_history.size() - 1; i++)
    status_history[i] = status_history[i] + 1;
  status_history.back() = new_status;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_control");
  ros::NodeHandle nh; // can't be omitted, must use this to start this node.
  ros::service::waitForService("/keyboard_control/execute");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok())
  {
    switch (status)
    {
    case MissionStatus::MANUAL:
      manual();
      break;
    case MissionStatus::GO_HOME:
      goHome();
      break;
    case MissionStatus::RUSH_A:
      break;
    case MissionStatus::RUSH_B:
      break;
    case MissionStatus::RETRY:
      break;
    default:
      ROS_ERROR("Status %d undefined! Should never reach this place!", status);
      break;
    }
  }
  return 0;
}