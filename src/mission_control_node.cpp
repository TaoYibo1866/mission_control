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

void setStatus(int new_status, int &status, vector<int> &status_history)
{
  status = new_status;
  for (int i = 0; i < status_history.size() - 1; i++)
    status_history[i] = status_history[i] + 1;
  status_history.back() = new_status;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_control");
  int status = MissionStatus::MANUAL;
  vector<int> status_history(5);
  ros::ServiceClient keyboard_control_ac;
  while (ros::ok())
  {
    switch (status)
    {
    case MissionStatus::MANUAL:
      break;
    case MissionStatus::GO_HOME:
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