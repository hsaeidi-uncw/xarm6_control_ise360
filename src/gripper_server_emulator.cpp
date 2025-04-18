#include "ros/ros.h"
#include "xarm_msgs/GripperMove.h"


bool GripperMoveCB(xarm_msgs::GripperMove::Request &req, xarm_msgs::GripperMove::Response &res)
{
  if(req.pulse_pos>850)
    req.pulse_pos = 850;
  else if(req.pulse_pos<-100)
    req.pulse_pos = -100;

  res.ret = req.pulse_pos; // fake feedback from the robot
  res.message = "gripper_move, ret = " + std::to_string(res.ret);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_server_emulator");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/xarm/gripper_move", GripperMoveCB);
  ROS_INFO("Ready to receive gripper commands.");
  ros::spin();

  return 0;
}
