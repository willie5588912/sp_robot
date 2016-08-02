#include <sp_robot_hw_interface/sp_robot_hw_interface.h>


int main(int argc, char **argv)
{
  //initialize ros
  ros::init(argc, argv, "sp_robot_ros_control_node");
  ros::NodeHandle nh;

  SpHwInterface robot;
  controller_manager::ControllerManager cm(&robot, nh);

  //start loop
  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
     robot.read();
     cm.update(robot.getTime(), robot.getPeriod());
     robot.write();
	 
	 rate.sleep();
  }
  spinner.stop();

  return 0;
}
