
#ifndef SP_ROBOT_HW_INTERFACE_H
#define SP_ROBOT_HW_INTERFACE_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#define NDOF NSLAVE
#define ENC_FULL 10000
#define PI 3.1415926

class SpHwInterface : public hardware_interface::RobotHW
{
public:
  SpHwInterface();
  ~SpHwInterface();

  //void read();
  //void write();
  void update();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;

private:
  // Raw Data
  unsigned int n_dof_;

  std::vector<std::string> jnt_names_;
  std::vector<double> jnt_curr_pos_;
  std::vector<double> jnt_curr_vel_;
  std::vector<double> jnt_curr_eff_;
  std::vector<double> jnt_cmd_pos_;

  // Hardware Interface
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

};

#endif // SP_ROBOT_HW_INTERFACE_H
