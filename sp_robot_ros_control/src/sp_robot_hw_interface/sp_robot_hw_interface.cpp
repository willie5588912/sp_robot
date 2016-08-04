#include <sstream>
#include <sp_robot_hw_interface/sp_robot_hw_interface.h>

extern "C" {
	#include "sp_ethercat/sp_ethercat.h"
}

int count = 0;
int curr_pos = 0;
int eth_enc_offset_[3];
int *curr_pos_temp_;
int cmd_pos_temp_[NDOF];
int cmd_pos_offset_[NDOF];

SpHwInterface::SpHwInterface():
    n_dof_(3)
{
  // Cleanup
  jnt_curr_pos_.clear();
  jnt_curr_vel_.clear();
  jnt_curr_eff_.clear();
  jnt_cmd_pos_.clear();
 
  // Joints
  jnt_names_.push_back("joint1");
  jnt_names_.push_back("joint2");
  jnt_names_.push_back("joint_eef");

  // Raw Data
  jnt_curr_pos_.resize(n_dof_);
  jnt_curr_vel_.resize(n_dof_);
  jnt_curr_eff_.resize(n_dof_);
  jnt_cmd_pos_.resize(n_dof_);

  // Hardware Interface
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_names_[i], &jnt_curr_pos_[i], &jnt_curr_vel_[i], &jnt_curr_eff_[i]));

    jnt_pos_interface_.registerHandle(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(jnt_names_[i]), &jnt_cmd_pos_[i]));

    ROS_DEBUG_STREAM("Registered joint '" << jnt_names_[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);

  // Initialize Ethercat
  igh_configure(); 
  igh_start(); 

  // Initialize joint value
  curr_pos_temp_ = igh_get_home_pos();
  for(int i = 0; i < n_dof_; i++)
  {
    cmd_pos_offset_[i] = 10;
    jnt_cmd_pos_[i] = 0;
  }
  //std::cout << curr_pos_temp_[0] << std::endl;
  //std::cout << curr_pos_temp_[1] << std::endl;
}

SpHwInterface::~SpHwInterface()
{
	igh_stop();
	igh_cleanup();
}

#if 0
void SpHwInterface::read()
{
  //std::cout << "read:" << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
  {
    jnt_curr_pos_[i] = jnt_cmd_pos_[i];
    //std::cout << jnt_names_[i] << ": "<< jnt_curr_pos_[i] << std::endl;
  }

  //std::cout << std::endl; 
}

void SpHwInterface::write()
{
  int *curr_pos_temp_;
#if 0
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < n_dof_; i++)
    std::cout << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << std::endl;

  std::cout << std::endl;
#endif
  eth_enc_offset_[0] = count;
  eth_enc_offset_[1] = 2 * count;
  curr_pos_temp_ = igh_update(eth_enc_offset_);
  if(count % 100 ==0)
  {
	for(int i = 0; i < 2; i++)
  		std::cout << curr_pos_temp_[i] << " ";
	std::cout << std::endl;
  }
  count ++;
}
#endif

void SpHwInterface::update()
{
#if 0
// Handle current position (for reading) 
  for(size_t i = 0; i < NDOF - 1; i++)
    jnt_curr_pos_[i] = curr_pos_temp_[i] * (2 * PI) / ENC_FULL;
  jnt_curr_pos_[NDOF - 1] = jnt_cmd_pos_[NDOF - 1];

  // Handle cammand data (for wirting)
  for(size_t jnt; jnt < NDOF; jnt++)
    cmd_pos_offset_[jnt] = ((jnt_cmd_pos_[jnt] - 0) * ENC_FULL) / (2 * PI);

#if 1
  if(count % 100 ==0)
  { 
	  std::cout << "cmd_pos_offset[0] = " << cmd_pos_offset_[0] << std::endl;
	  std::cout << "cmd_pos_offset[1] = " << cmd_pos_offset_[1] << std::endl;
	  std::cout << std::endl;

	  std::cout << "curr_pos_temp_[0] = " << curr_pos_temp_[0] << std::endl;
	  std::cout << "curr_pos_temp_[1] = " << curr_pos_temp_[1] << std::endl;
	  std::cout << std::endl;
  }
#endif

  // Update Ethercat
  curr_pos_temp_ = igh_update(cmd_pos_offset_);

#if 1
  if(count % 100 ==0)
  {
	  std::cout << "read at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
	  for(size_t i = 0; i < n_dof_; i++)
		std::cout << jnt_names_[i] << ": "<< curr_pos_temp_[i] << std::endl;

	  std::cout << std::endl;
  }
  count ++;
#endif

#if 0
  if(count % 100 ==0)
  {
	  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
	  for(size_t i = 0; i < n_dof_; i++)
		std::cout << jnt_names_[i] << ": "<< jnt_cmd_pos_[i] << std::endl;

	  std::cout << std::endl;
  }
  count ++;
#endif
#endif

#if 1
  int *curr_pos_temp_;
  eth_enc_offset_[0] = count;
  eth_enc_offset_[1] = 2 * count;
  curr_pos_temp_ = igh_update(eth_enc_offset_);
  if(count % 100 ==0)
  {
	for(int i = 0; i < 2; i++)
  		std::cout << curr_pos_temp_[i] << " ";
	std::cout << std::endl;
  }
  count ++;
#endif
}

ros::Time SpHwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration SpHwInterface::getPeriod() const 
{
    return ros::Duration(0.001);
}

