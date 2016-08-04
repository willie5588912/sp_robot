#include <sstream>
#include <sp_robot_hw_interface/sp_robot_hw_interface.h>

int count = 0;
int curr_pos = 0;
int *curr_pos_temp_;
int cmd_pos_temp_[NDOF];

SpHwInterface::SpHwInterface()
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
  jnt_curr_pos_.resize(NDOF);
  jnt_curr_vel_.resize(NDOF);
  jnt_curr_eff_.resize(NDOF);
  jnt_cmd_pos_.resize(NDOF);

  std::fill(jnt_cmd_pos_.begin(), jnt_cmd_pos_.end(), 0);

  // Hardware Interface
  for (size_t i = 0; i < NDOF; ++i)
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

  curr_pos_temp_ = igh_get_curr_pos();
#if 0
  std::cout << "initial pos:" << std::endl;
  for(int i = 0; i < NDOF; i++)
	  std::cout << "jnt" << i << ": " << curr_pos_temp_[i] << std::endl;
#endif
}

SpHwInterface::~SpHwInterface()
{
	igh_stop();
	igh_cleanup();
}

void SpHwInterface::update()
{
#if 0
  // Handle current position (for reading) 
  for(size_t i = 0; i < NDOF - 1; i++)
    jnt_curr_pos_[i] = curr_pos_temp_[i] * (2 * PI) / ENC_FULL;
  jnt_curr_pos_[NDOF - 1] = jnt_cmd_pos_[NDOF - 1];


  curr_pos_temp_ = igh_get_curr_pos();
  // Handle cammand data (for wirting)
  for(size_t jnt; jnt < NDOF; jnt++)
	cmd_pos_temp_[jnt] = (jnt_cmd_pos_[jnt] * ENC_FULL) / (2 * PI);
 
  // Update Ethercat
  curr_pos_temp_ = igh_update(cmd_pos_temp_); 


#if 1
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < NDOF; i++)
    std::cout << jnt_names_[i] << ": "<< cmd_pos_temp_[i] << std::endl;

  std::cout << std::endl;

#endif

#endif



#if 0
  curr_pos = igh_update(10);
  if(count % 100 ==0)
  	std::cout << curr_pos<< std::endl;
  count ++;
#endif
}
#if 0
void SpHwInterface::read()
{
  //std::cout << "read:" << std::endl;
  curr_pos_temp_ = igh_read(); 

  // Convert units
  for(size_t jnt = 0; jnt < NDOF; jnt++)
	  curr_pos_temp_[jnt] *= (2 * PI) / ENC_FULL; 

  for(size_t i = 0; i < NDOF - 1; i++)
  {
    jnt_curr_pos_[i] = curr_pos_temp_[i];
    //std::cout << jnt_names_[i] << ": "<< jnt_curr_pos_[i] << std::endl;
  }
  jnt_curr_pos_[NDOF - 1] = jnt_cmd_pos_[NDOF - 1];


  //std::cout << std::endl; 
}

void SpHwInterface::write()
{
  for(size_t jnt; jnt < NDOF; jnt++)
	cmd_pos_temp_[jnt] = (jnt_cmd_pos_[jnt] * ENC_FULL) / (2 * PI);

  igh_write(cmd_pos_temp_);

#if 1
  std::cout << "write at " << std::setprecision(13) << ros::Time::now().toSec() << " s : " << std::endl;
  for(size_t i = 0; i < NDOF; i++)
    std::cout << jnt_names_[i] << ": "<< (jnt_cmd_pos_[i] * ENC_FULL) / (2 * PI) << std::endl;

  std::cout << std::endl;

#endif
#if 0
  curr_pos = igh_update(10);
  if(count % 100 ==0)
  	std::cout << curr_pos<< std::endl;
  count ++;
#endif
}
#endif

ros::Time SpHwInterface::getTime() const 
{
    return ros::Time::now();
}

ros::Duration SpHwInterface::getPeriod() const 
{
    return ros::Duration(0.001);
}

