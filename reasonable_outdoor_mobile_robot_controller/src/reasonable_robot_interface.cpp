#include <string>
#include <ros/package.h>
#include <reasonable_outdoor_mobile_robot_controller/reasonable_robot_interface.h>
#include <reasonable_outdoor_mobile_robot_controller/reasonable_robot_arduino_communicator.h>

ReasonableRobotHW::ReasonableRobotHW()
{
  name_ = "reasonable_outdoor_mobile_robot_controller";

  if(!getMotorNames(nh, "reasonable_outdoor_mobile_robot_controller/motor_name", motor_names_))
  {
    ROS_ERROR_NAMED(name_, "Wheel param is not found.");
    return;
  }
  if(!getMotorDirections(nh, "reasonable_outdoor_mobile_robot_controller/motor_direction", motor_directions_))
  {
    ROS_ERROR_NAMED(name_, "Wheel param is not found.");
    return;
  }
  device_name_ = "/dev/ttyACM0";
  nh.param("reasonable_outdoor_mobile_robot_controller/device_name", device_name_, device_name_);
  motor_num_ = motor_names_.size();

  joint_info_.resize(motor_num_);
  prev_rotation_amount_rad_.resize(motor_num_);
  for(size_t i = 0; i < motor_num_; i++)
  {
    hardware_interface::JointStateHandle state_handle(motor_names_[i], &(joint_info_[i].pos_), &(joint_info_[i].vel_), &(joint_info_[i].eff_));
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(motor_names_[i]), &(joint_info_[i].cmd_));
    jnt_vel_interface.registerHandle(vel_handle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
}

ReasonableRobotHW::~ReasonableRobotHW()
{
}

void
ReasonableRobotHW::init()
{
  serial_port_ = std::make_shared<ReasonableRobotArduinoComunicator>(device_name_, motor_num_);
  if (!serial_port_->isConnected()) {
    ROS_ERROR_NAMED(name_, "Device can not be opened");
    return;
  }
}

void
ReasonableRobotHW::read()
{
  std::vector<float> rotation_amount_rad(motor_num_);
  std::vector<float> read_speed_radps(motor_num_);
  std::vector<float> read_current(motor_num_);

  serial_port_->readRad(rotation_amount_rad, read_speed_radps, read_current);
  for (size_t i = 0; i < motor_num_; i++) 
  {
    float diff_rotation_rad = rotation_amount_rad[i] - prev_rotation_amount_rad_[i];
    if (diff_rotation_rad > M_PI)
    {
      diff_rotation_rad -= 2*M_PI;
    }
    if (diff_rotation_rad < -M_PI)
    {
      diff_rotation_rad += 2*M_PI;
    }
    prev_rotation_amount_rad_[i] = rotation_amount_rad[i];
    joint_info_[i].pos_ += diff_rotation_rad * motor_directions_[i];

    joint_info_[i].vel_ = read_speed_radps[i] * motor_directions_[i];

    joint_info_[i].eff_ = read_current[i] * motor_directions_[i];
  }

}

void
ReasonableRobotHW::write()
{
  std::vector<float> send_command_radps(motor_num_);

  for (size_t i = 0; i < motor_num_; i++) 
  {
    send_command_radps[i] = joint_info_[i].cmd_ * motor_directions_[i];
  }
  serial_port_->writeRadps(send_command_radps);
}

bool
ReasonableRobotHW::getMotorNames(ros::NodeHandle& controller_nh,
  const std::string& motor_param,
  std::vector<std::string>& motor_names)
{
  XmlRpc::XmlRpcValue motor_list;
  if (!controller_nh.getParam(motor_param, motor_list))
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Couldn't retrieve motor param '" << motor_param << "'.");
    return false;
  }

  if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (motor_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_,
        "Motor param '" << motor_param << "' is an empty list.");
      return false;
    }

    for (size_t i = 0; i < motor_list.size(); i++)
    {
      if (motor_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM_NAMED(name_,
          "Motor param '" << motor_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    motor_names.resize(motor_list.size());
    for (size_t i = 0; i < motor_list.size(); i++)
    {
      motor_names[i] = static_cast<std::string>(motor_list[i]);
    }
  }
  else if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    motor_names.push_back(motor_list);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Motor param '" << motor_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

bool
ReasonableRobotHW::getMotorDirections(ros::NodeHandle& controller_nh,
  const std::string& motor_param,
  std::vector<int>& motor_directions)
{
  XmlRpc::XmlRpcValue motor_list;
  if (!controller_nh.getParam(motor_param, motor_list))
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Couldn't retrieve motor param '" << motor_param << "'.");
    return false;
  }

  if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (motor_list.size() == 0)
    {
      ROS_ERROR_STREAM_NAMED(name_,
        "Motor param '" << motor_param << "' is an empty list.");
      return false;
    }

    for (size_t i = 0; i < motor_list.size(); i++)
    {
      if (motor_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM_NAMED(name_,
          "Motor param '" << motor_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    motor_directions.resize(motor_list.size());
    for (size_t i = 0; i < motor_list.size(); i++)
    {
      motor_directions[i] = static_cast<int>(motor_list[i]);
    }
  }
  else if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    motor_directions.push_back(motor_list);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_,
      "Motor param '" << motor_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}
