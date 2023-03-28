#ifndef REASONABLE_ROBOT_CONTROLLER_H
#define REASONABLE_ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>

#include <reasonable_outdoor_mobile_robot_controller/reasonable_robot_arduino_communicator.h>

class ReasonableRobotHW : public hardware_interface::RobotHW
{
public:
  ReasonableRobotHW();
  ~ReasonableRobotHW();

  ros::NodeHandle nh;

  ros::Duration getPeriod() const { return ros::Duration(0.1); } // loop period is 0.1s
  void init();

  void read();

  void write();

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  struct JointInfo
  {
  public:
    JointInfo()
    : pos_(0.0)
    , vel_(0.0)
    , eff_(0.0)
    , cmd_(0.0)
    {}

    //! Joint position [rad]
    double pos_;
    //! Joint velocity [rad/s]
    double vel_;
    //! Joint effort
    double eff_;
    //! Joint command [rad/s]
    double cmd_;
  };
  std::vector<JointInfo> joint_info_;

private:
  bool getMotorNames(ros::NodeHandle& controller_nh,
                     const std::string& motor_param,
		     std::vector<std::string>& motor_names);
  bool getMotorDirections(ros::NodeHandle& controller_nh,
                     const std::string& motor_param,
		     std::vector<int>& motor_directions);
		     
  const static int MAX_ENCODER = 65535;

  std::string name_;
  size_t motor_num_;
  std::vector<std::string> motor_names_;
  std::vector<int> motor_directions_;
  std::vector<double> prev_rotation_amount_rad_;
  std::string device_name_;
  std::shared_ptr<ReasonableRobotArduinoComunicator> serial_port_;
};
#endif /* REASONABLE_ROBOT_CONTROLLER_H */
