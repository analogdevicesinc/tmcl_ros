/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_stepper_motor.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
StepperMotor::StepperMotor(ros::NodeHandle *p_nh, TmclInterpreter* p_tmcl_interpreter, 
  uint16_t module_number, uint8_t motor_number) :  
  Motor(p_nh, p_tmcl_interpreter, module_number, motor_number)
{
  ROS_DEBUG_STREAM("[StepperMotor::" <<  __func__ << "] called");
}

/* Destructor */
StepperMotor::~StepperMotor()
{
  ROS_DEBUG_STREAM("[StepperMotor::" <<  __func__ << "] called");
}

void StepperMotor::init()
{
  int32_t val = 0;

  ROS_INFO_STREAM("[StepperMotor::" << __func__ << "] called");

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "closed loop", motor_number_, &val))
  {
    comm_mode_ = (stepper_comm_mode_t) val;
  }
  else
  {
    comm_mode_ = STEPPER_OPENLOOP_MOTOR;
    ROS_WARN_STREAM("[" << __func__ << "] \"closed loop\" is not available. Set Commutation Mode to Open loop");
  }
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "MotorFullStepResolution", motor_number_, &val) ||
     p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "motor full step resolution", motor_number_, &val) ||
     p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "fullstep resolution", motor_number_, &val))
  {
    motor_fullstep_resolution_ = val;
  }
  else
  {
    motor_fullstep_resolution_ = 0;
    ROS_WARN_STREAM("[" << __func__ << "] \"motor full step resolution\" is not available");
  }
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "MicrostepResolution", motor_number_, &val))
  {
    microstep_resolution_ = val;
    switch (microstep_resolution_)
    {
      case 0:
      microstep_resolution_ = 1;
      break;
      case 1:
      microstep_resolution_ = 2;
      break;
      case 2:
      microstep_resolution_ = 4;
      break;
      case 3:
      microstep_resolution_ = 8;
      break;
      case 4:
      microstep_resolution_ = 16;
      break;
      case 5:
      microstep_resolution_ = 32;
      break;
      case 6:
      microstep_resolution_ = 64;
      break;
      case 7:
      microstep_resolution_ = 128;
      break;
      default: 
      microstep_resolution_ = 256;
      break;
    }
  }
  else
  {
    microstep_resolution_ = 0;
    ROS_WARN_STREAM("[" << __func__ << "] \"MicrostepResolution\" is not available");
  }
  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "relative positioning option", motor_number_, &val))
  {
    ROS_INFO_STREAM("[" << __func__ << "] Relative Positioning Option: " << val);
    ROS_INFO_STREAM("[" << __func__ << "] NOTE: " << param_tmc_cmd_relpos_topic_ <<
      " depends on the Relative Positioning Option Axis Parameter");
    ROS_INFO_STREAM("             Refer to datasheet on how to configure.");
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] \"relative positioning option\" is not available");
  }

  /* Print units of each commands */
  if(param_wheel_diameter_ == 0 || motor_fullstep_resolution_ == 0 || microstep_resolution_ == 0)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: pps");
  }
  else
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: m/s");
  }
  if(motor_fullstep_resolution_ == 0 || microstep_resolution_ == 0)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Position unit: pulses");
  }
  else
  {
    ROS_INFO_STREAM("[" << __func__ << "] Position unit: angular degrees");
  }

  ROS_INFO_STREAM("[" << __func__ << "] Torque unit: mA");

  initPublisher();
  this->initSubscriber();
  ROS_INFO("[%s] Motor %d Initialized!\n", __func__, motor_number_);
}

/* Publisher Callback */
void StepperMotor::rosPublishTmcInfo(const ros::TimerEvent& event)
{
  int32_t val = 0;
  bool b_drv_statusflag_available = false;

  tmc_info_msg_.header.stamp = ros::Time::now();
  tmc_info_msg_.header.seq = seq_ctr_;
  tmc_info_msg_.header.frame_id = frame_id_;
  tmc_info_msg_.interface_name = param_comm_interface_name_;
  tmc_info_msg_.motor_number = motor_number_;

  /* Initialize messages to 0 first */
  tmc_info_msg_.board_voltage = 0;
  tmc_info_msg_.status_flag = 0;
  tmc_info_msg_.velocity = 0;
  tmc_info_msg_.position = 0;
  tmc_info_msg_.torque = 0;

  /* No Board Voltage */
  tmc_info_msg_.board_voltage = NAN;

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "driver error flags", motor_number_, &val) || 
     p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "DrvStatusFlags", motor_number_, &val))
  {
    tmc_info_msg_.status = "driver error flags: "+ std::to_string(val);
    b_drv_statusflag_available = true;
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get driver error flags");
  }

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "extended error flags", motor_number_, &val))
  {
    if(b_drv_statusflag_available)
    {
      tmc_info_msg_.status = tmc_info_msg_.status + " | extended error flags: "+ std::to_string(val);
    }
    else
    {
      tmc_info_msg_.status = "extended error flags: " + std::to_string(val);
    }
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get extended error flags");
  }
  
  /* Velocity, Position, Torque */
  if(param_pub_actual_vel_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualVelocity", motor_number_, &val))
    {
      //converts pps to linear velocity
      if(param_wheel_diameter_ == 0 || microstep_resolution_ == 0 || motor_fullstep_resolution_ == 0)
      {
        tmc_info_msg_.velocity = val * param_add_ratio_vel_;
      }
      else
      {
        tmc_info_msg_.velocity = val * ((PI * param_wheel_diameter_) / 
          (microstep_resolution_ * (float)motor_fullstep_resolution_)) * param_add_ratio_vel_;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualVelocity");
    }
  }

  if(param_pub_actual_pos_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualPosition", motor_number_, &val))
    {
      //converts pulses to degrees
      if(microstep_resolution_ == 0 || motor_fullstep_resolution_ == 0)
      {
        tmc_info_msg_.position = val * param_add_ratio_pos_;
      }
      else
      {
        tmc_info_msg_.position = val * (ANGULAR_FULL_ROTATION / 
          (microstep_resolution_ * (float)motor_fullstep_resolution_)) * param_add_ratio_pos_;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualPosition");
    }
  }

  if(param_pub_actual_trq_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "torque", motor_number_, &val))
    {
      tmc_info_msg_.torque = val * param_add_ratio_trq_;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get torque");
    }
  }

  tmc_info_pub_.publish(tmc_info_msg_);
  seq_ctr_++;
}

/* Initialize Subscriber */
void StepperMotor::initSubscriber()
{
  ROS_INFO_STREAM("[StepperMotor::" << __func__ << "] called");

  if(comm_mode_ == STEPPER_OPENLOOP_MOTOR)
  {
    ROS_INFO_STREAM("[" << __func__ << "] CommutationMode : OPENLOOP");
    tmc_cmd_vel_sub_ = p_nh_->subscribe(param_tmc_cmd_vel_topic_, 1000, &StepperMotor::cmdVelCallback, this);
    tmc_cmd_abspos_sub_ = p_nh_->subscribe(param_tmc_cmd_abspos_topic_, 1000, &StepperMotor::cmdAbsPosCallback, this);
    tmc_cmd_relpos_sub_ = p_nh_->subscribe(param_tmc_cmd_relpos_topic_, 1000, &StepperMotor::cmdRelPosCallback, this);
  }
  else if(comm_mode_ == STEPPER_CLOSEDLOOP_MOTOR)
  {
    ROS_INFO_STREAM("[" << __func__ << "] CommutationMode : CLOSEDLOOP");
    tmc_cmd_vel_sub_ = p_nh_->subscribe(param_tmc_cmd_vel_topic_, 1000, &StepperMotor::cmdVelCallback, this);
    tmc_cmd_abspos_sub_ = p_nh_->subscribe(param_tmc_cmd_abspos_topic_, 1000, &StepperMotor::cmdAbsPosCallback, this);
    tmc_cmd_relpos_sub_ = p_nh_->subscribe(param_tmc_cmd_relpos_topic_, 1000, &StepperMotor::cmdRelPosCallback, this);
    tmc_cmd_trq_sub_ = p_nh_->subscribe(param_tmc_cmd_trq_topic_, 1000, &StepperMotor::cmdTrqCallback, this);
  }
}

/* Subscriber Callback */
void StepperMotor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  int32_t board_val = 0;

  //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
  if(param_wheel_diameter_ == 0 || microstep_resolution_ == 0 || motor_fullstep_resolution_ == 0)
  {
    board_val = val / param_add_ratio_vel_;
  }
  else
  {
    //Formula to convert linear value to pps (unit that the board accepts)
    board_val = val * ((microstep_resolution_ * (float)motor_fullstep_resolution_) / (PI * param_wheel_diameter_)) * 
      (1 / param_add_ratio_vel_);
  }

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: " 
    << board_val);

  if(board_val >= 0)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_ROR, 0, motor_number_, &board_val))
    {
      ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Velocity");
    }
  }
  else
  {
    board_val = abs(board_val);
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_ROL, 0, motor_number_, &board_val))
    {
      ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Velocity");
    }
  } 
}
void StepperMotor::cmdAbsPosCallback(const std_msgs::Int32 msg)
{
  float convert_const_deg = 0.00;
  int32_t unit_val = 0;
  int32_t val = msg.data;

  //convert input(degrees) to unit
  if(microstep_resolution_ > 0 && motor_fullstep_resolution_ > 0)
  {
    convert_const_deg = ((microstep_resolution_ * (float)motor_fullstep_resolution_) /  ANGULAR_FULL_ROTATION) *
      (1 / param_add_ratio_pos_);
  }
  else
  {
    //inverting position additional ratio
    convert_const_deg = 1 / param_add_ratio_pos_;
  }

  unit_val = val * convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: "
    << unit_val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, 0, motor_number_, &unit_val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Absolute Position");
  }
}
void StepperMotor::cmdRelPosCallback(const std_msgs::Int32 msg)
{
  float convert_const_deg = 0;
  int32_t unit_val = 0;
  int32_t val = msg.data;

  //convert input(degrees) to unit
  if(microstep_resolution_ > 0 && motor_fullstep_resolution_ > 0)
  {
    convert_const_deg = ((microstep_resolution_ * (float)motor_fullstep_resolution_) /  ANGULAR_FULL_ROTATION) *
      (1 / param_add_ratio_pos_);
  }
  else
  {
    //inverting position additional ratio
    convert_const_deg = 1 / param_add_ratio_pos_;
  }

  unit_val = val *  convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: "
    << unit_val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, 1, motor_number_, &unit_val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Relative Position");
  }
}
void StepperMotor::cmdTrqCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;

  val /=  param_add_ratio_trq_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, "torque", motor_number_, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Torque");
  }
}
