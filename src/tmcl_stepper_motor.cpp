/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_stepper_motor.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
StepperMotor::StepperMotor(ros::NodeHandle *p_nh, TmclInterpreter* p_tmcl_int, uint8_t motor_num)  :  
Motor(p_nh, p_tmcl_int, motor_num)
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

  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "closed loop", motor_num, &val))
  {
    comm_mode = (stepper_comm_mode_t) val;
  }
  else
  {
    comm_mode = STEPPER_OPENLOOP_MOTOR;
    ROS_WARN_STREAM("[" << __func__ << "] \"closed loop\" is not available. Set Commutation Mode to Open loop");
  }
  if((p_tmcl_int->executeCmd(TMCL_CMD_GAP, "motor full step resolution", motor_num, &val) ||
      p_tmcl_int->executeCmd(TMCL_CMD_GAP, "fullstep resolution", motor_num, &val)))
  {
    motor_fullstep_resolution = val;
  }
  else
  {
    motor_fullstep_resolution = 0;
    ROS_WARN_STREAM("[" << __func__ << "] \"motor full step resolution\" is not available");
  }
  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "MicrostepResolution", motor_num, &val))
  {
    microstep_resolution = val;
    switch (microstep_resolution)
    {
      case 0:
      microstep_resolution = 1;
      break;
      case 1:
      microstep_resolution = 2;
      break;
      case 2:
      microstep_resolution = 4;
      break;
      case 3:
      microstep_resolution = 8;
      break;
      case 4:
      microstep_resolution = 16;
      break;
      case 5:
      microstep_resolution = 32;
      break;
      case 6:
      microstep_resolution = 64;
      break;
      case 7:
      microstep_resolution = 128;
      break;
      default: 
      microstep_resolution = 256;
      break;
    }
  }
  else
  {
    microstep_resolution = 0;
    ROS_WARN_STREAM("[" << __func__ << "] \"MicrostepResolution\" is not available");
  }

  /* Print units of each commands */
  if(param_wheel_diameter == 0 || motor_fullstep_resolution == 0 || microstep_resolution == 0)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: pps");
  }
  else
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: m/s");
  }
  if(motor_fullstep_resolution == 0 || microstep_resolution == 0)
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
  ROS_INFO("[%s] Motor %d Initialized!\n", __func__, motor_num);
}

/* Publisher Callback */
void StepperMotor::rosPublishTmcInfo(const ros::TimerEvent& event)
{
  int32_t val = 0;
  bool b_drv_statusflag_available = false;

  tmc_info_msg.header.stamp = ros::Time::now();
  tmc_info_msg.header.seq = seq_ctr;
  tmc_info_msg.header.frame_id = s_node_name + "/motor" + std::to_string(motor_num) + "/tmcl_frame";
  tmc_info_msg.interface_name = param_comm_interface_name;
  tmc_info_msg.motor_num = motor_num;

  /* Initialize messages to 0 first */
  tmc_info_msg.board_voltage = 0;
  tmc_info_msg.status_flag = 0;
  tmc_info_msg.velocity = 0;
  tmc_info_msg.position = 0;
  tmc_info_msg.torque = 0;

  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "SupplyVoltage", motor_num, &val))
  {
    tmc_info_msg.board_voltage = val / 10; //converts mV to V
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get SupplyVoltage");
  }

  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "driver error flags", motor_num, &val) || 
     p_tmcl_int->executeCmd(TMCL_CMD_GAP, "DrvStatusFlags", motor_num, &val))
  {
    tmc_info_msg.status = "driver error flags: "+ std::to_string(val);
    b_drv_statusflag_available = true;
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get driver error flags");
  }

  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "extended error flags", motor_num, &val))
  {
    if(b_drv_statusflag_available)
    {
      tmc_info_msg.status = tmc_info_msg.status + " | extended error flags: "+ std::to_string(val);
    }
    else
    {
      tmc_info_msg.status = "extended error flags: " + std::to_string(val);
    }
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get extended error flags");
  }
  
  /* Velocity, Position, Torque */
  if(param_pub_actual_vel)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "ActualVelocity", motor_num, &val))
    {
      //converts pps to linear velocity
      if(param_wheel_diameter == 0 || microstep_resolution == 0 || motor_fullstep_resolution == 0)
      {
        tmc_info_msg.velocity = val * param_add_ratio_vel;
      }
      else
      {
        tmc_info_msg.velocity = ((val * PI * param_wheel_diameter) / (microstep_resolution * motor_fullstep_resolution)) * param_add_ratio_vel;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualVelocity");
    }
  }

  if(param_pub_actual_pos)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "ActualPosition", motor_num, &val))
    {
      //converts pulses to degrees
      if(microstep_resolution == 0 || motor_fullstep_resolution == 0)
      {
        tmc_info_msg.position = val * param_add_ratio_pos;
      }
      else
      {
        tmc_info_msg.position = ((val * ANGULAR_FULL_ROTATION) / (microstep_resolution * motor_fullstep_resolution)) * param_add_ratio_pos;
      }
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualPosition");
    }
  }

  if(param_pub_actual_trq)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "torque", motor_num, &val))
    {
      tmc_info_msg.torque = val * param_add_ratio_trq;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get torque");
    }
  }

  tmc_info_pub.publish(tmc_info_msg);
  seq_ctr++;
}

/* Initialize Subscriber */
void StepperMotor::initSubscriber()
{
  ROS_INFO_STREAM("[StepperMotor::" << __func__ << "] called");

  if(comm_mode == STEPPER_OPENLOOP_MOTOR)
  {
    ROS_INFO_STREAM("[" << __func__ << "] CommutationMode : OPENLOOP");
    tmc_cmd_vel_sub = p_nh_->subscribe(param_tmc_cmd_vel_topic, 1000, &StepperMotor::cmdVelCallback, this);
    tmc_cmd_abspos_sub = p_nh_->subscribe(param_tmc_cmd_abspos_topic, 1000, &StepperMotor::cmdAbsPosCallback, this);
    tmc_cmd_relpos_sub = p_nh_->subscribe(param_tmc_cmd_relpos_topic, 1000, &StepperMotor::cmdRelPosCallback, this);
  }
  else if(comm_mode == STEPPER_CLOSEDLOOP_MOTOR)
  {
    ROS_INFO_STREAM("[" << __func__ << "] CommutationMode : CLOSEDLOOP");
    tmc_cmd_vel_sub = p_nh_->subscribe(param_tmc_cmd_vel_topic, 1000, &StepperMotor::cmdVelCallback, this);
    tmc_cmd_abspos_sub = p_nh_->subscribe(param_tmc_cmd_abspos_topic, 1000, &StepperMotor::cmdAbsPosCallback, this);
    tmc_cmd_relpos_sub = p_nh_->subscribe(param_tmc_cmd_relpos_topic, 1000, &StepperMotor::cmdRelPosCallback, this);
    tmc_cmd_trq_sub = p_nh_->subscribe(param_tmc_cmd_trq_topic, 1000, &StepperMotor::cmdTrqCallback, this);
  }
}

/* Subscriber Callback */
void StepperMotor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  int32_t board_val = 0;

  //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
  if(param_wheel_diameter == 0 || microstep_resolution == 0 || motor_fullstep_resolution == 0)
  {
    board_val = val / param_add_ratio_vel;
  }
  else
  {
    //Formula to convert linear value to pps (unit that the board accepts)
    board_val = ((val * microstep_resolution * motor_fullstep_resolution) / (PI * param_wheel_diameter)) / param_add_ratio_vel;
  }

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: " << board_val);

  if(board_val >= 0)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_ROR, 0, motor_num, &board_val))
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
    if(p_tmcl_int->executeCmd(TMCL_CMD_ROL, 0, motor_num, &board_val))
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
  if(microstep_resolution > 0 && motor_fullstep_resolution > 0)
  {
    convert_const_deg = ((microstep_resolution * motor_fullstep_resolution) / (float) ANGULAR_FULL_ROTATION) / param_add_ratio_pos;
  }
  else
  {
    //inverting position additional ratio
    convert_const_deg = 1 / param_add_ratio_pos;
  }

  unit_val = val * convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: " << unit_val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_MVP, 0, motor_num, &unit_val))
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
  if(microstep_resolution > 0 && motor_fullstep_resolution > 0)
  {
    convert_const_deg = ((microstep_resolution * motor_fullstep_resolution) / (float) ANGULAR_FULL_ROTATION) / param_add_ratio_pos;
  }
  else
  {
    //inverting position additional ratio
    convert_const_deg = 1 / param_add_ratio_pos;
  }

  unit_val = val *  convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: " << unit_val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_MVP, 1, motor_num, &unit_val))
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

  val /=  param_add_ratio_trq;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_SAP, "torque", motor_num, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Torque");
  }
}
