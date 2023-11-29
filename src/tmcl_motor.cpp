/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_motor.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
Motor::Motor(ros::NodeHandle *p_nh, TmclInterpreter* p_tmcl_interpreter, 
  uint16_t module_number, uint8_t motor_number) : 
  p_nh_ (p_nh), 
  p_tmcl_interpreter_(p_tmcl_interpreter),
  module_number_(module_number),
  motor_number_(motor_number)
{
  ROS_DEBUG_STREAM("[Motor::" <<  __func__ << "] called");
  initParams();
}

/* Destructor */
Motor::~Motor()
{
  ROS_DEBUG_STREAM("[Motor::" <<  __func__ << "] called");
  p_tmcl_interpreter_ = nullptr;
  p_nh_ = nullptr;
}

void Motor::init()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  initPublisher();
  initSubscriber();
  ROS_INFO("[%s] Motor %d Initialized!\n", __func__, motor_number_);
}

/* Initialize Parameters */
void Motor::initParams()
{

  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  s_node_name_ = ros::this_node::getName();

  const std::string s_en_pub_tmc_info = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/en_pub_tmc_info";
  if(!p_nh_->getParam(s_en_pub_tmc_info, param_en_pub_tmc_info_))
  {
    param_en_pub_tmc_info_ = false;
    p_nh_->setParam(s_en_pub_tmc_info, param_en_pub_tmc_info_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get en_pub_tmc_info for motor" << std::to_string(motor_number_)
      << ", setting to default value: " << std::boolalpha << param_en_pub_tmc_info_);
  }
  const std::string s_pub_actual_vel = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/pub_actual_vel";
  if(!p_nh_->getParam(s_pub_actual_vel, param_pub_actual_vel_))
  {
    param_pub_actual_vel_ = false;
    p_nh_->setParam(s_pub_actual_vel, param_pub_actual_vel_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_vel for motor" << std::to_string(motor_number_) 
      << ", setting to default value: " << std::boolalpha << param_pub_actual_vel_);
  }
  const std::string s_pub_actual_pos = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/pub_actual_pos";
  if(!p_nh_->getParam(s_pub_actual_pos, param_pub_actual_pos_))
  {
    param_pub_actual_pos_ = false;
    p_nh_->setParam(s_pub_actual_pos, param_pub_actual_pos_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_pos for motor" << std::to_string(motor_number_) 
      << ", setting to default value: " << std::boolalpha << param_pub_actual_pos_);
  }
  const std::string s_pub_actual_trq = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/pub_actual_trq";
  if(!p_nh_->getParam(s_pub_actual_trq, param_pub_actual_trq_))
  {
    param_pub_actual_trq_ = false;
    p_nh_->setParam(s_pub_actual_trq, param_pub_actual_trq_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_trq for motor" << std::to_string(motor_number_) 
      << ", setting to default value: " << std::boolalpha << param_pub_actual_trq_);
  }
  const std::string s_tmc_info_topic = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/tmc_info_topic";
  if(!p_nh_->getParam(s_tmc_info_topic, param_tmc_info_topic_))
  {
    param_tmc_info_topic_ = "/tmc_info_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_info_topic, param_tmc_info_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_info_topic for motor" << std::to_string(motor_number_) 
      << ", setting to default value: " << param_tmc_info_topic_);
  }
  const std::string s_tmc_cmd_vel_topic = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/tmc_cmd_vel_topic";
  if(!p_nh_->getParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic_))
  {
    param_tmc_cmd_vel_topic_ = "/cmd_vel_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_vel_topic for motor" << std::to_string(motor_number_) 
      << ", setting to default value: " << param_tmc_cmd_vel_topic_);
  }
  const std::string s_tmc_cmd_abspos_topic = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/tmc_cmd_abspos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic_))
  {
    param_tmc_cmd_abspos_topic_ = "/cmd_abspos_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_abspos_topic for motor" 
      << std::to_string(motor_number_) << ", setting to default value: " << param_tmc_cmd_abspos_topic_);
  }
  const std::string s_tmc_cmd_relpos_topic = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/tmc_cmd_relpos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic_))
  {
    param_tmc_cmd_relpos_topic_ = "/cmd_relpos_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_relpos_topic for motor"
      << std::to_string(motor_number_) << ", setting to default value: " << param_tmc_cmd_relpos_topic_);
  }
  const std::string s_tmc_cmd_trq_topic = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/tmc_cmd_trq_topic";
  if(!p_nh_->getParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic_))
  {
    param_tmc_cmd_trq_topic_ = "/cmd_trq_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_trq_topic for motor" << std::to_string(motor_number_)
      << ", setting to default value: " << param_tmc_cmd_trq_topic_);
  }
  const std::string s_wheel_diameter = s_node_name_ + "/motor"+ std::to_string(motor_number_) + "/wheel_diameter";
  if(!p_nh_->getParam(s_wheel_diameter, param_wheel_diameter_))
  {
    param_wheel_diameter_ = 0;
    p_nh_->setParam(s_wheel_diameter, param_wheel_diameter_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get wheel_diameter for motor" << std::to_string(motor_number_)
      << ", setting to default value: " << param_wheel_diameter_);
  }
  const std::string s_additional_ratio_vel = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/additional_ratio_vel";
  if(!p_nh_->getParam(s_additional_ratio_vel, param_add_ratio_vel_))
  {
    param_add_ratio_vel_ = 1;
    p_nh_->setParam(s_additional_ratio_vel, param_add_ratio_vel_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_vel for motor"
      << std::to_string(motor_number_) << ", setting to default value: " << param_add_ratio_vel_);
  }
  const std::string s_additional_ratio_pos = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/additional_ratio_pos";
  if(!p_nh_->getParam(s_additional_ratio_pos, param_add_ratio_pos_))
  {
    param_add_ratio_pos_ = 1;
    p_nh_->setParam(s_additional_ratio_pos, param_add_ratio_pos_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_pos for motor"
      << std::to_string(motor_number_) << ", setting to default value: " << param_add_ratio_pos_);
  }
  const std::string s_additional_ratio_trq = s_node_name_ + "/motor"+ std::to_string(motor_number_) + 
    "/additional_ratio_trq";
  if(!p_nh_->getParam(s_additional_ratio_trq, param_add_ratio_trq_))
  {
    param_add_ratio_trq_ = 1;
    p_nh_->setParam(s_additional_ratio_trq, param_add_ratio_trq_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_trq for motor"
      << std::to_string(motor_number_) << ", setting to default value: " << param_add_ratio_trq_);
  }

  /* Parameters that are already validated in tmcl_ros.cpp. Only need to get the param value */
  const std::string s_pub_rate_tmc_info= s_node_name_ + "/pub_rate_tmc_info";
  (void)p_nh_->getParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info_);
  const std::string s_comm_interface_name= s_node_name_ + "/comm_interface_name";
  (void)p_nh_->getParam(s_comm_interface_name, param_comm_interface_name_);
}

/* Initialize Publisher */
void Motor::initPublisher()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  // Set frame ID
  s_namespace_ = ros::this_node::getNamespace();
  /* Checks if namespace is empty if not, remove "/" */
  if(s_namespace_.compare("/") == 0)
  {
    frame_id_ = "/tmcm"  + std::to_string(module_number_) +  "_mtr" + std::to_string(motor_number_) + "_frame";
  }
  else
  {
    s_namespace_.erase(s_namespace_.begin());
    frame_id_ = s_namespace_ + "/tmcm"  + std::to_string(module_number_) +  "_mtr" + std::to_string(motor_number_) + 
      "_frame";
  }

  if(param_en_pub_tmc_info_)
  {
    tmc_info_pub_ = p_nh_->advertise<adi_tmcl::TmcInfo>(param_tmc_info_topic_, 100);
    seq_ctr_ = 0;
    ros::Rate period(param_pub_rate_tmc_info_);
    timer_callback_ = p_nh_->createTimer(ros::Duration(period), &Motor::rosPublishTmcInfo, this);
    
    if(timer_callback_.isValid())
    {
      ROS_INFO_STREAM("[" << __func__  << "] Publisher TimerCallback is created successfully!");
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Publisher TimerCallback not created!");
    }
  }
}

/* Publisher Callback */
void Motor::rosPublishTmcInfo(const ros::TimerEvent& event)
{
  int32_t val = 0;

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

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "SupplyVoltage", motor_number_, &val))
  {
    tmc_info_msg_.board_voltage = val / 10; //converts mV to V
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get SupplyVoltage");
  }

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "StatusFlags", motor_number_, &val))
  {
    tmc_info_msg_.status_flag = val;
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get StatusFlags");
  }

  /* Velocity, Position, Torque */
  if(param_pub_actual_vel_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualVelocity", motor_number_, &val))
    {
      tmc_info_msg_.velocity = val * param_add_ratio_vel_;
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
      tmc_info_msg_.position = val * param_add_ratio_pos_;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualPosition");
    }
  }

  if(param_pub_actual_trq_)
  {
    if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, "ActualTorque", motor_number_, &val))
    {
      tmc_info_msg_.torque = val * param_add_ratio_trq_;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualTorque");
    }
  }

  tmc_info_pub_.publish(tmc_info_msg_);
  seq_ctr_++;
}

/* Initialize Subscriber */
void Motor::initSubscriber()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  tmc_cmd_vel_sub_ = p_nh_->subscribe(param_tmc_cmd_vel_topic_, 1000, &Motor::cmdVelCallback, this);
  tmc_cmd_abspos_sub_ = p_nh_->subscribe(param_tmc_cmd_abspos_topic_, 1000, &Motor::cmdAbsPosCallback, this);
  tmc_cmd_relpos_sub_ = p_nh_->subscribe(param_tmc_cmd_relpos_topic_, 1000, &Motor::cmdRelPosCallback, this);
  tmc_cmd_trq_sub_ = p_nh_->subscribe(param_tmc_cmd_trq_topic_, 1000, &Motor::cmdTrqCallback, this);
}

/* Subscriber Callback */
void Motor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  int32_t board_val = 0;

  board_val = val / param_add_ratio_vel_;

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
void Motor::cmdAbsPosCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;

  val /= param_add_ratio_pos_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
    << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, 0, motor_number_, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Absolute Position");
  }
}
void Motor::cmdRelPosCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;

  val /= param_add_ratio_pos_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
    << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MVP, 1, motor_number_, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Relative Position");
  }
}
void Motor::cmdTrqCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;

  val /=  param_add_ratio_trq_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val);

  if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, "TargetTorque", motor_number_, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Torque");
  }
}
