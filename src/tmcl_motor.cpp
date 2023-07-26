/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_motor.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
Motor::Motor(ros::NodeHandle *p_nh, TmclInterpreter* p_tmcl_int, uint8_t motor_num)  :  
p_nh_ (p_nh), 
p_tmcl_int(p_tmcl_int),
motor_num(motor_num)
{
  ROS_DEBUG_STREAM("[Motor::" <<  __func__ << "] called");
  initParams();
}

/* Destructor */
Motor::~Motor()
{
  ROS_DEBUG_STREAM("[Motor::" <<  __func__ << "] called");
  p_tmcl_int = nullptr;
  p_nh_ = nullptr;
}

void Motor::init()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  initPublisher();
  initSubscriber();
  ROS_INFO("[%s] Motor %d Initialized!\n", __func__, motor_num);
}

/* Initialize Parameters */
void Motor::initParams()
{

  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  s_node_name = ros::this_node::getName();

  const std::string s_en_pub_tmc_info = s_node_name + "/motor"+ std::to_string(motor_num) + "/en_pub_tmc_info";
  if(!p_nh_->getParam(s_en_pub_tmc_info, param_en_pub_tmc_info))
  {
    param_en_pub_tmc_info = false;
    p_nh_->setParam(s_en_pub_tmc_info, param_en_pub_tmc_info);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get en_pub_tmc_info for motor"<< std::to_string(motor_num) <<", setting to default value: " << std::boolalpha << param_en_pub_tmc_info);
  }
  const std::string s_pub_actual_vel = s_node_name + "/motor"+ std::to_string(motor_num) + "/pub_actual_vel";
  if(!p_nh_->getParam(s_pub_actual_vel, param_pub_actual_vel))
  {
    param_pub_actual_vel = false;
    p_nh_->setParam(s_pub_actual_vel, param_pub_actual_vel);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_vel for motor"<< std::to_string(motor_num) <<", setting to default value: " << std::boolalpha << param_pub_actual_vel);
  }
  const std::string s_pub_actual_pos = s_node_name + "/motor"+ std::to_string(motor_num) + "/pub_actual_pos";
  if(!p_nh_->getParam(s_pub_actual_pos, param_pub_actual_pos))
  {
    param_pub_actual_pos = false;
    p_nh_->setParam(s_pub_actual_pos, param_pub_actual_pos);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_pos for motor"<< std::to_string(motor_num) <<", setting to default value: " << std::boolalpha << param_pub_actual_pos);
  }
  const std::string s_pub_actual_trq = s_node_name + "/motor"+ std::to_string(motor_num) + "/pub_actual_trq";
  if(!p_nh_->getParam(s_pub_actual_trq, param_pub_actual_trq))
  {
    param_pub_actual_trq = false;
    p_nh_->setParam(s_pub_actual_trq, param_pub_actual_trq);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_trq for motor"<< std::to_string(motor_num) <<", setting to default value: " << std::boolalpha << param_pub_actual_trq);
  }
  const std::string s_tmc_info_topic = s_node_name + "/motor"+ std::to_string(motor_num) + "/tmc_info_topic";
  if(!p_nh_->getParam(s_tmc_info_topic, param_tmc_info_topic))
  {
    param_tmc_info_topic = "/tmc_info_" + std::to_string(motor_num);
    p_nh_->setParam(s_tmc_info_topic, param_tmc_info_topic);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_info_topic for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_tmc_info_topic);
  }
  const std::string s_tmc_cmd_vel_topic = s_node_name + "/motor"+ std::to_string(motor_num) + "/tmc_cmd_vel_topic";
  if(!p_nh_->getParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic))
  {
    param_tmc_cmd_vel_topic = "/cmd_vel_" + std::to_string(motor_num);
    p_nh_->setParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_vel_topic for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_tmc_cmd_vel_topic);
  }
  const std::string s_tmc_cmd_abspos_topic = s_node_name + "/motor"+ std::to_string(motor_num) + "/tmc_cmd_abspos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic))
  {
    param_tmc_cmd_abspos_topic = "/cmd_abspos_" + std::to_string(motor_num);
    p_nh_->setParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_abspos_topic for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_tmc_cmd_abspos_topic);
  }
  const std::string s_tmc_cmd_relpos_topic = s_node_name + "/motor"+ std::to_string(motor_num) + "/tmc_cmd_relpos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic))
  {
    param_tmc_cmd_relpos_topic = "/cmd_relpos_" + std::to_string(motor_num);
    p_nh_->setParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_relpos_topic for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_tmc_cmd_relpos_topic);
  }
  const std::string s_tmc_cmd_trq_topic = s_node_name + "/motor"+ std::to_string(motor_num) + "/tmc_cmd_trq_topic";
  if(!p_nh_->getParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic))
  {
    param_tmc_cmd_trq_topic = "/cmd_trq_" + std::to_string(motor_num);
    p_nh_->setParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_trq_topic for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_tmc_cmd_trq_topic);
  }
  const std::string s_wheel_diameter = s_node_name + "/motor"+ std::to_string(motor_num) + "/wheel_diameter";
  if(!p_nh_->getParam(s_wheel_diameter, param_wheel_diameter))
  {
    param_wheel_diameter = 0;
    p_nh_->setParam(s_wheel_diameter, param_wheel_diameter);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get wheel_diameter for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_wheel_diameter);
  }
  const std::string s_additional_ratio_vel = s_node_name + "/motor"+ std::to_string(motor_num) + "/additional_ratio_vel";
  if(!p_nh_->getParam(s_additional_ratio_vel, param_add_ratio_vel))
  {
    param_add_ratio_vel = 1;
    p_nh_->setParam(s_additional_ratio_vel, param_add_ratio_vel);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_vel for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_add_ratio_vel);
  }
  const std::string s_additional_ratio_pos = s_node_name + "/motor"+ std::to_string(motor_num) + "/additional_ratio_pos";
  if(!p_nh_->getParam(s_additional_ratio_pos, param_add_ratio_pos))
  {
    param_add_ratio_pos = 1;
    p_nh_->setParam(s_additional_ratio_pos, param_add_ratio_pos);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_pos for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_add_ratio_pos);
  }
  const std::string s_additional_ratio_trq = s_node_name + "/motor"+ std::to_string(motor_num) + "/additional_ratio_trq";
  if(!p_nh_->getParam(s_additional_ratio_trq, param_add_ratio_trq))
  {
    param_add_ratio_trq = 1;
    p_nh_->setParam(s_additional_ratio_trq, param_add_ratio_trq);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_trq for motor"<< std::to_string(motor_num) <<", setting to default value: " << param_add_ratio_trq);
  }

  /* Parameters that are already validated in tmcl_ros.cpp. Only need to get the param value */
  const std::string s_pub_rate_tmc_info= s_node_name + "/pub_rate_tmc_info";
  (void)p_nh_->getParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info);
  const std::string s_comm_interface_name= s_node_name + "/comm_interface_name";
  (void)p_nh_->getParam(s_comm_interface_name, param_comm_interface_name);
}

/* Initialize Publisher */
void Motor::initPublisher()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  if(param_en_pub_tmc_info)
  {
    tmc_info_pub = p_nh_->advertise<tmcl_ros::TmcInfo>(param_tmc_info_topic, 1);
    seq_ctr = 0;
    ros::Rate period(param_pub_rate_tmc_info);
    timer_callback = p_nh_->createTimer(ros::Duration(period), &Motor::rosPublishTmcInfo, this);
    
    if(timer_callback.isValid())
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

  if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "StatusFlags", motor_num, &val))
  {
    tmc_info_msg.status_flag = val;
  }
  else
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get StatusFlags");
  }

  /* Velocity, Position, Torque */
  if(param_pub_actual_vel)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "ActualVelocity", motor_num, &val))
    {
      tmc_info_msg.velocity = val * param_add_ratio_vel;
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
      tmc_info_msg.position = val * param_add_ratio_pos;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualPosition");
    }
  }

  if(param_pub_actual_trq)
  {
    if(p_tmcl_int->executeCmd(TMCL_CMD_GAP, "ActualTorque", motor_num, &val))
    {
      tmc_info_msg.torque = val * param_add_ratio_trq;
    }
    else
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Failed to get ActualTorque");
    }
  }

  tmc_info_pub.publish(tmc_info_msg);
  seq_ctr++;
}

/* Initialize Subscriber */
void Motor::initSubscriber()
{
  ROS_INFO_STREAM("[Motor::" << __func__ << "] called");

  tmc_cmd_vel_sub = p_nh_->subscribe(param_tmc_cmd_vel_topic, 1000, &Motor::cmdVelCallback, this);
  tmc_cmd_abspos_sub = p_nh_->subscribe(param_tmc_cmd_abspos_topic, 1000, &Motor::cmdAbsPosCallback, this);
  tmc_cmd_relpos_sub = p_nh_->subscribe(param_tmc_cmd_relpos_topic, 1000, &Motor::cmdRelPosCallback, this);
  tmc_cmd_trq_sub = p_nh_->subscribe(param_tmc_cmd_trq_topic, 1000, &Motor::cmdTrqCallback, this);
}

/* Subscriber Callback */
void Motor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  int32_t board_val = 0;

  board_val = val / param_add_ratio_vel;

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
void Motor::cmdAbsPosCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;

  val /= param_add_ratio_pos;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: " << val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_MVP, 0, motor_num, &val))
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

  val /= param_add_ratio_pos;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: " << val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_MVP, 1, motor_num, &val))
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

  val /=  param_add_ratio_trq;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val);

  if(p_tmcl_int->executeCmd(TMCL_CMD_SAP, "TargetTorque", motor_num, &val))
  {
    ROS_DEBUG_STREAM("\n[" << __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to set Torque");
  }
}
