/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_MOTOR_H
#define TMCL_MOTOR_H

#include "tmcl_interpreter.h"
#include <ros/ros.h>
#include <ros/console.h>
#include "adi_tmcl/TmcInfo.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

/* Conversion constants */

/* Derived from converting linear velocity (ROS velocity unit) to rpm (TMC board velocity unit) */
const double PI = 3.1415926535;
const uint8_t SECS_TO_MIN = 60;

/* Used for converting degrees (general position/angular unit) to steps (TMC board position/angular unit) */
const uint16_t ANGULAR_FULL_ROTATION = 360;

class Motor
{
public:
  /* Constructor */
  Motor(ros::NodeHandle* p_nh, TmclInterpreter *p_tmcl_interpreter, 
    uint16_t module_number, uint8_t motor_number);

  /* Destructor */
  virtual ~Motor();

  /* Initialize Motor */
  virtual void init();
      
protected:
  /* Publisher */
  void initPublisher();
  virtual void rosPublishTmcInfo(const ros::TimerEvent& event);
  ros::Timer timer_callback_;
  ros::Publisher tmc_info_pub_;
  adi_tmcl::TmcInfo tmc_info_msg_;
  float param_pub_rate_tmc_info_;
  std::string param_tmc_info_topic_;
  bool param_en_pub_tmc_info_;
  uint32_t seq_ctr_;

  /* Subscriber */
  virtual void initSubscriber();
  virtual void cmdVelCallback(const geometry_msgs::Twist& msg);
  virtual void cmdAbsPosCallback(const std_msgs::Int32 msg);
  virtual void cmdRelPosCallback(const std_msgs::Int32 msg);
  virtual void cmdTrqCallback(const std_msgs::Int32 msg);
  ros::Subscriber tmc_cmd_vel_sub_;
  ros::Subscriber tmc_cmd_abspos_sub_;
  ros::Subscriber tmc_cmd_relpos_sub_;
  ros::Subscriber tmc_cmd_trq_sub_;

  /* Pointers */
  ros::NodeHandle* p_nh_;
  TmclInterpreter* p_tmcl_interpreter_;

  /* Other Variables */
  std::string param_comm_interface_name_;
  std::string s_node_name_;
  std::string s_namespace_;
  std::string frame_id_;
  uint16_t module_number_;
  uint8_t motor_number_;

  /* Motor Specific Settings (Ext YAML) */
  std::string param_tmc_cmd_vel_topic_;
  std::string param_tmc_cmd_abspos_topic_;
  std::string param_tmc_cmd_relpos_topic_;
  std::string param_tmc_cmd_trq_topic_;
  bool param_pub_actual_vel_;
  bool param_pub_actual_pos_;
  bool param_pub_actual_trq_;
  float param_wheel_diameter_;
  float param_add_ratio_vel_;
  float param_add_ratio_pos_;
  float param_add_ratio_trq_;

private:
  /* Initialize Parameter for Motor Specific */
  void initParams();
};

#endif // TMCL_MOTOR_H
