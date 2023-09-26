/**
 * Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_BLDC_MOTOR_H
#define TMCL_BLDC_MOTOR_H

#include "tmcl_motor.h"

/* List Commutation Modes availaable for BLDC motors */ 
  typedef enum
  {
    BLDC_DISABLED_MOTOR = 0,
    BLDC_OPENLOOP_MOTOR,
    BLDC_CLOSEDLOOP_MOTOR,
    BLDC_COMM_MODE_MAX            /* This should not be used */
  } bldc_comm_mode_t;

class BLDCMotor : public Motor
{
public:
  /* Constructor */
  BLDCMotor(ros::NodeHandle* p_nh, TmclInterpreter *p_tmcl_interpreter, 
    uint16_t module_number, uint8_t motor_number);

  /* Destructor */
  ~BLDCMotor() override;

  /* Initialize BLDC Motor */
  void init() override;

private:
  /* Publisher */
  void rosPublishTmcInfo(const ros::TimerEvent& event) override;

  /* Subscriber */
  void initSubscriber() override;
  void cmdVelCallback(const geometry_msgs::Twist& msg) override;
  void cmdAbsPosCallback(const std_msgs::Int32 msg) override;
  void cmdRelPosCallback(const std_msgs::Int32 msg) override;

  int32_t position_scaler_;
  int32_t encoder_steps_;
  bool b_statusflags_register_available_;
  std::vector<std::string> param_statusflags_reg_name_;
  std::vector<int> param_statusflags_reg_shift_;

  bldc_comm_mode_t comm_mode_;
};

#endif // TMCL_BLDC_MOTOR_H
