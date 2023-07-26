/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_STEPPER_MOTOR_H
#define TMCL_STEPPER_MOTOR_H

#include "tmcl_motor.h"

/* List Commutation Modes availaable for BLDC motors */ 
typedef enum
{
  STEPPER_OPENLOOP_MOTOR = 0,
  STEPPER_CLOSEDLOOP_MOTOR,
  STEPPER_COMM_MODE_MAX            /* This should not be used */
} stepper_comm_mode_t;

class StepperMotor : public Motor
{
  public:
    /* Constructor */
    StepperMotor(ros::NodeHandle* p_nh, TmclInterpreter *p_tmcl_int, uint8_t motor_num);

    /* Destructor */
    ~StepperMotor() override;

    void init() override;

  private:
    /* Publisher */
    void rosPublishTmcInfo(const ros::TimerEvent& event) override;

    /* Subscriber */
    void initSubscriber() override;
    void cmdVelCallback(const geometry_msgs::Twist& msg) override;
    void cmdAbsPosCallback(const std_msgs::Int32 msg) override;
    void cmdRelPosCallback(const std_msgs::Int32 msg) override;
    void cmdTrqCallback(const std_msgs::Int32 msg) override;

    uint16_t microstep_resolution;
    uint16_t motor_fullstep_resolution;

    stepper_comm_mode_t comm_mode;
};

#endif // TMCL_STEPPER_MOTOR_H
