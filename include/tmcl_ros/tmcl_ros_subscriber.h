/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_ROS_SUBSCRIBER_H
#define TMCL_ROS_SUBSCRIBER_H

#include "tmcl_ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

class TmclRosSubscriber
{
  public:
    /* Subscriber for set Velocity command */
    ros::Subscriber tmc_cmd_vel_sub[TMCL_MOTOR_MAX];

    /* Subscriber for set Absolute Position command */
    ros::Subscriber tmc_cmd_abspos_sub[TMCL_MOTOR_MAX];

    /* Subscriber for set Relative Position command */
    ros::Subscriber tmc_cmd_relpos_sub[TMCL_MOTOR_MAX];

    /* Subscriber for set Torque command */
    ros::Subscriber tmc_cmd_trq_sub[TMCL_MOTOR_MAX];

    /* Constructor */
    TmclRosSubscriber();

    /* Constructor */
    TmclRosSubscriber(ros::NodeHandle nh, TmclROS *tmc_obj);

    /* Destructor */
    ~TmclRosSubscriber();
};

TmclROS *tmc_obj_sub;

#endif
