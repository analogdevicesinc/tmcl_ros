/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_ROS_PUBLISHER_H 
#define TMCL_ROS_PUBLISHER_H

#include "tmcl_ros.h"
#include "std_msgs/String.h"
#include "tmcl_ros/TmcInfo.h"

class TmclRosPublisher 
{
  public:
    /* Publisher for TMC info */
    ros::Publisher tmc_info_pub[TMCL_MOTOR_MAX];
    uint32_t seq_ctr[TMCL_MOTOR_MAX];

    /* TMC info message */
    tmcl_ros::TmcInfo tmc_info_msg;

    /* Constructor */    
    TmclRosPublisher();

    /* Constructor */    
    TmclRosPublisher(ros::NodeHandle nh, TmclROS *tmc_obj);

    /* Destructor */    
    ~TmclRosPublisher();

    /* Publish TMC info */
    bool rosPublishTmcInfo(TmclROS *tmc_obj);
};

#endif
