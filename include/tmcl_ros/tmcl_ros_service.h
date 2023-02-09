/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_ROS_SERVICE_H 
#define TMCL_ROS_SERVICE_H

#include "tmcl_ros.h"
#include "tmcl_ros/TmcSrv.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

class TmclRosService 
{
  public:
    std::string s_service_name;

    ros::ServiceServer service;
    
    /* Constructor */    
    TmclRosService();

    /* Constructor */    
    TmclRosService(ros::NodeHandle nh, TmclROS *tmc_obj);

    /* Destructor */    
    ~TmclRosService();
};
  
TmclROS *tmc_obj_srv;

#endif
