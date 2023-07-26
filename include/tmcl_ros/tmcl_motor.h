/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_MOTOR_H
#define TMCL_MOTOR_H

#include "tmcl_interpreter.h"
#include <ros/ros.h>
#include <ros/console.h>
#include "tmcl_ros/TmcInfo.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

/* Conversion constants */

/* Derived from converting linear velocity (ROS velocity unit) to rpm (TMC board velocity unit)*/
#define PI 3.1415926535
#define SECS_TO_MIN 60

/* Used for converting degrees (general position/angular unit) to steps (TMC board position/angular unit)*/
#define ANGULAR_FULL_ROTATION 360

class Motor
{
  public:
    /* Constructor */
    Motor(ros::NodeHandle* p_nh, TmclInterpreter *p_tmcl_int, uint8_t motor_num);

    /* Destructor */
    virtual ~Motor();

    virtual void init();

  private:
    void initParams();
        
  protected:
    /* Publisher */
    void initPublisher();
    virtual void rosPublishTmcInfo(const ros::TimerEvent& event);
    ros::Timer timer_callback;
    ros::Publisher tmc_info_pub;
    tmcl_ros::TmcInfo tmc_info_msg;
    float param_pub_rate_tmc_info;
    std::string param_tmc_info_topic;
    bool param_en_pub_tmc_info;
    uint32_t seq_ctr;

    /* Subscriber */
    virtual void initSubscriber();
    ros::Subscriber tmc_cmd_vel_sub;
    ros::Subscriber tmc_cmd_abspos_sub;
    ros::Subscriber tmc_cmd_relpos_sub;
    ros::Subscriber tmc_cmd_trq_sub;
    virtual void cmdVelCallback(const geometry_msgs::Twist& msg);
    virtual void cmdAbsPosCallback(const std_msgs::Int32 msg);
    virtual void cmdRelPosCallback(const std_msgs::Int32 msg);
    virtual void cmdTrqCallback(const std_msgs::Int32 msg);

    /* Pointers */
    ros::NodeHandle* p_nh_;
    TmclInterpreter* p_tmcl_int;

    /* Other Variables */
    std::string param_comm_interface_name;
    std::string s_node_name;
    uint8_t motor_num;

    /* Motor Specific Settings (Ext YAML) */
    std::string param_tmc_cmd_vel_topic;
    std::string param_tmc_cmd_abspos_topic;
    std::string param_tmc_cmd_relpos_topic;
    std::string param_tmc_cmd_trq_topic;
    bool param_pub_actual_vel;
    bool param_pub_actual_pos;
    bool param_pub_actual_trq;
    float param_wheel_diameter;
    float param_add_ratio_vel;
    float param_add_ratio_pos;
    float param_add_ratio_trq;
};

#endif // TMCL_MOTOR_H
