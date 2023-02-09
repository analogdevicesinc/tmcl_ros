/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_ros_subscriber.h"

void cmd_vel_0_callback(const geometry_msgs::Twist& msg);
void cmd_vel_1_callback(const geometry_msgs::Twist& msg);
void cmd_vel_2_callback(const geometry_msgs::Twist& msg);

void cmd_abspos_0_callback(const std_msgs::Int32 msg);
void cmd_abspos_1_callback(const std_msgs::Int32 msg);
void cmd_abspos_2_callback(const std_msgs::Int32 msg);

void cmd_relpos_0_callback(const std_msgs::Int32 msg);
void cmd_relpos_1_callback(const std_msgs::Int32 msg);
void cmd_relpos_2_callback(const std_msgs::Int32 msg);

void cmd_trq_0_callback(const std_msgs::Int32 msg);
void cmd_trq_1_callback(const std_msgs::Int32 msg);
void cmd_trq_2_callback(const std_msgs::Int32 msg);

/***
 * Constructor
 ***/
TmclRosSubscriber::TmclRosSubscriber()
{

}

/***
 * Constructor
 ***/
TmclRosSubscriber::TmclRosSubscriber(ros::NodeHandle nh, TmclROS *tmc_obj)
{

  for(int i = 0; i < tmc_obj->param_total_motors; i++)
  {
    if(tmc_obj->param_en_motors[i])
    {
        if(i == 0)
        {
            if(tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 0)
            {
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (0). Subscribing from velocity and torque is disabled");
            }
            else if (tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 1)
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_0_callback);
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (1). Subscribing from torque is disabled, open loop current will be used");
            }
            else
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_0_callback);
                tmc_cmd_trq_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_trq_topic[i], 1000, cmd_trq_0_callback);
            }

            tmc_cmd_abspos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_abspos_topic[i], 1000, cmd_abspos_0_callback);
            tmc_cmd_relpos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_relpos_topic[i], 1000, cmd_relpos_0_callback);
        }
        else if(i == 1)
        {
            if(tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 0)
            {
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (0). Subscribing from velocity and torque is disabled");
            }
            else if (tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 1)
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_1_callback);
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (1). Subscribing from torque is disabled, open loop current will be used");

            }
            else
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_1_callback);
                tmc_cmd_trq_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_trq_topic[i], 1000, cmd_trq_1_callback);
            }
            
            tmc_cmd_abspos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_abspos_topic[i], 1000, cmd_abspos_1_callback);
            tmc_cmd_relpos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_relpos_topic[i], 1000, cmd_relpos_1_callback);
        }
        else if(i == 2)
        {
            if(tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 0)
            {
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (0). Subscribing from velocity and torque is disabled");
            }
            else if (tmc_obj->ap_cfg_ext[ROW_IDX_CommutationMode][i] == 1)
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_2_callback);
                ROS_WARN_STREAM("Motor" << i << " CommutationMode is set to (1). Subscribing from torque is disabled, open loop current will be used");
            }
            else
            {
                tmc_cmd_vel_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_vel_topic[i], 1000, cmd_vel_2_callback);
                tmc_cmd_trq_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_trq_topic[i], 1000, cmd_trq_2_callback);
            }

            tmc_cmd_abspos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_abspos_topic[i], 1000, cmd_abspos_2_callback);
            tmc_cmd_relpos_sub[i] = nh.subscribe(tmc_obj->param_tmc_cmd_relpos_topic[i], 1000, cmd_relpos_2_callback);
        }
    }
  }

    tmc_obj_sub = tmc_obj;
}

/***
 * Destructor
 ***/
TmclRosSubscriber::~TmclRosSubscriber()
{

}

/**
 * Callback function for cmd_vel subscriber for Motor 0
 */
void cmd_vel_0_callback(const geometry_msgs::Twist& msg)
{
    float val = msg.linear.x;
    int32_t rpm_val = 0;

    //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
    if(tmc_obj_sub->param_wheel_diameter == 0)
    {
        rpm_val = val;
    }
    else
    {
        //Formula to convert linear value to rpm (unit that the board accepts)
        rpm_val = (val * SECS_TO_MIN) / (PI * tmc_obj_sub->param_wheel_diameter);
    }

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << rpm_val);

    if (rpm_val < MIN_VEL || rpm_val > MAX_VEL)
    {
        ROS_WARN_STREAM("Input value for motor0 velocity is out of range, will not execute");
    }
    else
    {   
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetVelocity][COL_IDX_AP], TMCL_MOTOR_0, &rpm_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/**
 * Callback function for cmd_vel subscriber for Motor 1
 */
void cmd_vel_1_callback(const geometry_msgs::Twist& msg)
{
    float val = msg.linear.x;
    int32_t rpm_val = 0;

    //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
    if(tmc_obj_sub->param_wheel_diameter == 0)
    {
        rpm_val = val;
    }
    else
    {
        //Formula to convert linear value to rpm (unit that the board accepts)
        rpm_val = (val * SECS_TO_MIN) / (PI * tmc_obj_sub->param_wheel_diameter);
    }

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << rpm_val);

    if (rpm_val < MIN_VEL || rpm_val > MAX_VEL)
    {
        ROS_WARN_STREAM("Input value for motor1 velocity is out of range, will not execute");
    }
    else
    {   
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetVelocity][COL_IDX_AP], TMCL_MOTOR_1, &rpm_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
    
}

/**
 * Callback function for cmd_vel subscriber for Motor 2
 */
void cmd_vel_2_callback(const geometry_msgs::Twist& msg)
{
    float val = msg.linear.x;
    int32_t rpm_val = 0;

    //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
    if(tmc_obj_sub->param_wheel_diameter == 0)
    {
        rpm_val = val;
    }
    else
    {
        //Formula to convert linear value to rpm (unit that the board accepts)
        rpm_val = (val * SECS_TO_MIN) / (PI * tmc_obj_sub->param_wheel_diameter);
    }

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << rpm_val);

    if (rpm_val < MIN_VEL || rpm_val > MAX_VEL)
    {
        ROS_WARN_STREAM("Input value for motor2 velocity is out of range, will not execute");
    }
    else
    {   
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetVelocity][COL_IDX_AP], TMCL_MOTOR_2, &rpm_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/**
 * Callback function for cmd_pos subscriber for Motor 0
 */
void cmd_abspos_0_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0.00;
    int32_t unit_val = 0;
    int32_t val = msg.data;

    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_0] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    if (unit_val < MIN_POS || unit_val > MAX_POS)
    {
        ROS_WARN_STREAM("Input value for motor0 position is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_0, &unit_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/*
 * Callback function for cmd_pos subscriber for Motor 1
 */
void cmd_abspos_1_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0.00;
    int32_t unit_val = 0;
    int32_t val = msg.data;
    
    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_1] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    if (unit_val < MIN_POS || unit_val > MAX_POS)
    {
        ROS_WARN_STREAM("Input value for motor1 position is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_1, &unit_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/*
 * Callback function for cmd_pos subscriber for Motor 2
 */
void cmd_abspos_2_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0.00;
    int32_t unit_val = 0;
    int32_t val = msg.data;
    
    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_2] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    if (unit_val < MIN_POS || unit_val > MAX_POS)
    {
        ROS_WARN_STREAM("Input value for motor2 position is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_2, &unit_val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/**
 * Callback function for cmd_pos subscriber for Motor 0
 */
void cmd_relpos_0_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0;
    int32_t unit_val = 0;
    int32_t val = msg.data;
    int32_t abspos_val = 0;

    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_0] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    // Get actual position then add the converted input with abspos (increment).
    if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_GAP, tmc_obj_sub->ap_cfg[ROW_IDX_ActualPosition][COL_IDX_AP], TMCL_MOTOR_0, &abspos_val)) 
    {
        unit_val = abspos_val + unit_val;
        if (unit_val < MIN_POS || unit_val > MAX_POS)
        {
            ROS_WARN_STREAM("Motor0 position is out of range, will not execute");
        }
        else
        {
            if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_0, &unit_val))
            {
                ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
            }
        }
    }
}

/**
 * Callback function for cmd_pos subscriber for Motor 1
 */
void cmd_relpos_1_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0;
    int32_t unit_val = 0;
    int32_t val = msg.data;
    int32_t abspos_val = 0;

    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_1] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    // Get actual position then add the converted input with abspos (increment).
    if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_GAP, tmc_obj_sub->ap_cfg[ROW_IDX_ActualPosition][COL_IDX_AP], TMCL_MOTOR_1, &abspos_val)) 
    {
        unit_val = abspos_val + unit_val;
        if (unit_val < MIN_POS || unit_val > MAX_POS)
        {
            ROS_WARN_STREAM("Motor1 position is out of range, will not execute");
        }
        else
        {
            if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_1, &unit_val))
            {
                ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
            }
        }
    }
}

/**
 * Callback function for cmd_pos subscriber for Motor 2
 */
void cmd_relpos_2_callback(const std_msgs::Int32 msg)
{
    float convert_const_deg = 0;
    int32_t unit_val = 0;
    int32_t val = msg.data;
    int32_t abspos_val = 0;

    //convert input(degrees) to unit
    convert_const_deg = tmc_obj_sub->ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_2] / (float) ANGULAR_FULL_ROTATION;
    unit_val = val * convert_const_deg;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val << " board value: " << unit_val);

    // Get actual position then add the converted input with abspos (increment).
    if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_GAP, tmc_obj_sub->ap_cfg[ROW_IDX_ActualPosition][COL_IDX_AP], TMCL_MOTOR_2, &abspos_val)) 
    {
        unit_val = abspos_val + unit_val;
        if (unit_val < MIN_POS || unit_val > MAX_POS)
        {
            ROS_WARN_STREAM("Motor2 position is out of range, will not execute");
        }
        else
        {
            if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetPosition][COL_IDX_AP], TMCL_MOTOR_2, &unit_val))
            {
                ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
            }
        }
    }
}

/**
 * Callback function for cmd_trq subscriber for Motor 0
 */
void cmd_trq_0_callback(const std_msgs::Int32 msg)
{
    int32_t val = msg.data;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val);

    if (val < MIN_TRQ || val > MAX_TRQ)
    {
        ROS_WARN_STREAM("Input value for motor0 torque is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetTorque][COL_IDX_AP], TMCL_MOTOR_0, &val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    }
}

/*
 * Callback function for cmd_trq subscriber for Motor 1
 */
void cmd_trq_1_callback(const std_msgs::Int32 msg)
{
    int32_t val = msg.data;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val);

    if (val < MIN_TRQ || val > MAX_TRQ)
    {
        ROS_WARN_STREAM("Input value for motor1 torque is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetTorque][COL_IDX_AP], TMCL_MOTOR_1, &val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }
    
    }
}

/*
 * Callback function for cmd_trq subscriber for Motor 2
 */
void cmd_trq_2_callback(const std_msgs::Int32 msg)
{
    int32_t val = msg.data;

    ROS_DEBUG_STREAM("Subscriber callback " << __func__  << " entered, received: " << val);
    
    if (val < MIN_TRQ || val > MAX_TRQ)
    {
        ROS_WARN_STREAM("Input value for motor2 torque is out of range, will not execute");
    }
    else
    {
        if(tmc_obj_sub->exec_tmcl_cmd(TMCL_CMD_SAP, tmc_obj_sub->ap_cfg[ROW_IDX_TargetTorque][COL_IDX_AP], TMCL_MOTOR_2, &val))
        {
            ROS_DEBUG_STREAM("\nSubscriber callback " << __func__  << " exited successfully");
        }  
    }
}
