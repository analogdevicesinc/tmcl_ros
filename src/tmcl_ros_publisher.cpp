/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_ros_publisher.h"

/***
 * Constructor
 ***/
TmclRosPublisher::TmclRosPublisher()
{

}

/***
 * Constructor
 ***/
TmclRosPublisher::TmclRosPublisher(ros::NodeHandle nh, TmclROS *tmc_obj)
{
    for(int i = 0; i < tmc_obj->param_total_motors; i++)
    {
        if(tmc_obj->param_en_motors[i] && tmc_obj->param_en_pub_tmc_info[i])
		{
			
			tmc_info_pub[i] = nh.advertise<tmcl_ros::TmcInfo>(tmc_obj->param_tmc_info_topic[i], 1);
			seq_ctr[i] = 0;
		}
    }
}

/***
 * Destructor
 ***/
TmclRosPublisher::~TmclRosPublisher()
{

}

/***
 * Publish camera_info ROS message
 ***/
bool TmclRosPublisher::rosPublishTmcInfo(TmclROS *tmc_obj)
{
    bool result = false;

    for(int i = 0; i < tmc_obj->param_total_motors; i++)
    {
	if(tmc_obj->param_en_motors[i] && tmc_obj->param_en_pub_tmc_info[i])
	{
            /* Step 1: Fill up tmc_info ROS message */
	    tmc_info_msg.header.stamp = ros::Time::now();
	    tmc_info_msg.header.seq = seq_ctr[i];
	    tmc_info_msg.header.frame_id = "tmcl_frame";
	    tmc_info_msg.interface_name = tmc_obj->param_comm_interface_name;
	    tmc_info_msg.motor_num = i;

		tmc_info_msg.board_voltage = tmc_obj->ap_cfg_ext[ROW_IDX_SupplyVoltage][i];

		tmc_info_msg.board_voltage = tmc_info_msg.board_voltage / 10; //converts mV to V

		if(tmc_obj->param_pub_actual_vel[i])
		{
                        tmc_info_msg.velocity = tmc_obj->ap_cfg_ext[ROW_IDX_ActualVelocity][i];
			
			//converts rpm to linear velocity
			if(tmc_obj->param_wheel_diameter > 0)
			{
				tmc_info_msg.velocity = (tmc_info_msg.velocity * PI * tmc_obj->param_wheel_diameter) / 60;
			}
		}
	    else
	    {
			tmc_info_msg.velocity = 0.0;
	    }

		if(tmc_obj->param_pub_actual_pos[i])
		{
                        tmc_info_msg.position = tmc_obj->ap_cfg_ext[ROW_IDX_ActualPosition][i];

			//converts units to degrees
			tmc_info_msg.position = tmc_info_msg.position / (tmc_obj->ap_cfg_ext[ROW_IDX_EncoderSteps][i] / ANGULAR_FULL_ROTATION);
		}
	    else
	    {
			tmc_info_msg.position = 0.0;
	    }

		if(tmc_obj->param_pub_actual_trq[i])
		{
                        tmc_info_msg.torque = tmc_obj->ap_cfg_ext[ROW_IDX_ActualTorque][i];
	    }
	    else
	    {
			tmc_info_msg.torque = 0.0;
	    }

            /* Step 2: Publish tmc_info ROS message */
            tmc_info_pub[i].publish(tmc_info_msg);
	    seq_ctr[i]++;
	}
    }

    result = true;

    return result;
}
