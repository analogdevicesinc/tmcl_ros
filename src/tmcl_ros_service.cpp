/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_ros_service.h"

bool tmclInstruction(tmcl_ros::TmcSrv::Request& req, tmcl_ros::TmcSrv::Response& res);

/***
 * Constructor
 ***/
TmclRosService::TmclRosService()
{

}

/***
 * Constructor
 ***/
TmclRosService::TmclRosService(ros::NodeHandle nh, TmclROS *tmc_obj)
{
	std::string s_node_name = ros::this_node::getNamespace();
	s_service_name = s_node_name + "/tmcl_custom_cmd";

    service = nh.advertiseService(s_service_name, tmclInstruction);
	tmc_obj_srv = tmc_obj;
}

/***
 * Destructor
 ***/
TmclRosService::~TmclRosService()
{

}

/***
 * Service callback
 ***/
bool tmclInstruction(tmcl_ros::TmcSrv::Request& req, tmcl_ros::TmcSrv::Response& res)
{
	bool b_result = false;
	int32_t val = 0;

	if (req.instruction == "SAP")
	{
		val = req.value;
		if(tmc_obj_srv->exec_tmcl_cmd(TMCL_CMD_SAP, req.instruction_type, (tmcl_motor_t) req.motor_num, &val))
		{
			b_result = true;
			res.output = req.value;
			ROS_DEBUG_STREAM("Service Done. Set value: " << res.output);
		}
		else
		{
			ROS_ERROR_STREAM("Service Failed!");
		}

		res.result = b_result;
	}
	
	else if (req.instruction == "GAP")
	{
		if (tmc_obj_srv->exec_tmcl_cmd(TMCL_CMD_GAP, req.instruction_type, (tmcl_motor_t) req.motor_num, &val))
		{
			b_result = true;
			res.output = val;
			ROS_DEBUG_STREAM("Service Done. Get value: " << res.output);
		}
		else
		{
			ROS_ERROR_STREAM("Service Failed!");
		}

		res.result = b_result;
	}

	else if (req.instruction == "SGP")
	{
		val = req.value;

		if(req.instruction_type == TMCL_TYPE_GP_TXID || req.instruction_type == TMCL_TYPE_GP_RXID)
		{
			(void) tmc_obj_srv->exec_tmcl_cmd(TMCL_CMD_SGP, req.instruction_type, (tmcl_motor_t) req.motor_num, &val);
			// In changing tx/rx, it is expected that exec_tmcl_cmd will not return rx data.
		}
		else
		{
			if (tmc_obj_srv->exec_tmcl_cmd(TMCL_CMD_SGP, req.instruction_type, (tmcl_motor_t) req.motor_num, &val))
			{
				res.output = val;
				ROS_DEBUG_STREAM("Service Done. Set value: " << res.output);
				b_result = true;
			}
			else
			{
				ROS_ERROR_STREAM("Service Failed!");
			}
		}

        res.result = b_result;
	}

	else if (req.instruction == "GGP")
	{
		if (tmc_obj_srv->exec_tmcl_cmd(TMCL_CMD_GGP, req.instruction_type, (tmcl_motor_t) req.motor_num, &val))
		{
			res.output = val;
			ROS_DEBUG_STREAM("Service Done. Get value: " << res.output);
			b_result = true;
		}
		else
		{
			ROS_ERROR_STREAM("Service Failed!");
		}

		res.result = b_result;
	}

	return b_result;
}
