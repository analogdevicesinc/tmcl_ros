/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_ros.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmclROS::TmclROS()
{
    /* Initialize all AP configurations to 0 */
    ap_cfg.resize(ROW_IDX_MAX, std::vector<int>(COL_IDX_MAX, 0));
    ap_cfg_ext.resize(ROW_IDX_MAX, std::vector<int>(TMCL_MOTOR_MAX, 0));
}

/* Destructor */
TmclROS::~TmclROS()
{

}

/* Initialization */
bool TmclROS::init(ros::NodeHandle nh)
{
  bool b_result = false;

  n_retries = 0;    // initialize n_retries 

  ROS_INFO_STREAM("[" <<  __func__ << "] called");

  s_node_name = ros::this_node::getName();
  s_namespace = ros::this_node::getNamespace();
  ROS_INFO_STREAM("Namespace: " << s_namespace);
  ROS_INFO_STREAM("Node name: " << s_node_name);

/* Validate all parameters first before using its values to set the TMC configurations */
  if(validate_params(nh))
  {
    /* Initialize TMCL Interpreter */
    tmcl = new TmclInterpreter();
    tmcl->tmcl_interface = (tmcl_interface_t) param_comm_interface;
    tmcl->tmcl_cfg.interface_name = param_comm_interface_name;
    tmcl->tmcl_cfg.bit_rate = param_comm_bit_rate;
    tmcl->tmcl_cfg.tx_id = param_comm_tx_id;
    tmcl->tmcl_cfg.rx_id = param_comm_rx_id;
    tmcl->timeout_ms = param_comm_timeout_ms;
	
/* Reset the interface to be used */
    b_result = tmcl->reset_interface();
    if(b_result)
    {
      ROS_INFO_STREAM("[" << __func__ << "] reset_interface() success");
      ROS_INFO_STREAM("Updating TMC settings...");
      int32_t val = 0;

      /* Try to get FW version first before proceeding */
      if(exec_tmcl_cmd((tmcl_cmd_t) 0x88, 0x01, (tmcl_motor_t) 0x00, &val))
      {
        ROS_INFO_STREAM("[" << __func__ << "] Able to get Firmware Version: " << val);
        ROS_INFO_STREAM("[" << __func__ << "] Trying to get and validate configurable axis parameter values");
        if(validate_configurable_axis_params(nh))
	    {
          ROS_INFO_STREAM("[" << __func__ << "] Able to validate configurable axis parameters");
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] ERROR! Validate configurable axis parameters failed (check the logs above for invalid parameters)");
          b_result = false;
	    }
      }
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "] ERROR! Getting board firmware version failed");
        b_result = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] ERROR! reset_interface() failed");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] ERROR! Validate parameters failed (check the logs above for invalid parameters)");
  }

  return b_result;
}

/* Get TMC information */
bool TmclROS::getInfo()
{
  bool b_result = false;
  
  for(int i = 0; i < param_total_motors; i++)
  {
    if(param_en_motors[i] && param_en_pub_tmc_info[i])
    {
      b_result = true;
      int32_t val = 0;

       /* Get actual board voltage if it will be published as part of TMC info */
       if (b_result && exec_tmcl_cmd(TMCL_CMD_GAP, ap_cfg[ROW_IDX_SupplyVoltage][COL_IDX_AP], (tmcl_motor_t) i, &val))
       {
         ap_cfg_ext[ROW_IDX_SupplyVoltage][i] = val;
         ROS_DEBUG_STREAM("[" << __func__ << "] Able to get actual board voltage");
       }
       else
       {
         b_result = false;
       }

       /* Get status flags if it will be published as part of TMC info */
       if (b_result && exec_tmcl_cmd(TMCL_CMD_GAP, ap_cfg[ROW_IDX_StatusFlags][COL_IDX_AP], (tmcl_motor_t) i, &val))
       {
         ap_cfg_ext[ROW_IDX_StatusFlags][i] = val;
         ROS_DEBUG_STREAM("[" << __func__ << "] Able to get status flags");
       }
       else
       {
         b_result = false;
       }

      /* Get actual velocity if it will be published as part of TMC info */
      if(b_result && ap_cfg_ext[ROW_IDX_CommutationMode][i] > 0 && param_pub_actual_vel[i])
      {
        if (exec_tmcl_cmd(TMCL_CMD_GAP, ap_cfg[ROW_IDX_ActualVelocity][COL_IDX_AP], (tmcl_motor_t) i, &val))
        {
          ap_cfg_ext[ROW_IDX_ActualVelocity][i] = val;
          ROS_DEBUG_STREAM("[" << __func__ << "] Able to get actual velocity");
        }
        else
        {
          b_result = false;
        }
      }

      /* Get actual torque if it will be published as part of TMC info */
      if(b_result && ap_cfg_ext[ROW_IDX_CommutationMode][i] > 1 && param_pub_actual_trq[i])
      {
        if (exec_tmcl_cmd(TMCL_CMD_GAP, ap_cfg[ROW_IDX_ActualTorque][COL_IDX_AP], (tmcl_motor_t) i, &val))
        {
          ap_cfg_ext[ROW_IDX_ActualTorque][i] = val;
          ROS_DEBUG_STREAM("[" << __func__ << "] Able to get actual torque");
        }
        else
        {
          b_result = false;
        }
      }  

      /* Get actual position if it will be published as part of TMC info */
      if(b_result && param_pub_actual_pos[i])
      {
        if (exec_tmcl_cmd(TMCL_CMD_GAP, ap_cfg[ROW_IDX_ActualPosition][COL_IDX_AP], (tmcl_motor_t) i, &val))
        {
          ap_cfg_ext[ROW_IDX_ActualPosition][i] = val;
          ROS_DEBUG_STREAM("[" << __func__ << "] Able to get actual position");
        }
        else
        {
          b_result = false;
        }
      }

      if(!b_result)
      {
        break;
      }
    }
	}

  return b_result;
}

/* Deinitialization */
bool TmclROS::deInit()
{
  bool b_result = false;
  int32_t val = 0;
  
  if((tmcl != nullptr && tmcl->interface_enabled) && n_retries > 0)
  {
    /* Stop all TMC movements */
    for(int i = 0; i < param_total_motors; i++)
    {
      val = 0;
      if((param_en_motors[i]) && exec_tmcl_cmd(TMCL_CMD_SAP, ap_cfg[ROW_IDX_TargetVelocity][COL_IDX_AP], (tmcl_motor_t) i, &val))
      {
        ROS_INFO_STREAM("[" << __func__ << "] Stopped motor " << i << " before shutting down...");
      }
    }
    printf("\n\n==%d==\n\n", val);
    (void) exec_tmcl_cmd(TMCL_CMD_STOP, 0, (tmcl_motor_t) 0, &val);
    
    /* Shut down the interface */
  	b_result = tmcl->shutdown_interface();
  }

  else 
  {
        ROS_INFO_STREAM("[" << __func__ << "] Timeout reached (board unresponsive), skipping stop of motors");
        b_result = true;
  }
	
    if(b_result)
    {
        ROS_INFO_STREAM("[" << __func__ << "] shutdown_interface() success");
    }
    else
    {
        ROS_INFO_STREAM("[" << __func__ << "] shutdown_interface() failed");
    }

  delete tmcl;
  tmcl = nullptr;
  return b_result;
}

/* Execute a TMCL command within the program */
bool TmclROS::exec_tmcl_cmd(tmcl_cmd_t cmd, uint8_t type, tmcl_motor_t motor, int32_t *val)
{
  bool b_result = false;
  n_retries = param_comm_exec_cmd_retries;
  tmcl_msg_t tmcl_msg;
  // Setting tx_id and rx_id one time here
  tmcl_msg.tx_id = tmcl->tmcl_cfg.tx_id;
  tmcl_msg.rx_id = tmcl->tmcl_cfg.rx_id;

  while(n_retries > 0)
  {
    // Setting cmd, type, motor, value is always needed every call to execute_cmd()
    tmcl_msg.cmd = cmd;
    tmcl_msg.type = type;
    tmcl_msg.motor = motor;
    tmcl_msg.value[0] = (*val & 0xFF000000) >> 24;
    tmcl_msg.value[1] = (*val & 0x00FF0000) >> 16;
    tmcl_msg.value[2] = (*val & 0x0000FF00) >> 8;
    tmcl_msg.value[3] = (*val & 0x000000FF);

    
    ROS_DEBUG("[%s][T%d] before execute_cmd(), value=%d sending [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", __func__, (param_comm_exec_cmd_retries - n_retries + 1),
        *val, tmcl_msg.tx_id, tmcl_msg.cmd, tmcl_msg.type, tmcl_msg.motor, tmcl_msg.value[0], tmcl_msg.value[1], tmcl_msg.value[2], tmcl_msg.value[3]);

    b_result = tmcl->execute_cmd(&tmcl_msg);
    if(b_result)
    {
        //* - Rx: | [[Reply  Address]] | <Module Address> | <Status> | <Command> | <Value> | <Value> | <Value> | <Value> | [[Checksum]]
        ROS_DEBUG("\n[%s][T%d] execute_cmd() success, received [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n", __func__, (param_comm_exec_cmd_retries - n_retries + 1),
            tmcl_msg.tx_id, tmcl_msg.sts, tmcl_msg.cmd, tmcl_msg.value[0], tmcl_msg.value[1], tmcl_msg.value[2], tmcl_msg.value[3]);

        *val = (tmcl_msg.value[0] << 24) + (tmcl_msg.value[1] << 16) + (tmcl_msg.value[2] << 8) + tmcl_msg.value[3];      
        break;
    }
    else
    {
        ROS_WARN_STREAM("[" << __func__ << "] execute_cmd() failed");
    }
    n_retries --;
  }

    return b_result;
}

bool TmclROS::validate_params(ros::NodeHandle nh)
{
  bool b_result = true;
  bool b_all_mtr_en = false;
  int index = 0;

/* Get all configuration values from ROS parameters */
/*================================================================================================================*/
  /*Parameters that have no limits */
  const std::string s_comm_interface = s_node_name + "/comm_interface";
  if(!nh.getParam(s_comm_interface, param_comm_interface))
  {
    param_comm_interface = 0;
    ROS_WARN_STREAM("Failed to get comm_interface, setting to default value: " << param_comm_interface);
  }
  const std::string s_comm_interface_name = s_node_name + "/comm_interface_name";
  if(!nh.getParam(s_comm_interface_name, param_comm_interface_name))
  {
    param_comm_interface_name = "can0";
    ROS_WARN_STREAM("Failed to get comm_interface_name, setting to default value: " << param_comm_interface_name);
    
  }
  const std::string s_total_motors = s_node_name + "/TotalMotors";
  if(!nh.getParam(s_total_motors, param_total_motors))
  {
    ROS_ERROR_STREAM("Failed to get TotalMotors. Double check your YAMLs. Exiting!");
    return false;
  }
  else
  {
    if(param_total_motors > TMCL_MOTOR_MAX)
    {
      ROS_ERROR_STREAM("Total motors configured (total_motors=" << param_total_motors << ") are greater than maximum supported motors (TMCL_MOTOR_MAX=" << TMCL_MOTOR_MAX << "). Please double check your configuration file. Exiting!");
      return false;
    }
  }
  const std::string s_wheel_diameter = s_node_name + "/wheel_diameter";
  if(!nh.getParam(s_wheel_diameter, param_wheel_diameter))
  {
    param_wheel_diameter = 0;
    ROS_WARN_STREAM("Failed to get wheel diameter, setting to default value: " << param_wheel_diameter);
  }
  const std::string s_tmc_info_topic = s_node_name + "/tmc_info_topic";
  if(!nh.getParam(s_tmc_info_topic, param_tmc_info_topic))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_info_topic.push_back(s_namespace + "/tmc_info_" + std::to_string(index));
    }
    ROS_WARN_STREAM("Failed to get tmc_info_topic, setting to default value: /tmc_info_n");
  }
  else
  {
    for (index = 0; index < param_total_motors; index++)
    {
      param_tmc_info_topic[index] = s_namespace + param_tmc_info_topic[index];
    }
    /* Set unused indeces to 0 (if total motors is less than max motor) */
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_info_topic[index] = "/tmc_info";
    }
  }
  const std::string s_tmc_cmd_vel_topic = s_node_name + "/tmc_cmd_vel_topic";
  if(!nh.getParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_vel_topic.push_back("/cmd_vel_" + std::to_string(index));
    }
    ROS_WARN_STREAM("Failed to get tmc_cmd_vel_topic, setting to default value: /cmd_vel_n");
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_vel_topic[index] = "/cmd_vel";
    }
  }
  const std::string s_tmc_cmd_abspos_topic = s_node_name + "/tmc_cmd_abspos_topic";
  if(!nh.getParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_abspos_topic.push_back("/cmd_abspos_" + std::to_string(index));
    }
    ROS_WARN_STREAM("Failed to get tmc_cmd_abspos_topic, setting to default value: /cmd_abspos_n");
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_abspos_topic[index] = "/cmd_abspos";
    }
  }
  const std::string s_tmc_cmd_relpos_topic = s_node_name + "/tmc_cmd_relpos_topic";
  if(!nh.getParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic))
  {
    for(int index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_relpos_topic.push_back("/cmd_relpos_" + std::to_string(index));
    }
    ROS_WARN_STREAM("Failed to get tmc_cmd_relpos_topic, setting to default value: /cmd_relpos_n");
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_relpos_topic[index] = "/cmd_relpos";
    }
  }
  const std::string s_tmc_cmd_trq_topic = s_node_name + "/tmc_cmd_trq_topic";
  if(!nh.getParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_trq_topic.push_back("/cmd_trq_" + std::to_string(index));
    }
    ROS_WARN_STREAM("Failed to get tmc_cmd_trq_topic, setting to default value: /cmd_trq_n");
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_tmc_cmd_trq_topic[index] = "/cmd_trq";
    }
  }
  const std::string s_en_pub_tmc_info = s_node_name + "/en_pub_tmc_info";
  if(!nh.getParam(s_en_pub_tmc_info, param_en_pub_tmc_info))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_en_pub_tmc_info.push_back(false);
    }
    ROS_WARN_STREAM("Failed to get en_pub_tmc_info, setting to default value to all motors: " << std::boolalpha << param_en_pub_tmc_info[0]);
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_en_pub_tmc_info[index] = false;
    }
  }
  const std::string s_pub_actual_vel = s_node_name + "/pub_actual_vel";
  if(!nh.getParam(s_pub_actual_vel, param_pub_actual_vel))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_vel.push_back(false);
    }
    ROS_WARN_STREAM("Failed to get pub_actual_vel, setting to default value to all motors: " << std::boolalpha << param_pub_actual_vel[0]);
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_vel[index] = false;
    }
  }
  const std::string s_pub_actual_trq = s_node_name + "/pub_actual_trq";
  if(!nh.getParam(s_pub_actual_trq, param_pub_actual_trq))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_trq.push_back(false);
    }
    ROS_WARN_STREAM("Failed to get pub_actual_trq, setting to default value to all motors: " << std::boolalpha << param_pub_actual_trq[0]);
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_trq[index] = false;
    }
  }
  const std::string s_pub_actual_pos = s_node_name + "/pub_actual_pos";
  if(!nh.getParam(s_pub_actual_pos, param_pub_actual_pos))
  {
    for(index = 0; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_pos.push_back(false);
    }
    ROS_WARN_STREAM("Failed to get pub_actual_pos, setting to default value to all motors: " << std::boolalpha << param_pub_actual_pos[0]);
  }
  else
  {
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_pub_actual_pos[index] = false;
    }
  }
  /*================================================================================================================*/
  /*Parameters with limits*/
  /*Will turn values to default if limit exceeds*/
  /*================================================================================================================*/

  const std::string s_comm_bit_rate = s_node_name + "/comm_bit_rate";
  if(!nh.getParam(s_comm_bit_rate, param_comm_bit_rate))
  {
    param_comm_bit_rate = RATE_1000KBPS;
    ROS_WARN_STREAM("Failed to get comm_bit_rate, setting to default value: " << param_comm_bit_rate);
  }
  else
  {
    if (param_comm_bit_rate != RATE_20KBPS && param_comm_bit_rate != RATE_50KBPS && param_comm_bit_rate != RATE_100KBPS&&
        param_comm_bit_rate != RATE_125KBPS && param_comm_bit_rate != RATE_250KBPS && param_comm_bit_rate != RATE_500KBPS &&
        param_comm_bit_rate != RATE_1000KBPS)
      {
        param_comm_bit_rate =  RATE_1000KBPS;
        ROS_WARN_STREAM("Set value to comm_bit_rate is invalid, setting comm_bit_rate value to default: " << param_comm_bit_rate);
      }
  }
  const std::string s_comm_tx_id = s_node_name + "/comm_tx_id";
  if(!nh.getParam(s_comm_tx_id, param_comm_tx_id))
  {
    param_comm_tx_id = TX_ID_DEFAULT;
    ROS_WARN_STREAM("Failed to get comm_tx_id, setting to default value: "<< param_comm_tx_id);
  }
  else
  {
    if (param_comm_tx_id < 0 || param_comm_tx_id > TXRX_ID_MAX)
    {
      param_comm_tx_id = TX_ID_DEFAULT;
      ROS_WARN_STREAM("Set value to comm_tx_id is out of range, setting comm_tx_id value to default: "<< param_comm_tx_id);
    }
  }
  const std::string s_comm_rx_id = s_node_name + "/comm_rx_id";
  if(!nh.getParam(s_comm_rx_id, param_comm_rx_id))
  {
    param_comm_rx_id = RX_ID_DEFAULT;
    ROS_WARN_STREAM("Failed to get comm_rx_id, setting to default value: " << param_comm_rx_id); 
  }
  else
  {
    if (param_comm_rx_id < 0 || param_comm_rx_id > TXRX_ID_MAX)
    {
      param_comm_rx_id = RX_ID_DEFAULT;
      ROS_WARN_STREAM("Set value to comm_rx_id is out of range, setting comm_rx_id value to default: " << param_comm_rx_id);
    }
  }
  const std::string s_comm_timeout_ms = s_node_name + "/comm_timeout_ms";
  if(!nh.getParam(s_comm_timeout_ms, param_comm_timeout_ms))
  {
    param_comm_timeout_ms = TIMEOUT_MS_DEFAULT;
    ROS_WARN_STREAM("Failed to get comm_timeout_ms, setting to default value: " << param_comm_timeout_ms); 
  }
  else
  {
    if (param_comm_timeout_ms < 0 || param_comm_timeout_ms > TIMEOUT_MS_MAX)
    {
      param_comm_timeout_ms = TIMEOUT_MS_DEFAULT;
      ROS_WARN_STREAM("Set value to comm_timeout_ms is out of range, setting comm_timeout_ms value to default: " << param_comm_timeout_ms);
    }
  }
  const std::string s_comm_exec_cmd_retries = s_node_name + "/comm_exec_cmd_retries";
  if(!nh.getParam(s_comm_exec_cmd_retries, param_comm_exec_cmd_retries))
  {
    param_comm_exec_cmd_retries = EXEC_CMD_RETRIES_DEFAULT;
    ROS_WARN_STREAM("Failed to get comm_exec_cmd_retries, setting to default value: " << param_comm_exec_cmd_retries); 
  }
  else
  {
    if (param_comm_exec_cmd_retries < 0 || param_comm_exec_cmd_retries > EXEC_CMD_RETRIES_MAX)
    {
      param_comm_exec_cmd_retries = EXEC_CMD_RETRIES_DEFAULT;
      ROS_WARN_STREAM("Set value to comm_exec_cmd_retries is out of range, setting comm_timeout_retry value to default: " << param_comm_exec_cmd_retries);
    }
  }
  const std::string s_pub_rate_tmc_info = s_node_name + "/pub_rate_tmc_info";
  if(!nh.getParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info))
  {
    param_pub_rate_tmc_info = 10;
    ROS_WARN_STREAM("Failed to get pub_rate_tmc_info, setting to default value: " << param_pub_rate_tmc_info);
  }
  else
  {
    if (param_pub_rate_tmc_info < 10 || param_pub_rate_tmc_info > PUB_RATE_MAX)
    {
      param_pub_rate_tmc_info = 10;
      ROS_WARN_STREAM("Set value to pub_rate_tmc_info is out of range, setting pub_rate_tmc_info value to default: " << param_pub_rate_tmc_info);
    }
  }
  const std::string s_en_motors = s_node_name + "/en_motors";
  if(!nh.getParam(s_en_motors, param_en_motors))
  {
    ROS_ERROR_STREAM("Failed to get en_motors. Double check your YAMLs. Exiting!");
    return false;
  }
  else
  {
    /* Checks if input is valid */
    for (index = 0; index < param_total_motors; index++)
    {
      if (param_en_motors[index] != 0 && param_en_motors[index] != 1)
      {
        param_en_motors[index] = 0;
        ROS_WARN_STREAM("Set value to en_motors for motor" <<index<< " is out of range, setting en_motors value to default: " << param_en_motors[index]);
      }
    }
    /* Set unused indeces to 0 (if total motors is less than max motor) */
    for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
    {
      param_en_motors[index] = 0;
    }
  }

  /*================================================================================================================*/
  /*                                        Axis Parameter configurations                                           */
  /*================================================================================================================*/

  ////////////////////////////////
  // Non-user-configurable ones 
  ////////////////////////////////

  /* StatusFlags checks */
  const std::string s_status_flags = s_node_name + "/StatusFlags";
  if(nh.getParam(s_status_flags, ap_cfg[ROW_IDX_StatusFlags]))
  {
    ROS_INFO_STREAM("Succeeded to get StatusFlags.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get StatusFlags. Double check your YAMLs. Exiting!");
    return false;
  }
  const std::string s_status_flags_reg_name = s_node_name + "/StatusFlagsRegName";
  if(nh.getParam(s_status_flags_reg_name, param_status_flags_reg_name))
  {
    ROS_INFO_STREAM("Succeeded to get StatusFlagsRegName.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get StatusFlagsRegName. Double check your YAMLs. Exiting!");
    return false;
  }
  const std::string s_status_flags_reg_shift = s_node_name + "/StatusFlagsRegShift";
  if(nh.getParam(s_status_flags_reg_shift, param_status_flags_reg_shift))
  {
    ROS_INFO_STREAM("Succeeded to get StatusFlagsRegShift.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get StatusFlagsRegShift. Double check your YAMLs. Exiting!");
    return false;
  }

  /* Both StatusFlagsRegName and StatusFlagsRegShift should have the same size of vector*/
  if (param_status_flags_reg_name.size() != param_status_flags_reg_shift.size())
  {
    ROS_ERROR_STREAM("StatusFlagsRegName and StatusFlagsRegShift have different size of vector. Double check your YAMLs. Exiting!");
    return false;
  }

  /* SupplyVoltage checks */
  const std::string s_SupplyVoltage = s_node_name + "/SupplyVoltage";
  if(nh.getParam(s_SupplyVoltage, ap_cfg[ROW_IDX_SupplyVoltage]))
  {
    ROS_INFO_STREAM("Succeeded to get SupplyVoltage.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get SupplyVoltage. Double check your YAMLs. Exiting!");
    return false;
  }

  /* TargetVelocity checks */
  const std::string s_TargetVelocity = s_node_name + "/TargetVelocity";
  if(nh.getParam(s_TargetVelocity, ap_cfg[ROW_IDX_TargetVelocity]))
  {
    ROS_INFO_STREAM("Succeeded to get TargetVelocity.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get TargetVelocity. Double check your YAMLs. Exiting!");
    return false;
  }

  /* ActualVelocity checks */
  const std::string s_ActualVelocity = s_node_name + "/ActualVelocity";
  if(nh.getParam(s_ActualVelocity, ap_cfg[ROW_IDX_ActualVelocity]))
  {
    ROS_INFO_STREAM("Succeeded to get ActualVelocity.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get ActualVelocity. Double check your YAMLs. Exiting!");
    return false;
  }

  /* TargetPosition checks */
  const std::string s_TargetPosition = s_node_name + "/TargetPosition";
  if(nh.getParam(s_TargetPosition, ap_cfg[ROW_IDX_TargetPosition]))
  {
    ROS_INFO_STREAM("Succeeded to get TargetPosition.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get TargetPosition. Double check your YAMLs. Exiting!");
    return false;
  }

  /* ActualPosition checks */
  const std::string s_ActualPosition = s_node_name + "/ActualPosition";
  if(nh.getParam(s_ActualPosition, ap_cfg[ROW_IDX_ActualPosition]))
  {
    ROS_INFO_STREAM("Succeeded to get ActualPosition.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get ActualPosition. Double check your YAMLs. Exiting!");
    return false;
  }

  /* TargetTorque checks */
  const std::string s_TargetTorque = s_node_name + "/TargetTorque";
  if(nh.getParam(s_TargetTorque, ap_cfg[ROW_IDX_TargetTorque]))
  {
    ROS_INFO_STREAM("Succeeded to get TargetTorque.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get TargetTorque. Double check your YAMLs. Exiting!");
    return false;
  }

  /* ActualTorque checks */
  const std::string s_ActualTorque = s_node_name + "/ActualTorque";
  if(nh.getParam(s_ActualTorque, ap_cfg[ROW_IDX_ActualTorque]))
  {
    ROS_INFO_STREAM("Succeeded to get ActualTorque.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get ActualTorque. Double check your YAMLs. Exiting!");
    return false;
  }

  /*================================================================================================================*/

  ROS_INFO_STREAM("\n\n============UPDATED PARAMETERS: ============");
  ROS_INFO_STREAM("comm_interface:         [" <<param_comm_interface<< "]");
  ROS_INFO_STREAM("comm_interface_name:    [" <<param_comm_interface_name<< "]");
  ROS_INFO_STREAM("comm_bit_rate:          [" << param_comm_bit_rate << "]");
  ROS_INFO_STREAM("comm_tx_id:             [" << param_comm_tx_id<< "]");
  ROS_INFO_STREAM("comm_rx_id:             [" << param_comm_rx_id<< "]");
  ROS_INFO_STREAM("TotalMotors:            [" <<param_total_motors<< "]");
  ROS_INFO_STREAM("wheel_diameter:         [" <<param_wheel_diameter<< "]");
  ROS_INFO_STREAM("comm_timeout_secs:      [" << param_comm_timeout_ms<< "]");
  ROS_INFO_STREAM("comm_timeout_retry:     [" << param_comm_exec_cmd_retries<< "]");
  ROS_INFO_STREAM("pub_rate_tmc_info:      [" << param_pub_rate_tmc_info<< "]");
  ROS_INFO_STREAM("tmc_info_topic:         ["<<param_tmc_info_topic[0]<<","<<param_tmc_info_topic[1]<<","<<param_tmc_info_topic[2]<<"]");
  ROS_INFO_STREAM("tmc_cmd_vel_topic:      ["<<param_tmc_cmd_vel_topic[0]<<","<<param_tmc_cmd_vel_topic[1]<<","<<param_tmc_cmd_vel_topic[2]<<"]");
  ROS_INFO_STREAM("tmc_cmd_abspos_topic:   ["<<param_tmc_cmd_abspos_topic[0]<<","<<param_tmc_cmd_abspos_topic[1]<<","<<param_tmc_cmd_abspos_topic[2]<<"]");
  ROS_INFO_STREAM("tmc_cmd_relpos_topic:   ["<<param_tmc_cmd_relpos_topic[0]<<","<<param_tmc_cmd_relpos_topic[1]<<","<<param_tmc_cmd_relpos_topic[2]<<"]");
  ROS_INFO_STREAM("tmc_cmd_trq_topic:      ["<<param_tmc_cmd_trq_topic[0]<<","<<param_tmc_cmd_trq_topic[1]<<","<<param_tmc_cmd_trq_topic[2]<<"]");
  ROS_INFO_STREAM("en_pub_tmc_info:        ["<< std::boolalpha <<param_en_pub_tmc_info[0]<<","<<param_en_pub_tmc_info[1]<<","<<param_en_pub_tmc_info[2]<<"]");
  ROS_INFO_STREAM("en_motors:              ["<< std::boolalpha <<param_en_motors[0]<<","<<param_en_motors[1]<<","<<param_en_motors[2]<<"]");
  ROS_INFO_STREAM("pub_actual_vel:         ["<< std::boolalpha <<param_pub_actual_vel[0]<<","<<param_pub_actual_vel[1]<<","<<param_pub_actual_vel[2]<<"]");
  ROS_INFO_STREAM("pub_actual_pos:         ["<< std::boolalpha <<param_pub_actual_pos[0]<<","<<param_pub_actual_pos[1]<<","<<param_pub_actual_pos[2]<<"]");
  ROS_INFO_STREAM("pub_actual_trq:         ["<< std::boolalpha <<param_pub_actual_trq[0]<<","<<param_pub_actual_trq[1]<<","<<param_pub_actual_trq[2]<<"]");
  ROS_INFO_STREAM("\n\n");

  for(index = 0; index < param_total_motors; index++)
  {
    if(param_en_motors[index])
    b_all_mtr_en = true;
  }
  if(!b_all_mtr_en)
  {
    ROS_ERROR_STREAM("All motors are disabled. Exiting");
    b_result = false;
  }

  /*================================================================================================================*/
  /*setParam all parameters*/
  /*================================================================================================================*/
  nh.setParam(s_comm_interface, param_comm_interface);
  nh.setParam(s_comm_interface_name, param_comm_interface_name);
  nh.setParam(s_comm_bit_rate, param_comm_bit_rate);
  nh.setParam(s_comm_tx_id, param_comm_tx_id);
  nh.setParam(s_comm_rx_id, param_comm_rx_id);
  nh.setParam(s_comm_timeout_ms, param_comm_timeout_ms);
  nh.setParam(s_comm_exec_cmd_retries, param_comm_exec_cmd_retries);
  nh.setParam(s_total_motors, param_total_motors);
  nh.setParam(s_wheel_diameter, param_wheel_diameter);
  nh.setParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info);
  nh.setParam(s_tmc_info_topic, param_tmc_info_topic);
  nh.setParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic);
  nh.setParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic);
  nh.setParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic);
  nh.setParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic);
  nh.setParam(s_en_pub_tmc_info, param_en_pub_tmc_info);
  nh.setParam(s_en_motors,param_en_motors);
  nh.setParam(s_pub_actual_vel, param_pub_actual_vel);
  nh.setParam(s_pub_actual_trq, param_pub_actual_trq);
  nh.setParam(s_pub_actual_pos, param_pub_actual_pos);

  return b_result;
}

bool TmclROS::validate_configurable_axis_params(ros::NodeHandle nh)
{
  bool b_result = true;
  int index = 0;

  /*================================================================================================================*/
  /*Validate AP*/
  /*================================================================================================================*/

  /* FollowEepromConfig checks */
  const std::string s_FollowEepromConfig = s_node_name + "/FollowEepromConfig";
  if(nh.getParam(s_FollowEepromConfig, param_follow_eeprom_cfg))
  {
    ROS_INFO_STREAM("Succeeded to get FollowEepromConfig.");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get FollowEepromConfig. Double check your YAMLs. Exiting!");
    return false;
  }

  // Validate TMC configurations
  for(index = 8; index < ROW_IDX_MAX; index++) // Configurable setting starts at 7th ROW_IDX_*
  {
    if(!validate_ap_cfg(nh, tmcl_lut_row_str[index], index))
    {
      ROS_ERROR_STREAM(tmcl_lut_row_str[index] << " values are invalid (check logs above to identify which)");
      ROS_ERROR_STREAM("Either configure TMC properly via IDE and store such configurations in the TMC EEPROM; OR set correct values in your YAML. Exiting!");
      return false;
    }
  }

  /*================================================================================================================*/
  /*Print updated axis parameters*/
  /*================================================================================================================*/

  ROS_INFO_STREAM("\n\n============UPDATED AXIS PARAMETERS: ============");
  ROS_INFO_STREAM("CommutationMode         ["<<ap_cfg_ext[ROW_IDX_CommutationMode][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_CommutationMode][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_CommutationMode][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("CommutationModeVelocity ["<<ap_cfg_ext[ROW_IDX_CommutationModeVelocity][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_CommutationModeVelocity][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_CommutationModeVelocity][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("CommutationModePosition ["<<ap_cfg_ext[ROW_IDX_CommutationModePosition][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_CommutationModePosition][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_CommutationModePosition][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("MaxTorque               ["<<ap_cfg_ext[ROW_IDX_MaxTorque][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_MaxTorque][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_MaxTorque][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("OpenLoopCurrent         ["<<ap_cfg_ext[ROW_IDX_OpenLoopCurrent][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_OpenLoopCurrent][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_OpenLoopCurrent][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("Acceleration            ["<<ap_cfg_ext[ROW_IDX_Acceleration][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_Acceleration][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_Acceleration][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("MotorPolePairs          ["<<ap_cfg_ext[ROW_IDX_MotorPolePairs][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_MotorPolePairs][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_MotorPolePairs][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("PWMFrequency            ["<<ap_cfg_ext[ROW_IDX_PWMFrequency][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_PWMFrequency][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_PWMFrequency][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("HallSensorPolarity      ["<<ap_cfg_ext[ROW_IDX_HallSensorPolarity][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_HallSensorPolarity][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_HallSensorPolarity][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("HallSensorDirection     ["<<ap_cfg_ext[ROW_IDX_HallSensorDirection][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_HallSensorDirection][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_HallSensorDirection][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("HallInterpolation       ["<<ap_cfg_ext[ROW_IDX_HallInterpolation][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_HallInterpolation][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_HallInterpolation][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("HallSensorOffset        ["<<ap_cfg_ext[ROW_IDX_HallSensorOffset][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_HallSensorOffset][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_HallSensorOffset][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("EncoderDirection        ["<<ap_cfg_ext[ROW_IDX_EncoderDirection][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_EncoderDirection][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_EncoderDirection][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("EncoderSteps            ["<<ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_EncoderSteps][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("EncoderInitMode         ["<<ap_cfg_ext[ROW_IDX_EncoderInitMode][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_EncoderInitMode][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_EncoderInitMode][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("TorqueP                 ["<<ap_cfg_ext[ROW_IDX_TorqueP][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_TorqueP][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_TorqueP][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("TorqueI                 ["<<ap_cfg_ext[ROW_IDX_TorqueI][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_TorqueI][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_TorqueI][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("VelocityP               ["<<ap_cfg_ext[ROW_IDX_VelocityP][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_VelocityP][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_VelocityP][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("VelocityI               ["<<ap_cfg_ext[ROW_IDX_VelocityI][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_VelocityI][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_VelocityI][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("PositionP               ["<<ap_cfg_ext[ROW_IDX_PositionP][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_PositionP][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_PositionP][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("BrakeChopperEnabled     ["<<ap_cfg_ext[ROW_IDX_BrakeChopperEnabled][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperEnabled][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperEnabled][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("BrakeChopperVoltage     ["<<ap_cfg_ext[ROW_IDX_BrakeChopperVoltage][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperVoltage][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperVoltage][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("BrakeChopperHysteresis  ["<<ap_cfg_ext[ROW_IDX_BrakeChopperHysteresis][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperHysteresis][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_BrakeChopperHysteresis][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("PositionScalerM         ["<<ap_cfg_ext[ROW_IDX_PositionScalerM][TMCL_MOTOR_0]<<","<<ap_cfg_ext[ROW_IDX_PositionScalerM][TMCL_MOTOR_1]<<","<<ap_cfg_ext[ROW_IDX_PositionScalerM][TMCL_MOTOR_2]<<"]");
  ROS_INFO_STREAM("\n\n");

  return b_result;
}

bool TmclROS::validate_ap_cfg(ros::NodeHandle nh, std::string ap_cfg_str, uint8_t lut_idx)
{
  bool b_result = true;
  int index = 0;
  tmcl_cmd_t cmd = TMCL_CMD_MAX;

  const std::string s_Config = s_node_name + "/" + ap_cfg_str;
  const std::string s_Config_Ext = s_Config + "_Ext";
  if(nh.getParam(s_Config, ap_cfg[lut_idx]))
  {
    ROS_INFO_STREAM("Succeeded to get " << ap_cfg_str);
    if(ap_cfg[lut_idx][COL_IDX_USED])
    {
      /* Option 1: User set the proper/calibrated configuration in the TMC EEPROM before and would like to use whatever's stored in the EEPROM */
      if(param_follow_eeprom_cfg)
      {
        ROS_INFO_STREAM("Using TMC EEPROM-stored values for " << ap_cfg_str << ".");
        cmd = TMCL_CMD_GAP;
      }
      /* Option 2: User set the configuration in the YAML file */
      else
      {
        ROS_INFO_STREAM("Using user-set values for " << ap_cfg_str << " specified in YAML as " << ap_cfg_str << "_Ext.");
        cmd = TMCL_CMD_SAP;
        if(!nh.getParam(s_Config_Ext, ap_cfg_ext[lut_idx]))
        {
          ROS_ERROR_STREAM("Failed to get/set TMC (YAML) value for " << ap_cfg_str << ". Double check in YAML. Exiting!");
          b_result = false;
        }
      }

      if(b_result)
      {
        if((ap_cfg[lut_idx].size() == COL_IDX_MAX) && (ap_cfg_ext[lut_idx].size() == TMCL_MOTOR_MAX))
        {
          /* Checks if input is valid */
          for(index = 0 ; index < param_total_motors; index++ )
          {
            if(param_en_motors[index])
            {
              /* Either get or set the values depending on Option 1 or 2 above */
              if(!exec_tmcl_cmd(cmd, ap_cfg[lut_idx][COL_IDX_AP], (tmcl_motor_t) index, (int32_t *) &ap_cfg_ext[lut_idx][index]))
              {
                ROS_ERROR_STREAM("Failed to get/set TMC value for " << ap_cfg_str << " (motor = " << index << "). Double check in YAML/TMCL-IDE. Exiting!");
                b_result = false;
                break;
              }

              /* Regardless of ap_cfg_ext[] containing either of ROS YAML or TMC EEPROM, ap_cfg_ext[] values will still need to be validated
               * because it might be dangerous for ROS Driver to still proceed and continously send TMC commands with invalid values. */
              if((ap_cfg_ext[lut_idx][index] < ap_cfg[lut_idx][COL_IDX_VAL_MIN]) || (ap_cfg_ext[lut_idx][index] > ap_cfg[lut_idx][COL_IDX_VAL_MAX]))
              {
                ROS_WARN_STREAM("Set value to " << ap_cfg_str.c_str() << " for motor " << index <<" is out of range, changing value to default: " << ap_cfg[lut_idx][COL_IDX_VAL_DEFAULT]);
                ap_cfg_ext[lut_idx][index] = ap_cfg[lut_idx][COL_IDX_VAL_DEFAULT];

                // Set updated configuration used in TMC runtime after validation changed it
                ROS_WARN_STREAM("Overriding EEPROM/YAML settings with the validated values instead of the invalid values...");
                if(!exec_tmcl_cmd(TMCL_CMD_SAP, ap_cfg[lut_idx][COL_IDX_AP], (tmcl_motor_t) index, (int32_t *) &ap_cfg_ext[lut_idx][index]))
                {
                  ROS_ERROR_STREAM("Failed to set TMC value for " << ap_cfg_str << " (motor = " << index << "). Exiting!");
                  b_result = false;
                  break;
                }
              }
            }
          }
          /* Set unused indeces to 0 (if total motors is less than max motor) */
          for (index = param_total_motors; index < TMCL_MOTOR_MAX; index++)
          {
            ap_cfg_ext[lut_idx][index] = 0;
          }
        }
        else
        {
          ROS_ERROR_STREAM("Incorrect configuration size for " << ap_cfg_str.c_str() << ". Exiting!");
          b_result = false;
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("Configuration " << ap_cfg_str << " is there but unused. Skipping validation for this.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Configuration " << ap_cfg_str << " is expected to be in the autogenerated YAML. Exiting!");
    b_result = false;
  }

  if(b_result)
  {
    ROS_INFO_STREAM(ap_cfg_str << " values are valid");
    nh.setParam(s_Config_Ext, ap_cfg_ext[lut_idx]);
  
  }
  else
  {
    ROS_ERROR_STREAM(ap_cfg_str << " values are invalid. Either configure TMC properly via IDE and store such configurations in the TMC EEPROM; OR set correct values in your YAML. Exiting!");
  }

  return b_result;
}
