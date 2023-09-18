/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmcl_ros.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmclROS::TmclROS(ros::NodeHandle* p_nh) : 
p_nh_ (p_nh), 
p_motor_(1),
p_tmcl_interpreter_(nullptr)
{
  ROS_DEBUG_STREAM("[TmclROS::" <<  __func__ << "] called");  
}

/* Destructor */
TmclROS::~TmclROS()
{
  ROS_DEBUG_STREAM("[TmclROS::" <<  __func__ << "] called");

  for(uint8_t index = 0; index < p_motor_.size(); index++)
  {
    delete p_motor_[index];
    p_motor_[index] = nullptr;
  }

  if(p_tmcl_interpreter_ != nullptr)
  {
    delete p_tmcl_interpreter_;
    p_tmcl_interpreter_ = nullptr;
    p_nh_ = nullptr;
  }
}

/* Initialization */
bool TmclROS::init()
{
  bool b_result = false;
  bool b_autostart_mode = false;
  int32_t val = 0;
  uint8_t instruction = 0;
  uint8_t firmware_value[TMCL_MSG_VALUE_SZ]; 

  total_motors_ = 0;

  ROS_INFO_STREAM("[TmclROS::" <<  __func__ << "] called");

  s_node_name_ = ros::this_node::getName();
  s_namespace_ = ros::this_node::getNamespace();
  ROS_INFO_STREAM("[" << __func__ << "] Namespace: " << s_namespace_);
  ROS_INFO_STREAM("[" << __func__ << "] Node name: " << s_node_name_);
  if(validateParams())
  {
    /* Initialize TMCL Interpreter */
    p_tmcl_interpreter_= new TmclInterpreter(param_comm_timeout_ms_, param_comm_exec_cmd_retries_,
      param_ap_name_, param_ap_type_);
    p_tmcl_interpreter_->tmcl_interface_ = (tmcl_interface_t) param_comm_interface_;
    p_tmcl_interpreter_->tmcl_cfg_.interface_name = param_comm_interface_name_;
    p_tmcl_interpreter_->tmcl_cfg_.tx_id = param_comm_tx_id_;
    p_tmcl_interpreter_->tmcl_cfg_.rx_id = param_comm_rx_id_;

    /* Reset the interface to be used */
    if(p_tmcl_interpreter_->resetInterface())
    {
      ROS_INFO_STREAM("[" << __func__ << "] resetInterface() success");

      /* Try to get FW version first before proceeding */
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_APPGFWV, 0x01, 0x00, &val))
      {
        /* Parse value for total motors, motor type and firmware version */
        firmware_value[0] = (val & 0xFF000000) >> 24;
        firmware_value[1] = (val & 0x00FF0000) >> 16;
        firmware_value[2] = (val & 0x0000FF00) >> 8;
        firmware_value[3] = (val & 0x000000FF);
        module_number_ = (firmware_value[0] << 8) + (firmware_value[1]);
        total_motors_ =  module_number_ / 1000;
        motor_type_ = (module_number_ - (total_motors_*1000)) / 100;

        ROS_INFO("[%s] Module number : %d", __func__, module_number_);
        ROS_INFO("[%s] Firmware version : %d.%d", __func__, firmware_value[2], firmware_value[3]);

        /* Try to get Autostart mode value */
        auto iterator = std::find(param_gp_name_.begin(), param_gp_name_.end(), "auto start mode");
        if(iterator != param_gp_name_.end()) 
        {
          instruction = param_gp_type_[std::distance(param_gp_name_.begin(), iterator)];
          (void) p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP, instruction, 0x00, &val);
          b_autostart_mode = val;
        } 
        else 
        {
          ROS_WARN_STREAM("[" << __func__ << "] Auto start mode not available");
        }

        if(b_autostart_mode)
        {
          ROS_INFO_STREAM("[" << __func__ << "] Auto start mode is enabled");
          ROS_INFO_STREAM("[" << __func__ << "] Wait " << 2.0 + param_auto_start_additional_delay_
            << "secs to autostart TMCL program");
          ros::Duration(2.0 + param_auto_start_additional_delay_).sleep();
        }
        else
        {
          ROS_INFO_STREAM("[" << __func__ << "] Auto start mode is disabled");
        }
        
        createMotor();
        initService();
        b_result = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] resetInterface failed");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] validateParams failed");
  }

  return b_result;
}

bool TmclROS::validateParams()
{
  bool b_result = true;

  ROS_INFO_STREAM("[TmclROS::" << __func__ << "] called");

  /* Get all configuration values from ROS parameters */
  /*================================================================================================================*/
  /* Parameters from Autogenerated YAML. Node will exit if any of the parameters is/are missing/broken */
  /*================================================================================================================*/
  const std::string s_ap_name = s_node_name_ + "/AP_name";
  if(!p_nh_->getParam(s_ap_name, param_ap_name_))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to get AP_name. "
      "Check autogenerated YAML if broken or missing. Exiting!");
    b_result = false;
  }
  const std::string s_ap_type = s_node_name_ + "/AP_type";
  if(b_result && !p_nh_->getParam(s_ap_type, param_ap_type_))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to get AP_type. "
      "Check autogenerated YAML if broken or missing. Exiting!");
    b_result = false;
  }

  /* Check if AP_name and AP_type vectors have the same length/size */
  if(b_result && (param_ap_name_.size() != param_ap_type_.size()))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Vector size mismatch between AP_name and AP_type. "
      "Check autogenerated YAML. Exiting!");
    b_result = false;
  }

  const std::string s_gp_name = s_node_name_ + "/GP_name";
  if(b_result && !p_nh_->getParam(s_gp_name, param_gp_name_))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to get GP_name. "
      "Check autogenerated YAML if broken or missing. Exiting!");
    b_result = false;
  }
  const std::string s_gp_type = s_node_name_ + "/GP_type";
  if(b_result && !p_nh_->getParam(s_gp_type, param_gp_type_))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to get GP_type. "
      "Check autogenerated YAML if broken or missing. Exiting!");
    b_result = false;
  }

  /* Check if GP_name and GP_type vectors have the same length/size */
  if(b_result && (param_gp_name_.size() != param_gp_type_.size()))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Vector size mismatch between GP_name and GP_type. "
      "Check autogenerated YAML. Exiting!");
    b_result = false;
  }

  if(b_result)
  {

    /*==============================================================================================================*/
    /*Parameters with no limits */
    /*==============================================================================================================*/
    const std::string s_comm_interface = s_node_name_ + "/comm_interface";
    if(!p_nh_->getParam(s_comm_interface, param_comm_interface_))
    {
      param_comm_interface_ = 0;
      p_nh_->setParam(s_comm_interface, param_comm_interface_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_interface, setting to default value: "
        << param_comm_interface_);
    }
    const std::string s_comm_interface_name = s_node_name_ + "/comm_interface_name";
    if(!p_nh_->getParam(s_comm_interface_name, param_comm_interface_name_))
    {
      param_comm_interface_name_ = "can0";
      p_nh_->setParam(s_comm_interface_name, param_comm_interface_name_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_interface_name, setting to default value: "
        << param_comm_interface_name_);  
    }
    const std::string s_adhoc_mode= s_node_name_ + "/adhoc_mode";
    if(!p_nh_->getParam(s_adhoc_mode, param_adhoc_mode_))
    {
      param_adhoc_mode_ = false;
      p_nh_->setParam(s_adhoc_mode, param_adhoc_mode_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get adhoc_mode, setting to default value: "
        << param_adhoc_mode_);
    }

    /*==============================================================================================================*/
    /*Parameters with limits. Will turn values to default if limit exceeds*/
    /*==============================================================================================================*/
    const std::string s_comm_tx_id = s_node_name_ + "/comm_tx_id";
    if(!p_nh_->getParam(s_comm_tx_id, param_comm_tx_id_))
    {
      param_comm_tx_id_ = TX_ID_DEFAULT;
      p_nh_->setParam(s_comm_tx_id, param_comm_tx_id_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_tx_id, setting to default value: "
        << param_comm_tx_id_);
    }
    else
    {
      if(param_comm_tx_id_ < 0 || param_comm_tx_id_ > TXRX_ID_MAX)
      {
        param_comm_tx_id_ = TX_ID_DEFAULT;
        p_nh_->setParam(s_comm_tx_id, param_comm_tx_id_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to comm_tx_id is out of range, "
          "setting comm_tx_id value to default: " << param_comm_tx_id_);
      }
    }
    const std::string s_comm_rx_id = s_node_name_ + "/comm_rx_id";
    if(!p_nh_->getParam(s_comm_rx_id, param_comm_rx_id_))
    {
      param_comm_rx_id_ = RX_ID_DEFAULT;
      p_nh_->setParam(s_comm_rx_id, param_comm_rx_id_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_rx_id, setting to default value: " 
        << param_comm_rx_id_); 
    }
    else
    {
      if(param_comm_rx_id_ < 0 || param_comm_rx_id_ > TXRX_ID_MAX)
      {
        param_comm_rx_id_ = RX_ID_DEFAULT;
        p_nh_->setParam(s_comm_rx_id, param_comm_rx_id_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to comm_rx_id is out of range, "
          "setting comm_rx_id value to default: " << param_comm_rx_id_);
      }
    }
    const std::string s_comm_timeout_ms = s_node_name_ + "/comm_timeout_ms";
    if(!p_nh_->getParam(s_comm_timeout_ms, param_comm_timeout_ms_))
    {
      param_comm_timeout_ms_ = TIMEOUT_MS_DEFAULT;
      p_nh_->setParam(s_comm_timeout_ms, param_comm_timeout_ms_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_timeout_ms, setting to default value: "
        << param_comm_timeout_ms_); 
    }
    else
    {
      if(param_comm_timeout_ms_ < 0 || param_comm_timeout_ms_ > TIMEOUT_MS_MAX)
      {
        param_comm_timeout_ms_ = TIMEOUT_MS_DEFAULT;
        p_nh_->setParam(s_comm_timeout_ms, param_comm_timeout_ms_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to comm_timeout_ms is out of range, "
          "setting comm_timeout_ms value to default: " << param_comm_timeout_ms_);
      }
    }
    const std::string s_comm_exec_cmd_retries = s_node_name_ + "/comm_exec_cmd_retries";
    if(!p_nh_->getParam(s_comm_exec_cmd_retries, param_comm_exec_cmd_retries_))
    {
      param_comm_exec_cmd_retries_ = EXEC_CMD_RETRIES_DEFAULT;
      p_nh_->setParam(s_comm_exec_cmd_retries, param_comm_exec_cmd_retries_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get comm_exec_cmd_retries, setting to default value: "
        << param_comm_exec_cmd_retries_); 
    }
    else
    {
      if(param_comm_exec_cmd_retries_ < 0 || param_comm_exec_cmd_retries_ > EXEC_CMD_RETRIES_MAX)
      {
        param_comm_exec_cmd_retries_ = EXEC_CMD_RETRIES_DEFAULT;
        p_nh_->setParam(s_comm_exec_cmd_retries, param_comm_exec_cmd_retries_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to comm_exec_cmd_retries is out of range, "
          "setting comm_timeout_retry value to default: " << param_comm_exec_cmd_retries_);
      }
    }
    const std::string s_pub_rate_tmc_info = s_node_name_ + "/pub_rate_tmc_info";
    if(!p_nh_->getParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info_))
    {
      param_pub_rate_tmc_info_ = PUB_RATE_DEFAULT;
      p_nh_->setParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_rate_tmc_info, setting to default value: "
        << param_pub_rate_tmc_info_);
    }
    else
    {
      if(param_pub_rate_tmc_info_ < PUB_RATE_MIN || param_pub_rate_tmc_info_ > PUB_RATE_MAX)
      {
        param_pub_rate_tmc_info_ = PUB_RATE_DEFAULT;
        p_nh_->setParam(s_pub_rate_tmc_info, param_pub_rate_tmc_info_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to pub_rate_tmc_info is out of range, "
          "setting pub_rate_tmc_info value to default: " << param_pub_rate_tmc_info_);
      }
    }
    const std::string s_auto_start_additional_delay = s_node_name_ + "/auto_start_additional_delay";
    if(!p_nh_->getParam(s_auto_start_additional_delay, param_auto_start_additional_delay_))
    {
      param_auto_start_additional_delay_ = AUTO_START_ADDITIONAL_DELAY_DEFAULT;
      p_nh_->setParam(s_auto_start_additional_delay, param_auto_start_additional_delay_);
      ROS_WARN_STREAM("[" << __func__ << "] Failed to get auto_start_additional_delay, setting to default value: "
        << param_auto_start_additional_delay_);
    }
    else
    {
      if(param_auto_start_additional_delay_ < AUTO_START_ADDITIONAL_DELAY_DEFAULT || 
        param_auto_start_additional_delay_ > AUTO_START_ADDITIONAL_DELAY_MAX)
      {
        param_auto_start_additional_delay_ = AUTO_START_ADDITIONAL_DELAY_DEFAULT;
        p_nh_->setParam(s_auto_start_additional_delay, param_auto_start_additional_delay_);
        ROS_WARN_STREAM("[" << __func__ << "] Set value to auto_start_additional_delay is out of range, "
          "setting pub_rate_tmc_info value to default: " << param_auto_start_additional_delay_);
      }
    }
  }

  return b_result;
}

/* Creation of motor */
void TmclROS::createMotor()
{
  int32_t val = 0;
  uint8_t index = 0;

  ROS_INFO_STREAM("[TmclROS::" << __func__ << "] called");

  /* Resize p_motor_ depending on total_motors_ value */
  p_motor_.resize(total_motors_, nullptr);

  /* Special handling for en_motors*/
  const std::string s_en_motors = s_node_name_ + "/en_motors";
  if(!p_nh_->getParam(s_en_motors, param_en_motors_))
  {
    for(index = 0; index < total_motors_; index++)
    {
      param_en_motors_.push_back(0);
    }
    p_nh_->setParam(s_en_motors, param_en_motors_);
    ROS_WARN("[%s] Failed to get en_motors, setting to default value: 0", __func__);
  }
  else
  {
    /* Check if en_motors size is equal to total_motors_ */
    if((param_en_motors_.size()) < total_motors_)
    {
      for(index = param_en_motors_.size(); index < total_motors_; index++)
      {
        param_en_motors_.push_back(0);
      }
      ROS_WARN("[%s] Missing indeces for en_motors, setting missing en_motors value to default: 0", __func__);
    }
    else if((param_en_motors_.size()) > total_motors_)
    {
      for(index = total_motors_; index <= param_en_motors_.size(); index++)
      {
        param_en_motors_.erase(param_en_motors_.begin() + total_motors_, param_en_motors_.end());
      }
      ROS_WARN("[%s] Indeces exceeded total motors available, deleting unused indeces", __func__);
    }
    /* Check each indeces of en_motors if valid */
    for(index = 0; index < param_en_motors_.size(); index++)
    {
      if(param_en_motors_[index] != 0 && param_en_motors_[index] != 1)
      {
        param_en_motors_[index] = 0;
        ROS_WARN("[%s] Set value to en_motors for motor %d is out of range, setting en_motors value to default: %d "
          , __func__, index, param_en_motors_[index]);
      }
    }
    p_nh_->setParam(s_en_motors, param_en_motors_);
  }

  /* Check for motor type */
  if(param_adhoc_mode_)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Adhoc Mode is enabled\n");
    
    //Generic
    for(index = 0; index < total_motors_; index++)
    {
      if(param_en_motors_[index])
      {
        p_motor_[index] = new Motor(p_nh_, p_tmcl_interpreter_, module_number_, index);
        p_motor_[index]->init();
      } 
      else
      {
        p_motor_[index] = nullptr;
        ROS_WARN("[%s] Motor %d is disabled\n", __func__, index);
      }
    }
  }
  else
  {
    //BLDC
    if(motor_type_ == MOTOR_TYPE_BLDC)
    {
      ROS_INFO_STREAM("[" << __func__ << "] Detected module for BLDC Motors\n");
      
      for(index = 0; index < total_motors_; index++)
      {
        if(param_en_motors_[index])
        {
          p_motor_[index] = new BLDCMotor(p_nh_, p_tmcl_interpreter_, module_number_, index);
          p_motor_[index]->init();
        } 
        else
        {
          p_motor_[index] = nullptr;
          ROS_WARN("[%s] motor %d is disabled\n", __func__, index);
        }
      }
    }

    //Stepper
    else
    {
      ROS_INFO_STREAM("[" << __func__ << "] Detected module for Stepper Motors\n");
      for(index = 0; index < total_motors_; index++)
      {
        if(param_en_motors_[index])
        {
          p_motor_[index] = new StepperMotor(p_nh_, p_tmcl_interpreter_, module_number_, index);
          p_motor_[index]->init();
        } 
        else
        {
          p_motor_[index] = nullptr;
          ROS_WARN("[%s] motor %d is disabled\n", __func__, index);
        }
      }
    }
  }
}

/* Initialize ROS Service */
void TmclROS::initService()
{
  ROS_INFO_STREAM("[TmclROS::" << __func__ << "] called");

  /* Make s_namespace_ empty if namespace is empty */
  if(s_namespace_.compare("/") == 0)
  {
    s_namespace_ = "";
  }

  std::string s_custom_cmd_srvname = s_namespace_ + "/tmcl_custom_cmd";
  std::string s_gap_all_srvname = s_namespace_ + "/tmcl_gap_all";
  std::string s_ggp_all_srvname = s_namespace_ + "/tmcl_ggp_all";

  custom_cmd_server_ = p_nh_->advertiseService(s_custom_cmd_srvname, &TmclROS::tmclCustomCMDCallBack, this);
  if(ros::service::exists(s_custom_cmd_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] tmcl_custom_cmd server advertised. Service name: " << s_custom_cmd_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] tmcl_custom_cmd server failed to advertise.");
  }

  gap_all_server_ = p_nh_->advertiseService(s_gap_all_srvname, &TmclROS::tmclGAPAllCallBack, this);
  if(ros::service::exists(s_gap_all_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] tmcl_gap_all server advertised. Service name: " << s_gap_all_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] tmcl_gap_all server failed to advertise.");
  }
  
  ggp_all_server_ = p_nh_->advertiseService(s_ggp_all_srvname, &TmclROS::tmclGGPAllCallBack, this);
  if(ros::service::exists(s_ggp_all_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] tmcl_ggp_all server advertised. Service name: " << s_ggp_all_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] tmcl_ggp_all server failed to advertise.");
  }
}

/* ROS Service Callbacks */
bool TmclROS::tmclCustomCMDCallBack(tmcl_ros::TmcCustomCmd::Request& req, tmcl_ros::TmcCustomCmd::Response& res)
{
  bool b_result = false;
  int32_t val = 0;

  if(req.instruction == "SAP")
  {
    val = req.value;
    if(req.motor_number < total_motors_ && 
      p_tmcl_interpreter_->executeCmd(TMCL_CMD_SAP, req.instruction_type, req.motor_number, &val))
    {
      b_result = true;
      ROS_DEBUG_STREAM("[" << __func__ << "] Service Done. Set value: " << res.output);
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service Failed!");
    }
  }
  else if(req.instruction == "GAP")
  {
    if(req.motor_number < total_motors_ && 
      p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, req.instruction_type, req.motor_number, &val))
    {
      b_result = true;
      ROS_DEBUG_STREAM("[" << __func__ << "] Service Done. Get value: " << res.output);
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service Failed!");
    }
  }
  else if(req.instruction == "SGP")
  {
    val = req.value;
    if(req.motor_number < total_motors_ && 
      p_tmcl_interpreter_->executeCmd(TMCL_CMD_SGP, req.instruction_type, req.motor_number, &val))
    {
      b_result = true;
      ROS_DEBUG_STREAM("[" << __func__ << "] Service Done. Set value: " << res.output);
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service Failed!");
    }
  }
  else if(req.instruction == "GGP")
  {
    if(req.motor_number < total_motors_ && 
      p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP, req.instruction_type, req.motor_number, &val))
    {
      b_result = true;
      ROS_DEBUG_STREAM("[" << __func__ << "] Service Done. Get value: " << res.output);
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service Failed!");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Unkown instruction.");
  }

  res.output = val;
  res.result = b_result;

  return b_result;
}
bool TmclROS::tmclGAPAllCallBack(tmcl_ros::TmcGapGgpAll::Request& req, tmcl_ros::TmcGapGgpAll::Response& res)
{
  bool b_result = true;
  int32_t val = 0;
  uint8_t index = 0;

  if(req.motor_number < total_motors_)
  {
    for(index = 0; index < param_ap_name_.size(); index++)
    {
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GAP, param_ap_type_[index], req.motor_number, &val))
      {
        tmc_param_msg_.name = param_ap_name_[index];
        tmc_param_msg_.value = val;
        res.output.push_back(tmc_param_msg_);
      }
      else
      {
        tmc_param_msg_.name = param_ap_name_[index] + ">>> ERROR";
        tmc_param_msg_.value = 0;
        res.output.push_back(tmc_param_msg_);
        b_result = false;
      }
    }
  }
  else
  {
    b_result = false;
    ROS_ERROR_STREAM("[" << __func__ << "] Required motor number exceeds to axis available");
  }
  res.result = b_result;

  return b_result;
}
bool TmclROS::tmclGGPAllCallBack(tmcl_ros::TmcGapGgpAll::Request& req, tmcl_ros::TmcGapGgpAll::Response& res)
{
  bool b_result = true;
  int32_t val = 0;
  uint8_t index = 0;

  if(req.motor_number == 0)
  {
    for(index = 0; index < param_gp_name_.size(); index++)
    {
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_GGP, param_gp_type_[index], req.motor_number, &val))
      {
        tmc_param_msg_.name = param_gp_name_[index];
        tmc_param_msg_.value = val;
        res.output.push_back(tmc_param_msg_);
      }
      else
      {
        tmc_param_msg_.name = param_gp_name_[index] + ">>> ERROR";
        tmc_param_msg_.value = 0;
        res.output.push_back(tmc_param_msg_);
        b_result = false;
      }
    }
  }
  else
  {
    b_result = false;
    ROS_ERROR_STREAM("[" << __func__ << "] GGP All Service only accepts value \"0\" as motor number");
  }
  res.result = b_result;


  return b_result;
}

/* Deinitialization */
bool TmclROS::deInit()
{   
  bool b_result = false;
  int32_t val = 0;

  ROS_INFO_STREAM("[TmclROS::" << __func__ << "] called");
  
  if(!this->getRetriesExceededStatus())
  {
    for(uint8_t index = 0; index < total_motors_; index++)
    {
      if(p_tmcl_interpreter_->executeCmd(TMCL_CMD_MST, 0, index , &val))
      {
        ROS_INFO("[%s] Stopped motor %d before shutting down...",__func__, index);
      }
      else
      {
        ROS_WARN("[%s] Timeout reached. Stopping of motor %d failed",__func__, index);
      }
    }
  }
  else 
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Timeout reached (board unresponsive), skipping motor stop commands");
  }
  
  /* Shut down the interface */
  if(p_tmcl_interpreter_ != nullptr && p_tmcl_interpreter_->shutdownInterface())
  {
    b_result = true;
    ROS_INFO_STREAM("[" << __func__ << "] shutdownInterface() success");
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] shutdownInterface() failed");
  }

  return b_result;
}

/* Getter b_retries_exceeded_ variable */
bool TmclROS::getRetriesExceededStatus()
{
  bool b_result = true;

  if(p_tmcl_interpreter_ != nullptr)
  {
    b_result = p_tmcl_interpreter_->getRetriesExceededStatus();
  }

  return b_result;
}
