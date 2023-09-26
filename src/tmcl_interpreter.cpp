/**
 * Copyright (c) 2022-2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <ros/console.h>
#include "tmcl_interpreter.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmclInterpreter::TmclInterpreter(uint16_t timeout_ms, uint8_t comm_exec_cmd_retries, 
  std::vector<std::string> param_ap_name, std::vector<int> param_ap_type) :
  timeout_ms_(timeout_ms), 
  comm_exec_cmd_retries_(comm_exec_cmd_retries), 
  param_ap_name_(param_ap_name), 
  param_ap_type_(param_ap_type)
{
  ROS_DEBUG_STREAM("[TmclInterpreter::" <<  __func__ << "] called");

  tmcl_interface_ = TMCL_INTERFACE_CAN;

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.p_socket_can = nullptr;
    tmcl_cfg_.interface_name = "can0";
    tmcl_cfg_.tx_id = 1;
    tmcl_cfg_.rx_id = 2;
  }
  interface_enabled_ = false;
  b_retries_exceeded_ = false;
}

/* Destructor */
TmclInterpreter::~TmclInterpreter()
{
  ROS_DEBUG_STREAM("[TmclInterpreter::" <<  __func__ << "] called");

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.interface_name = "";
    tmcl_cfg_.tx_id = 0;
    tmcl_cfg_.rx_id = 0;
  }
  interface_enabled_ = false;
}

/* Reset interface */
bool TmclInterpreter::resetInterface()
{
  ROS_INFO_STREAM("[TmclInterpreter::" <<  __func__ << "] called");

  bool b_result = false;

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.p_socket_can = new SocketCAN();

    if(tmcl_cfg_.p_socket_can->initialize(tmcl_cfg_.interface_name.c_str()))
    {
      ROS_INFO_STREAM("[" << __func__ <<"] called");
      interface_enabled_ = true;
      b_result = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ <<"] Interface not yet supported");
  }

  return b_result;
}

/* Execute a TMCL command within the program */
bool TmclInterpreter::executeCmd(tmcl_cmd_t cmd, uint8_t type, uint8_t motor, int32_t *val)
{
  ROS_DEBUG_STREAM("[" <<  __func__ << "] called");
  
  bool b_result = false;
  uint8_t n_retries = comm_exec_cmd_retries_;
  uint8_t rx_msg[TMCL_MSG_SZ] = {0};
  uint32_t rx_msg_id = 0;
  uint8_t rx_msg_sz = 0;

  tmcl_msg_t tmcl_msg;

  if(interface_enabled_)
  {
    if(tmcl_interface_ == TMCL_INTERFACE_CAN)
    {
      tmcl_msg.tx_id = tmcl_cfg_.tx_id;
      tmcl_msg.rx_id = tmcl_cfg_.rx_id;
      tmcl_msg.cmd = cmd;
      tmcl_msg.type = type;
      tmcl_msg.motor = motor;
      tmcl_msg.value[0] = (*val & 0xFF000000) >> 24;
      tmcl_msg.value[1] = (*val & 0x00FF0000) >> 16;
      tmcl_msg.value[2] = (*val & 0x0000FF00) >> 8;
      tmcl_msg.value[3] = (*val & 0x000000FF);
      // Setting cmd, type, motor, value is always needed every call to execute_cmd()
      uint8_t tx_msg[TMCL_MSG_SZ] = {tmcl_msg.cmd, tmcl_msg.type, tmcl_msg.motor, tmcl_msg.value[0], 
        tmcl_msg.value[1], tmcl_msg.value[2], tmcl_msg.value[3]};
      auto start_time = std::chrono::system_clock::now();
      auto end_time = start_time;

      while (0 < n_retries)
      {
        ROS_DEBUG("[%s] [T%d] before execute_cmd(), value=%d sending "
          "[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", __func__, n_retries, *val, tmcl_msg.tx_id,
          tmcl_msg.cmd, tmcl_msg.type, tmcl_msg.motor, tmcl_msg.value[0], tmcl_msg.value[1], tmcl_msg.value[2], 
          tmcl_msg.value[3]);

        /* Send TMCL command */
        if(tmcl_cfg_.p_socket_can->writeFrame(tmcl_msg.tx_id, tx_msg, TMCL_MSG_SZ))
        {
          while(timeout_ms_ > std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count())
          {
            if(tmcl_cfg_.p_socket_can->framesAvailable() && 
              tmcl_cfg_.p_socket_can->readFrame(&rx_msg_id, rx_msg, &rx_msg_sz))
            {
              /* A response for the TMCL command is received */
              if((rx_msg_id == tmcl_msg.rx_id) && 
                (rx_msg_sz == TMCL_MSG_SZ) &&
                (rx_msg[0] == tmcl_msg.tx_id) &&
                (rx_msg[2] == tmcl_msg.cmd))
              {
                tmcl_msg.sts = (tmcl_sts_t)rx_msg[1];
                tmcl_msg.value[0] = rx_msg[3];
                tmcl_msg.value[1] = rx_msg[4];
                tmcl_msg.value[2] = rx_msg[5];
                tmcl_msg.value[3] = rx_msg[6];

                //Rx: [Reply Address] | <Module Address> | <Status> | <Command> | <Value> | <Value> | <Value> | <Value> | [Checksum]
                ROS_DEBUG("\n[%s] [T%d] execute_cmd() success, received "
                  "[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]\n", __func__, n_retries, tmcl_msg.tx_id,
                  tmcl_msg.sts, tmcl_msg.cmd, tmcl_msg.value[0], tmcl_msg.value[1], tmcl_msg.value[2],
                  tmcl_msg.value[3]);
                *val = (tmcl_msg.value[0] << 24) + (tmcl_msg.value[1] << 16) + (tmcl_msg.value[2] << 8) + 
                  tmcl_msg.value[3];
                b_result = true;
                break;
              }
            }
            end_time = std::chrono::system_clock::now();
          }
        }
        
        if(!b_result)
        {
          n_retries--;
        }
        else
        {
          break;
        }
      }

      if(n_retries == 0)
      {
        b_retries_exceeded_ = true;
        ROS_ERROR_STREAM("[" << __func__ << "] Retries exceeded");
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Interface not yet supported");
    }
  }

  return b_result;
}

bool TmclInterpreter::executeCmd(tmcl_cmd_t cmd, std::string type, uint8_t motor, int32_t *val)
{
  bool b_result = false;
  uint8_t int_type = 0;
  uint8_t index = 0;

  auto iterator = std::find(param_ap_name_.begin(), param_ap_name_.end(), type);
    
  if(iterator != param_ap_name_.end()) 
  {
    index = std::distance(param_ap_name_.begin(), iterator);
    int_type = param_ap_type_[index];
    b_result = executeCmd(cmd, int_type, motor, val);
  } 
  else 
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Instruction Type: " << type << " not found");
  }

  return b_result;
}

/* Shutdown interface */
bool TmclInterpreter::shutdownInterface()
{
  bool b_result = false;
  
  ROS_INFO_STREAM("[TmclInterpreter::" << __func__ << "] called");

  if(tmcl_interface_ == TMCL_INTERFACE_CAN)
  {
    tmcl_cfg_.p_socket_can->deinitialize();
    delete tmcl_cfg_.p_socket_can;
    tmcl_cfg_.p_socket_can = nullptr;
    interface_enabled_ = false;
    b_result = true;
  }
  
  return b_result;
}

/* Getter b_retries_exceeded_ variable */
bool TmclInterpreter::getRetriesExceededStatus()
{
  return b_retries_exceeded_;
}
