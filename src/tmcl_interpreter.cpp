/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <ros/console.h>
#include "tmcl_interpreter.h"

////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmclInterpreter::TmclInterpreter()
{
    tmcl_interface = TMCL_INTERFACE_CAN;

    if(tmcl_interface == TMCL_INTERFACE_CAN)
    {
        tmcl_cfg.socket_can = nullptr;
        tmcl_cfg.interface_name = "can0";
        tmcl_cfg.bit_rate = 1000000;
        tmcl_cfg.tx_id = 1;
        tmcl_cfg.rx_id = 2;
    }
    interface_enabled = false;
    timeout_ms = 0;
}

/* Destructor */
TmclInterpreter::~TmclInterpreter()
{
    if(tmcl_interface == TMCL_INTERFACE_CAN)
    {
        tmcl_cfg.interface_name = "";
        tmcl_cfg.bit_rate = 0;
        tmcl_cfg.tx_id = 0;
        tmcl_cfg.rx_id = 0;
    }
    interface_enabled = false;
    timeout_ms = 0;
}

/* Reset interface */
bool TmclInterpreter::reset_interface()
{
    bool b_result = false;

    if(tmcl_interface == TMCL_INTERFACE_CAN)
    {
        tmcl_cfg.socket_can = new SocketCAN();
        if(tmcl_cfg.socket_can->initialize(tmcl_cfg.interface_name.c_str(), tmcl_cfg.bit_rate))
        {
            ROS_INFO_STREAM("[" << __func__ <<"] called");
	    interface_enabled = true;
            b_result = true;
        }
    }
    else
    {
        ROS_INFO_STREAM("[" << __func__ <<"] Interface not yet supported");
    }

    return b_result;
}

/* Execute command (Direct mode) */
bool TmclInterpreter::execute_cmd(tmcl_msg_t *msg)
{
    bool b_result = false;

    uint8_t rx_msg[TMCL_MSG_SZ] = { 0 };
    uint32_t rx_msg_id = 0;
    uint8_t rx_msg_sz = 0;

    uint8_t tx_msg[TMCL_MSG_SZ] = { msg->cmd, msg->type, msg->motor, msg->value[0], msg->value[1], msg->value[2], msg->value[3] };

    if(interface_enabled)
    {
        if(tmcl_interface == TMCL_INTERFACE_CAN)
        {
            /* Send TMCL command */
            if(tmcl_cfg.socket_can->writeFrame(msg->tx_id, tx_msg, TMCL_MSG_SZ))
            {
                /* Wait for the reply for the TMCL command */
                auto start_time = std::chrono::system_clock::now();
                auto end_time = start_time;

                while(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() < timeout_ms)
                {
                    if(tmcl_cfg.socket_can->framesAvailable() && tmcl_cfg.socket_can->readFrame(&rx_msg_id, rx_msg, &rx_msg_sz))
                    {
                        /* A response for the TMCL command is received */
                        if((rx_msg_id == msg->rx_id) && 
                           (rx_msg_sz == TMCL_MSG_SZ) &&
                           (rx_msg[0] == msg->tx_id) &&
                           (rx_msg[2] == msg->cmd))
                        {
                            msg->sts = (tmcl_sts_t) rx_msg[1];
                            msg->value[0] = rx_msg[3];
                            msg->value[1] = rx_msg[4];
                            msg->value[2] = rx_msg[5];
                            msg->value[3] = rx_msg[6];
                            b_result = true;
	                    break;
                        }
                        else
                        {
                            ROS_INFO_STREAM("\nDifferent rx_id received"); 
                        }
                    }

                    end_time = std::chrono::system_clock::now();
                }
            }
        }
        else
        {
            ROS_WARN_STREAM("[" << __func__ <<"] Interface not yet supported");
        }
    }

    return b_result;
}

/* Shutdown interface */
bool TmclInterpreter::shutdown_interface()
{
    bool b_result = false;

    if(tmcl_interface == TMCL_INTERFACE_CAN)
    {
        tmcl_cfg.socket_can->deinitialize();
        delete tmcl_cfg.socket_can;
        tmcl_cfg.socket_can = nullptr;
        interface_enabled = false;
        b_result = true;
	}
    return b_result;
}
