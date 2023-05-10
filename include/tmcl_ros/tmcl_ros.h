/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_ROS_H
#define TMCL_ROS_H

#include "tmcl_interpreter.h"
#include <ros/ros.h>
#include <ros/console.h>

#define TXRX_ID_MAX 255
#define TX_ID_DEFAULT 1
#define RX_ID_DEFAULT 2
#define TIMEOUT_MS_MAX 5000
#define TIMEOUT_MS_DEFAULT 10
#define EXEC_CMD_RETRIES_MAX 3
#define EXEC_CMD_RETRIES_DEFAULT 1
#define PUB_RATE_MAX 100
#define RATE_20KBPS 20000
#define RATE_50KBPS 50000
#define RATE_100KBPS 100000
#define RATE_125KBPS 125000
#define RATE_250KBPS 250000
#define RATE_500KBPS 500000
#define RATE_1000KBPS 1000000


/* Conversion constants */

/* Derived from converting linear velocity (ROS velocity unit) to rpm (TMC board velocity unit)
   Where: 1 revolution = Circumference of the wheel (PI * diameter)
          1 minute     = 60 seconds
   Then:  rpm = (linear_vel * 60) / (PI * wheel_diameter)
*/
#define PI 3.1415926535
#define SECS_TO_MIN 60

/* Used for converting degrees (general position/angular unit) to steps (TMC board position/angular unit)
   Formula: unit = degrees * (motor esteps / 360)
*/
#define ANGULAR_FULL_ROTATION 360

class TmclROS
{
    public:
        /* Constructor */
        TmclROS();

        /* Destructor */
        ~TmclROS();

	/* Initialization */
        bool init(ros::NodeHandle nh);

	/* Get TMC information */
	bool getInfo();

	/* De-initialization */
        bool deInit();

	/* Execute a TMCL command within the program */
	bool exec_tmcl_cmd(tmcl_cmd_t cmd, uint8_t type, tmcl_motor_t motor, int32_t *val);

        /* Validate all the ROS parameters based on acceptable values */
        bool validate_params(ros::NodeHandle nh);
        bool validate_configurable_axis_params(ros::NodeHandle nh);
        bool validate_ap_cfg(ros::NodeHandle nh, std::string ap_cfg_str, uint8_t lut_idx);

	/* ROS parameters */
        std::string s_node_name;
        std::string s_namespace;
        int param_comm_interface;
        std::string param_comm_interface_name;
        int param_comm_bit_rate;
        int param_comm_tx_id;
        int param_comm_rx_id;
        int param_comm_timeout_ms;
        int param_comm_exec_cmd_retries;
        int param_total_motors;
        float param_wheel_diameter;
        std::vector<std::string> param_tmc_info_topic;
        std::vector<std::string> param_tmc_cmd_vel_topic;
        std::vector<std::string> param_tmc_cmd_abspos_topic;
        std::vector<std::string> param_tmc_cmd_relpos_topic;
        std::vector<std::string> param_tmc_cmd_trq_topic;
        std::vector<int> param_en_motors;
        std::vector<bool> param_en_pub_tmc_info;
        float param_pub_rate_tmc_info;
        std::vector<bool> param_pub_actual_vel;
        std::vector<bool> param_pub_actual_trq;
        std::vector<bool> param_pub_actual_pos;

        /* Configurations related to AP */
        bool param_follow_eeprom_cfg;
        std::vector<std::vector<int>> ap_cfg;
        std::vector<std::vector<int>> ap_cfg_ext;

        /* Separate vectors for registers of Statusflags */
        std::vector<std::string> param_status_flags_reg_name;
        std::vector<int> param_status_flags_reg_shift;

	/* Handle for TmclInterpreter */
        TmclInterpreter *tmcl;

    private:
        uint8_t n_retries;
};

#endif // TMCL_ROS_H
