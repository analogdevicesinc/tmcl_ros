/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMCL_INTERPRETER_H
#define TMCL_INTERPRETER_H

#include <future>
#include <chrono>
#include <vector>

#include "socket_can_wrapper.h"

/** A TMCL Tx/Rx format will always be lke the following:
  * - Tx: | [[Module Address]] |     <Command>    |  <Type>  |  <Motor>  | <Value> | <Value> | <Value> | <Value> | [[Checksum]]
  * - Rx: | [[Reply  Address]] | <Module Address> | <Status> | <Command> | <Value> | <Value> | <Value> | <Value> | [[Checksum]]
  *
  * Specifically, for CAN interface:
  *  - Both Tx and Rx length will just be 7 bytes.
  *    Those enclosed with [[]] are not part of the message (they will automatically be added on the hardware-level).
  *  - Module Address: CAN-ID of the Sender
  *  - Reply Address: CAN-ID of the Receiver
  *  - Command, Type, Motor, Status can be either of the values enumerated below
  **/
#define TMCL_MSG_SZ        7
#define TMCL_MSG_VALUE_SZ  4

/*******************************************************************************/
/*                        Interface related settings                           */
/*******************************************************************************/

/* Supported interfaces */
typedef enum
{
    TMCL_INTERFACE_CAN = 0, /* 0 - CAN Interface */
    TMCL_INTERFACE_MAX,     /* This should not be used */
} tmcl_interface_t;

/* Supported interface's configurations */
/* Note: So far, these are all needed for CAN. Once other interfaces are added,
         convert this tmcl_cfg_t to a union then include different tmcl_cfg_<interface>_t as members. */
typedef struct 
{
    SocketCAN *socket_can;
    std::string interface_name;
    uint32_t bit_rate;
    uint16_t tx_id;
    uint16_t rx_id;
} tmcl_cfg_t;

/*******************************************************************************/
/*                      TMCL protocol related settings                         */
/*******************************************************************************/

/* Possible values for <Command> */ 
typedef enum
{
    TMCL_CMD_ROR = 1,       /*   1 - Rotate right */
    TMCL_CMD_ROL,           /*   2 - Rotate left */
    TMCL_CMD_MST,           /*   3 - Motor stop */
    TMCL_CMD_MVP,           /*   4 - Move to position */
    TMCL_CMD_SAP,           /*   5 - Set axis parameter */
    TMCL_CMD_GAP,           /*   6 - Get axis parameter */
    TMCL_CMD_STAP,          /*   7 - Store axis parameter */  
    TMCL_CMD_RSAP,          /*   8 - Restore axis parameter */
    TMCL_CMD_SGP,           /*   9 - Set global parameter */
    TMCL_CMD_GGP,           /*  10 - Get global parameter */
    TMCL_CMD_STGP,          /*  11 - Store global parameter */
    TMCL_CMD_RSGP,          /*  12 - Restore global parameter */
    TMCL_CMD_RFS,           /*  13 - Reference search */
    TMCL_CMD_SIO,           /*  14 - Set input output */
    TMCL_CMD_GIO,           /*  15 - Get input output */
    TMCL_CMD_CALC = 19,     /*  19 - Calculate (accumulator) */
    TMCL_CMD_COMP,          /*  20 - Compare */
    TMCL_CMD_JC,            /*  21 - Jump conditional */
    TMCL_CMD_JA,            /*  22 - Jump always */
    TMCL_CMD_CSUB,          /*  23 - Call subroutine */
    TMCL_CMD_RSUB,          /*  24 - Return from subroutine */
    TMCL_CMD_EI,            /*  25 - Enable interrupt */
    TMCL_CMD_DI,            /*  26 - Disable interrupt */
    TMCL_CMD_WAIT,          /*  27 - Wait for event */
    TMCL_CMD_STOP,          /*  28 - Stop (end of program) */
    TMCL_CMD_SCO = 30,      /*  30 - Set coordinate */
    TMCL_CMD_GCO,           /*  31 - Get coordinate */
    TMCL_CMD_CCO,           /*  32 - Capture coordinate */
    TMCL_CMD_CALCX,         /*  33 - Calculate X-register */
    TMCL_CMD_AAP,           /*  34 - Accumulator to axis parameter */
    TMCL_CMD_AGP,           /*  35 - Accumulator to global parameter */
    TMCL_CMD_CLE,           /*  36 - Clear error */
    TMCL_CMD_VECT,          /*  37 - Set interrupt vector */
    TMCL_CMD_RETI,          /*  38 - Return from interrupt */
    TMCL_CMD_ACO,           /*  39 - Accumulator to coordinate */
    TMCL_CMD_CALCVV,        /*  40 - Calculate (variable / variable) */
    TMCL_CMD_CALCVA,        /*  41 - Calculate (variable / accumulator) */
    TMCL_CMD_CALCAV,        /*  42 - Calculate (accumulator / variable) */
    TMCL_CMD_CALCVX,        /*  43 - Calculate (variable / X-reg) */
    TMCL_CMD_CALCXV,        /*  44 - Calculate (X-reg / variable) */
    TMCL_CMD_CALCV,         /*  45 - Calculate (variable) */
    TMCL_CMD_MVPA,          /*  46 - Move to position (accumulator) */
    TMCL_CMD_MVPXA,         /*  47 - Move to position (X-reg / accumulator) */
    TMCL_CMD_RST,           /*  48 - Restart */
    TMCL_CMD_DJNZ,          /*  49 - Decrement and jump if not zero */
    TMCL_CMD_ROLA,          /*  50 - Rotate left (accumulator) */
    TMCL_CMD_RORA,          /*  51 - Rotate right (accumulator) */
    TMCL_CMD_ROLXA,         /*  52 - Rotate left (X-reg / accumulator) */
    TMCL_CMD_RORXA,         /*  53 - Rotate right (X-reg / accumulator) */
    TMCL_CMD_MSTX,          /*  54 - Motor stop (X-reg) */
    TMCL_CMD_SIV,           /*  55 - Set indexed variable */
    TMCL_CMD_GIV,           /*  56 - Get indexed variable */
    TMCL_CMD_AIV,           /*  57 - Accumulator to indexed variable */
    TMCL_CMD_UF0 = 64,      /*  64 - User function #0 */
    TMCL_CMD_UF1,           /*  65 - User function #1 */
    TMCL_CMD_UF2,           /*  66 - User function #2 */
    TMCL_CMD_UF3,           /*  67 - User function #3 */
    TMCL_CMD_UF4,           /*  68 - User function #4 */
    TMCL_CMD_UF5,           /*  69 - User function #5 */
    TMCL_CMD_UF6,           /*  70 - User function #6 */
    TMCL_CMD_UF7,           /*  71 - User function #7 */
    TMCL_CMD_CALL,          /*  80 - Call subroutine unconditional */
    TMCL_CMD_APPSTOP = 128, /* 128 - Stop application */
    TMCL_CMD_APPRUN,        /* 129 - Run application */
    TMCL_CMD_APPSTEP,       /* 130 - Step application */
    TMCL_CMD_APPRESET,      /* 131 - Reset application */
    TMCL_CMD_APPENDL,       /* 132 - Enter download mode */
    TMCL_CMD_APPEXDL,       /* 133 - Exit download mode */
    TMCL_CMD_APPRPM,        /* 134 - Read program memory */
    TMCL_CMD_APPGSTS,       /* 135 - Get application status */
    TMCL_CMD_APPGFWV,       /* 136 - Get firmware version */
    TMCL_CMD_APPRFS,        /* 137 - Restore factory settings */
    TMCL_CMD_APPENAM = 139, /* 139 - Enter ASCII mode */
    TMCL_CMD_WC0 = 146,     /* 146 - Write channel register 0 */
    TMCL_CMD_WC1,           /* 147 - Write channel register 1 */
    TMCL_CMD_RC0,           /* 148 - Read channel register 0 */
    TMCL_CMD_RC1,           /* 149 - Read channel register 1 */
    TMCL_CMD_MAX            /* This should not be used */
} tmcl_cmd_t;

/* LUT row indeces of get-able/set-able Axis Parameters */
typedef enum
{
    ROW_IDX_SupplyVoltage = 0,
    ROW_IDX_TargetVelocity,
    ROW_IDX_ActualVelocity,
    ROW_IDX_TargetPosition,
    ROW_IDX_ActualPosition,
    ROW_IDX_TargetTorque,
    ROW_IDX_ActualTorque,
    ROW_IDX_CommutationMode,
    ROW_IDX_CommutationModeVelocity,
    ROW_IDX_CommutationModePosition,
    ROW_IDX_MaxTorque,
    ROW_IDX_OpenLoopCurrent,
    ROW_IDX_Acceleration,
    ROW_IDX_MotorPolePairs,
    ROW_IDX_PWMFrequency,
    ROW_IDX_HallSensorPolarity,
    ROW_IDX_HallSensorDirection,
    ROW_IDX_HallInterpolation,
    ROW_IDX_HallSensorOffset,
    ROW_IDX_EncoderDirection,
    ROW_IDX_EncoderSteps,
    ROW_IDX_EncoderInitMode,
    ROW_IDX_TorqueP,
    ROW_IDX_TorqueI,
    ROW_IDX_VelocityP,
    ROW_IDX_VelocityI,
    ROW_IDX_PositionP,
    ROW_IDX_BrakeChopperEnabled,
    ROW_IDX_BrakeChopperVoltage,
    ROW_IDX_BrakeChopperHysteresis,
    ROW_IDX_PositionScalerM,
    ROW_IDX_MAX          /* This should not be used */
} tmcl_lut_row_idx_t;

/* LUT row strings of get-able/set-able Axis Parameters */
std::vector<std::string> tmcl_lut_row_str = {
    "SupplyVoltage",
    "TargetVelocity",
    "ActualVelocity",
    "TargetPosition",
    "ActualPosition",
    "TargetTorque",
    "ActualTorque",
    "CommutationMode",
    "CommutationModeVelocity",
    "CommutationModePosition",
    "MaxTorque",
    "OpenLoopCurrent",
    "Acceleration",
    "MotorPolePairs",
    "PWMFrequency",
    "HallSensorPolarity",
    "HallSensorDirection",
    "HallInterpolation",
    "HallSensorOffset",
    "EncoderDirection",
    "EncoderSteps",
    "EncoderInitMode",
    "TorqueP",
    "TorqueI",
    "VelocityP",
    "VelocityI",
    "PositionP",
    "BrakeChopperEnabled",
    "BrakeChopperVoltage",
    "BrakeChopperHysteresis",
    "PositionScalerM",
    "ROW_STR_MAX"         /* This should not be used */
};

/* LUT column indeces of get-able/set-able Axis Parameters */
typedef enum
{
    COL_IDX_USED = 0,
    COL_IDX_AP,
    COL_IDX_VAL_DEFAULT,
    COL_IDX_VAL_MIN,
    COL_IDX_VAL_MAX,
    COL_IDX_MAX          /* This should not be used */
} tmcl_lut_col_idx_t;

/* Possible values for global parameter */
typedef enum
{
    TMCL_TYPE_GP_RXID = 70,
    TMCL_TYPE_GP_TXID = 71,
    TMCL_TYPE_GP_MAX     /* This should not be used */
}tmcl_type_gp_t;

/* Possible values for <Motor> */
typedef enum
{
    TMCL_MOTOR_0 = 0,
    TMCL_MOTOR_1,
    TMCL_MOTOR_2,
    TMCL_MOTOR_MAX          /* This should not be used */
} tmcl_motor_t;

/* Possible values for <Status> */
typedef enum
{
    TMCL_STS_ERR_CHKSUM = 1,
    TMCL_STS_ERR_CMD,
    TMCL_STS_ERR_TYPE,
    TMCL_STS_ERR_VAL,
    TMCL_STS_ERR_EEPROM_LCK,
    TMCL_STS_ERR_CMD_NA,
    TMCL_STS_ERR_NONE = 100,
    TMCL_STS_CMD_LOADED,
    TMCL_STS_MAX            /* This should not be used */
} tmcl_sts_t;

/* Definition for info needed to execute 1 TMCL command */
typedef struct
{
    uint16_t tx_id;                     /* Module Address */
    uint16_t rx_id;                     /* <Reply Address */
    tmcl_cmd_t cmd;                     /* <Command> */
    uint8_t type;                       /* <Type> */
    tmcl_motor_t motor;                 /* <Motor> */
    uint8_t value[TMCL_MSG_VALUE_SZ];   /* 4-byte value */
    tmcl_sts_t sts;
} tmcl_msg_t;

/* vel,pos,trq limits */
#define MIN_VEL -200000
#define MAX_VEL 200000
#define MIN_POS -2147483647
#define MAX_POS 2147483647
#define MIN_TRQ -15000
#define MAX_TRQ 15000

/* Class definition for TMCL Interpreter */
class TmclInterpreter 
{
    public:
        /* Constructor */
        TmclInterpreter();

        /* Destructor */
        ~TmclInterpreter();

	/* Reset interface */
        bool reset_interface();

	/* Execute command (Direct mode) */
        bool execute_cmd(tmcl_msg_t *msg);

	/* Shutdown interface */
        bool shutdown_interface();

        tmcl_interface_t tmcl_interface;
        tmcl_cfg_t tmcl_cfg;

        bool interface_enabled; 
        int timeout_ms;
};

#endif /* _TMCL_INTERPRETER_H */
