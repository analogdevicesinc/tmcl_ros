/**
 * Copyright (c) 2022-2023 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <stdio.h>
#include <signal.h>
#include "tmcl_ros.h"

/* Globals */
TmclROS *p_tmc = nullptr; 
bool g_shutdown_signal = false;

/* Function Prototypes */
void graceful_shutdown();
void signal_callback_handler(int signum);

/* Locals */
void graceful_shutdown()
{
  ROS_INFO_STREAM("> Initiating graceful shutdown...");
  if(p_tmc->deInit())
  {
    ROS_INFO_STREAM("> Successfully de-initialized TMC.");
  }
  delete p_tmc;
  p_tmc = nullptr;

  ros::shutdown();
}

/**
 * Callback function for Ctrl-C handling
 */
void signal_callback_handler(int signum)
{
  ROS_INFO_STREAM("> Caught signal: " << signum << ". Terminating...");
  g_shutdown_signal = true;
}

/**
 * Main Program
 */
int main(int argc, char** argv)
{
#ifdef DEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  /* Initialize ROS */
  ros::init(argc, argv, "tmcl_ros_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  try
  {
    ROS_INFO_STREAM("> Starting tmcl_ros ...");
    signal(SIGINT, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);
    signal(SIGKILL, signal_callback_handler);
  
    ROS_INFO_STREAM("> Initializing tmcl_ros ...");
    p_tmc = new TmclROS(&nh);

    /* Initialize TMC */
    if(p_tmc->init())
    {
      ROS_INFO_STREAM("> Successfully initialized TMCL Protocol Interpreter.\n\n");
      while(!g_shutdown_signal && nh.ok() && !p_tmc->getRetriesExceededStatus())
      {
        ros::spinOnce();
      }
      throw-1; 
    }
    else
    {
      ROS_ERROR_STREAM("[ERROR] Initializing TMCL Protocol Interpreter failed!, restart node");
      throw-1; 
    }
  }
  catch(...)
  {
    ROS_ERROR_STREAM("[ERROR] Exception encountered. Exiting...");
    graceful_shutdown();
  }

  return 0;
}
