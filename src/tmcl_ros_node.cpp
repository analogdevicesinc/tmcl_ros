/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <stdio.h>
#include <signal.h>
#include "tmcl_ros.h"
#include "tmcl_ros_subscriber.h"
#include "tmcl_ros_publisher.h"
#include "tmcl_ros_service.h"

// /* Globals */
TmclROS *p_tmc = nullptr; // Needed for signal_callback_handler()
TmclRosSubscriber *p_tmcs = nullptr;
TmclRosPublisher *p_tmcp = nullptr;
TmclRosService *p_tmcv = nullptr;

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

    delete p_tmcp;
    p_tmcp = nullptr;
    delete p_tmcs;
    p_tmcs = nullptr;
    delete p_tmcv;
    p_tmcv = nullptr;
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
    graceful_shutdown();
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
        if (argc == 2)
        {
            std::string arg_mode;
            arg_mode.assign(argv[1]);
        
            ROS_INFO_STREAM("> Starting tmcl_ros ...");
            signal(SIGINT, signal_callback_handler);

            ROS_INFO_STREAM("> Initializing tmcl_ros ...");
            p_tmc = new TmclROS();

            /* Initialize TMC */
            if(p_tmc->init(nh))
            {
                ROS_INFO_STREAM("> Successfully initialized TMCL Protocol Interpreter.\n\n");

                if(arg_mode == "normal")
                {   
                    p_tmcs = new TmclRosSubscriber(nh, p_tmc);
                    p_tmcp = new TmclRosPublisher(nh, p_tmc);
                    p_tmcv = new TmclRosService(nh, p_tmc);
                    ros::Rate loop_rate(p_tmc->param_pub_rate_tmc_info);

                    ROS_INFO_STREAM("#######################################################");
                    for(int i = 0; i < p_tmc->param_total_motors; i++)
                    {
                        if(p_tmc->param_en_motors[i])
                        {
                            ROS_INFO_STREAM("ROS Subscriber callback for Motor" << i << " active (subscribed on topic: " 
                            << p_tmc->param_tmc_cmd_vel_topic[i] << ", " << p_tmc->param_tmc_cmd_abspos_topic[i] << ", " << p_tmc->param_tmc_cmd_relpos_topic[i] << ", " << p_tmc->param_tmc_cmd_trq_topic[i] << ")");

                            ROS_INFO_STREAM("ROS Publisher topics for Motor" << i <<" active (publishing on topic: " << p_tmc->param_tmc_info_topic[i] << ")");
                        }
                    }
                    ROS_INFO_STREAM("ROS Service for accepting TMC commands active (service: " << p_tmcv->s_service_name << ")");
                    ROS_INFO_STREAM("#######################################################\n\n");

                    while (nh.ok())
                    {
                    /* Get TMC information */
                        if(p_tmc->getInfo())
                        {
                        /* Publish TMC information */
                            if(p_tmcp->rosPublishTmcInfo(p_tmc))
                            {
                                ROS_DEBUG_STREAM("[SUCCESS] Publishing TMCL info OK!");
                            }
                            else
                            {
                                ROS_ERROR_STREAM("[ERROR] Publishing TMCL info failed!");
                            }
                        }
                        else
                        {
                            ROS_ERROR_STREAM("[ERROR] getInfo() failed!, restart node");
                            throw-1;
                        }

                        /* Spin depending on loop rate */
                        ros::spinOnce();
                        loop_rate.sleep();
                    }

                    /* Clean-up */
                    graceful_shutdown();
                }
                else if (arg_mode == "service")
                {
                    /* Setup pointers for signal handler to use */
                    p_tmcv = new TmclRosService(nh, p_tmc);
                    ROS_INFO_STREAM("\nService Mode only. Waiting for Service...");
                    ros::spin();
                }
                else
                {
                    ROS_ERROR_STREAM("Mode not recognized.");
                    throw-1; 
                }
            }
            else
            {
                ROS_ERROR_STREAM("[ERROR] Initializing TMCL Protocol Interpreter failed!, restart node");
                throw-1; 
            } 
        }
        else
        {
            ROS_ERROR_STREAM("Invalid argument count.");
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
