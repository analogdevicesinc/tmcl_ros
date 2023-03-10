/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <ros/console.h>

#include "socket_can_wrapper.h"

////////////////////////////////////////////////////////////////////////////////

SocketCAN::SocketCAN()
{
    can_socket = -1;
    interface_name = ""; 
}

SocketCAN::~SocketCAN()
{
    
}

////////////////////////////////////////////////////////////////////////////////

bool SocketCAN::initialize(const char *interface_name, uint32_t bit_rate)
{
    bool b_result = false;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int flags;

    ROS_INFO_STREAM("[" << __func__ << "] called");

#ifdef DUMMYMODE
    ROS_INFO_STREAM("DUMMY " << __func__ << ", always TRUE");
    b_result = true;
#else
    if(can_socket == -1)
    {
        this->interface_name = interface_name;

        /* Create the socket */
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket != -1)
	{

            /* Get the interface index by interface name */
            addr.can_family = AF_CAN;
            strcpy(ifr.ifr_name, interface_name);
            if(ioctl(can_socket, SIOCGIFINDEX, &ifr) != -1)
            {
                addr.can_ifindex = ifr.ifr_ifindex;

                /* Set the non-blocking socket flag */
                flags = fcntl(can_socket, F_GETFL, 0);
                if(flags != -1)
                {
                    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);

                    /* Bind the socket */
                    if(bind(can_socket, (struct sockaddr *) &addr, sizeof(addr)) == 0)
                    {
                        b_result = true;
                    }
		    else
   		    {
                        ROS_ERROR_STREAM("[" << __func__ << "] Binding the CAN socket failed");
                        close(can_socket);
                        can_socket = -1;
	            }
		}
		else
		{
                    ROS_ERROR_STREAM("[" << __func__ << "] Setting F_GETFL flag failed");
                    close(can_socket);
                    can_socket = -1;
		}
	    }
	    else
	    {
                ROS_ERROR_STREAM("[" << __func__ << "] Getting the interface index failed");
                close(can_socket);
                can_socket = -1;
	    }
	}
	else
	{
            ROS_ERROR_STREAM("[" << __func__ << "] Creating a CAN socket failed");
	}
    }
    else
    {
        ROS_ERROR_STREAM("[" << __func__ << "] There is an opened CAN socket already");
    }
#endif

    return b_result;
}

void SocketCAN::deinitialize()
{
    ROS_INFO_STREAM("[" << __func__ << "] called");

#ifdef DUMMYMODE
    ROS_INFO_STREAM("DUMMY " << __func__ << ", always TRUE");
#else
    if (can_socket != -1)
    {
        close(can_socket);
        can_socket = -1;
    }
#endif

    return;
}

bool SocketCAN::framesAvailable()
{
    bool b_result = false;

    //ROS_INFO_STREAM("[" << __func__ << "] called");
 
#ifdef DUMMYMODE
    //ROS_INFO_STREAM("DUMMY " << __func__ << ", always TRUE");
    b_result = true;
#else

    if (can_socket != -1)
    {
        struct pollfd pfd;

        pfd.fd = can_socket;
        pfd.revents = 0;
        pfd.events = POLLIN;

        /* Poll the CAN socket for incoming data without a timeout */
        if (poll(&pfd, 1, 0) == 1)
	{
            /* Ignore all events except for incoming data */
            if (pfd.revents == POLLIN)
	    {
                b_result = true;
	    }
	}
    }
    else
    {
        ROS_INFO_STREAM("[" << __func__ << "] Error: No opened CAN socket");
    }
#endif

    return b_result;
}

bool SocketCAN::readFrame(uint32_t *id, uint8_t *data, uint8_t *size)
{
    bool b_result = false;

    //ROS_INFO_STREAM("[" << __func__ << "] called");

#ifdef DUMMYMODE
    //ROS_INFO_STREAM("DUMMY " << __func__ << ", always TRUE");
    b_result = true;
#else

    if(can_socket != -1)
    {
        can_frame frame;

        ssize_t count = read(can_socket, &frame, sizeof(can_frame));
        if (count == sizeof(can_frame))
        {
            *id = frame.can_id;
            *size = frame.can_dlc;
            memcpy(data, frame.data, frame.can_dlc);
            b_result = true;
        }
	else
	{
            ROS_INFO_STREAM("[" << __func__ << "] Error: Incorrect size of CAN frame (size = " << count << ")");
	}
    }
    else
    {
        ROS_INFO_STREAM("[" << __func__ << "] Error: No opened CAN socket");
    }
#endif
    return b_result;
}

bool SocketCAN::writeFrame(uint32_t id, uint8_t *data, uint8_t size)
{
    bool b_result = false;

    //ROS_INFO_STREAM("[" << __func__ << "] called");

#ifdef DUMMYMODE
    //ROS_INFO_STREAM("DUMMY " << __func__ << ", always TRUE");
    b_result = true;
#else

    if (can_socket != -1)
    {
        can_frame frame;

        frame.can_id = id;
        frame.can_dlc = size;
        memcpy(frame.data, data, frame.can_dlc);

        ssize_t count = write(can_socket, &frame, sizeof(can_frame));

        if (count == sizeof(can_frame))
	{
	    b_result = true;
	}
	else
	{
            ROS_INFO_STREAM("[" << __func__ << "] Error: Incorrect size of CAN frame (size = " << count << ")");
	}
    }
    else
    {
        ROS_INFO_STREAM("[" << __func__ << "] Error: No opened CAN socket");
    }
#endif

    return b_result;
}

std::string* SocketCAN::getInterfaceName()
{
    ROS_INFO_STREAM("[" << __func__ << "] called");

    return &interface_name;
}

