/**
 * Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef SOCKET_CAN_WRAPPER_H
#define SOCKET_CAN_WRAPPER_H

#include <string>

class SocketCAN
{
    public:
        /* Constructor */
        SocketCAN();

        /* Destructor */
        ~SocketCAN();

	/* Initialize CAN with specified interface name */
        bool initialize(const char *interface_name, uint32_t bit_rate);

	/* De-initialize CAN */
        void deinitialize();

	/* Check if CAN frames are available */
        bool framesAvailable();

	/* Rx CAN frames */
        bool readFrame(uint32_t *id, uint8_t *data, uint8_t *size);

	/* Tx CAN frames */
        bool writeFrame(uint32_t id, uint8_t *data, uint8_t size);

	/* Get interface name */
        std::string* getInterfaceName();

    protected:
        int can_socket;
        std::string interface_name;
};

#endif // SOCKET_CAN_WRAPPER_H
