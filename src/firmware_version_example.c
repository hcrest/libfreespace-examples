/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2013, Hillcrest Laboratories, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the Hillcrest Laboratories, Inc. nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef WIN32
#include "win32/stdafx.h"
#include <windows.h>
#else
#include <stdlib.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

#include <string.h>

#define BUFFER_LENGTH 1024

// Cross platform sleep macro
#ifdef _WIN32
#define SLEEP    Sleep(100)
#else
#define SLEEP    sleep(1)
#endif

/**
 * Callback that handles product ID response messages received from devices
 * @param id the device the message is from
 * @param message a pointer to the message to send
 * @param cookie not used in this example
 * @param result FREESPACE_SUCCESS if a packet was received; else error code
 */
static void receiveMessageCallback(FreespaceDeviceId id,
                                   struct freespace_message* message,
                                   void* cookie,
                                   int result) {
                                       
    struct freespace_ProductIDResponse* pr = &(message->productIDResponse);
    
    if (result == FREESPACE_SUCCESS && message != NULL) {
        if (message->messageType == FREESPACE_MESSAGE_PRODUCTIDRESPONSE) {
            printf("Received product ID response message from device ID: %d\n", id);
            if (pr->deviceClass == 1) {
                printf("    Device class:   dongle\n");
            } else if (pr->deviceClass == 2) {
                printf("    Device class:   handheld\n");
            } else {
                printf("    Device class:   unknown\n");
            }
            printf("    Part Number:    %d\n", pr->swPartNumber);
            printf("    Build Number:   %d\n", pr->swBuildNumber);
            printf("    Serial Number:  %d\n", pr->serialNumber);
            printf("    Version Number: %d.%d.%d\n", pr->swVersionMajor, pr->swVersionMinor , pr->swVersionPatch);
        }
    } else if (result == FREESPACE_ERROR_NO_DATA) {
        printf("Message with no data received from device ID %d.\n", id);
    } else {
        printf("Problem with message received from device ID %d.\n", id);
    }
}


/**
 * Callback that 
 *  - displays the devices that have been inserted into and removed from the system.
 *  - requests the firmware version of any devices that have been inserted into the system.
 * Implements freespace_hotplugCallback
 * @param event The type of event.
 * @param id The affected device.
 * @param cookie Not used for this application.
 */
void hotplugCallback(enum freespace_hotplugEvent event,
                     FreespaceDeviceId id,
                     void* cookie) {
    int rc;
    struct freespace_message message;

    switch (event) {

        case FREESPACE_HOTPLUG_INSERTION:
            // Get and print USB HID information about the device.
            printf("Device Inserted: %d\n", id);
            rc = printDeviceInfo(id);
            if (rc != FREESPACE_SUCCESS) {
                printf("Could not display device info: %d\n", rc);
                return;
            }

            // Open the device to be able to communicate with it
            rc = freespace_openDevice(id);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error opening device.\n");
                return;
            }

            // Set the handler that handles messages received from the device.
            freespace_setReceiveMessageCallback(id, receiveMessageCallback, NULL);

            // Create and send a product ID request message to the device
            memset(&message, 0, sizeof(message)); // Start with a clean message struct
            message.messageType = FREESPACE_MESSAGE_PRODUCTIDREQUEST;
            // Allow message.dest to have the default value of 0. This will cause the message
            // to go to the remote or the module
            rc = freespace_sendMessageAsync(id, &message, 100, NULL, 0);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error sending productID request\n");
                return;
            }
            // To communicate with a dongle set the dest field to 1
            message.dest = 1;
            rc = freespace_sendMessageAsync(id, &message, 100, NULL, 0);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error sending productID request\n");
                return;
            }

            break;

        case FREESPACE_HOTPLUG_REMOVAL:
            printf("Device Removed: %d\n", id);
            break;

        default:
            printf("Unrecognized freespace_hotplugEvent.\n");
            break;
    }
}

/**
 * main
 * This example uses the asynchronous API to
 *  - detect removal or additon of devices
 *  - get the product ID information from added devices
 * Devices may or may not be connected when this example is launched
 * Devices may be add/removed (pugged/unplugged) while this example is running.
 */
int main(int argc, char* argv[]) {
    int numIds;
    int deviceIds[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int rc;
    
    // Flag to indicate that the application should quit
    // Set by the control signal handler
    int quit = 0;

    printVersionInfo(argv[0]);

    addControlHandler(&quit);

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
        return 1;
    }

    // Set the callback to catch the initial devices.
    printf("Detecting the Freespace devices already connected...\n");
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);
    freespace_perform();

    printf("Waiting for Freespace devices to be inserted...\n");
    printf("Type Ctrl-C to exit\n");
    while (!quit) {
        // Easy event loop - just poll freespace_perform periodically
        // rather than waiting on select or WaitForMultipleObjects
        SLEEP;

        // Callbacks are called from within the perform call.
        freespace_perform();
    }

    printf("Exiting\n");
    printf("Cleaning up all devices...\n");
    rc = freespace_getDeviceList(deviceIds, FREESPACE_MAXIMUM_DEVICE_COUNT, &numIds);
    if (rc == FREESPACE_SUCCESS) {
        int i;
        for (i = 0; i < numIds; i++) {
            freespace_closeDevice(deviceIds[i]);
        }
    } else {
        printf("Error getting device list.\n");
    }

    freespace_exit();

    return 0;
}
