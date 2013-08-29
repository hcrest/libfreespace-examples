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

#ifdef _WIN32
#include "win32/stdafx.h"
#else
#include <stdio.h>
#include <stdlib.h>
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"
#include <string.h>

// Cross platform sleep macro
#ifdef _WIN32
#define SLEEP    Sleep(200)
#else
#include <unistd.h>
#define SLEEP    sleep(1)
#endif

// Limit on how many times to try to get a response
#define RETRY_COUNT_LIMIT 100

/**
 * main
 * This example uses the synchronous API to 
 *  - find a device
 *  - open the device found
 *  - send a message
 *  - look for a response
 * This example assumes that the device is already connected.
 */
int main(int argc, char* argv[]) {
    FreespaceDeviceId device;                       // Keep track of the device you are talking to
    struct freespace_message send;                  // A place to create messages to send to the device
    struct freespace_message receive;               // A place to put a message received from the device
    int numIds;                                     // Keep track of how many devices are available
    int rc;                                         // Return Code
    int retryCount = 0;                             // How many times tried so far to get a response

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

    printf("Scanning for Freespace devices...\n");
    // Get the ID of the first device in the list of availble devices
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("Didn't find any devices.\n");
        return 1;
    }

    printf("Found a device. Trying to open it...\n");
    // Prepare to communicate with the device found above
    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error opening device: %d\n", rc);
        return 1;
    }

    // Display the device information.
    printDeviceInfo(device);

    // Make sure any old messages are cleared out of the system
    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error flushing device: %d\n", rc);
        return 1;
    }

    printf("Requesting battery level messages.\n");

    memset(&send, 0, sizeof(send)); // Start with a clean message struct
    // Populate the message fields. Two options are shown below. Uncomment one desired
    // and comment out the one not desired.
    //send.messageType = FREESPACE_MESSAGE_BATTERYLEVELREQUEST; // To send a battery level request
    send.messageType = FREESPACE_MESSAGE_PRODUCTIDREQUEST;    // To send a product ID request

    while (!quit) {
        if (retryCount < RETRY_COUNT_LIMIT) {
            retryCount++;
            // Send the message constructed above.
            rc = freespace_sendMessage(device, &send);
            if (rc != FREESPACE_SUCCESS) {
                printf("Could not send message: %d.\n", rc);
            }

            // Read the response message.
            rc = freespace_readMessage(device, &receive, 100);
            if (rc == FREESPACE_SUCCESS) {
                // Print the received message
                freespace_printMessage(stdout, &receive);
                retryCount = 0;
            } else if (rc == FREESPACE_ERROR_TIMEOUT) {
                printf("<timeout>  Try moving the Freespace device to wake it up.\n");
            } else if (rc == FREESPACE_ERROR_INTERRUPTED) {
                printf("<interrupted>\n");
            } else {
                printf("Error reading: %d. Quitting...\n", rc);
                break;
            }
        } else {
            printf("Did not receive response after %d trials\n", RETRY_COUNT_LIMIT);
            quit = 1;
        }
        SLEEP;
    }

    printf("Cleaning up...\n");
    freespace_closeDevice(device);

    freespace_exit();

    return 0;
}
