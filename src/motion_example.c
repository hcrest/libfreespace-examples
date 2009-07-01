/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009, Hillcrest Laboratories, Inc.
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
#include "stdafx.h"
#include <stdio.h>
#else
#include <stdlib.h>
#include <signal.h>
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"


int main(int argc, char* argv[]) {
    struct freespace_DataMotionControl d;
    FreespaceDeviceId device;
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int numIds;
    int rc;

    printVersionInfo(argv[0]);

    addControlHandler();

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
	return 1;
    }

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    printf("Scanning for Freespace devices...\n");
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("Didn't find any devices.\n");
        return 1;
    }

    printf("Found a device. Trying to open it...\n");
    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error opening device: %d\n", rc);
        return 1;
    }

    // Display the device information.
    printDeviceInfo(device);

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error flushing device: %d\n", rc);
        return 1;
    }

    printf("Sending message to enable body-frame motion data.\n");
    d.enableBodyMotion = 1;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 0;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        }
    } else {
        printf("Could not encode message.\n");
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/


    printf("Listening for messages.\n");
    while (!quit) {
        int length;

        rc = freespace_read(device, buffer, sizeof(buffer), 1000, &length);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            // Both timeout and interrupted are ok.
            // Timeout happens if there aren't any events for a second.
            // Interrupted happens if you type CTRL-C or if you
            // type CTRL-Z and background the app on Linux.
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("Error reading: %d. Quitting...\n", rc);
            break;
        }

        freespace_printMessage(stdout, buffer, length);
    }

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("Sending message to enable mouse motion data.\n");
    d.enableBodyMotion = 0;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 1;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        }
    } else {
        printf("Could not encode message.\n");
    }

    printf("Cleaning up...\n");
    freespace_closeDevice(device);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/

    freespace_exit();

    return 0;
}
