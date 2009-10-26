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

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include "appControlHandler.h"

struct freespace_BodyFrame cachedBodyFrame;

static void receiveCallback(FreespaceDeviceId id,
                            const uint8_t* buffer,
                            int length,
                            void* cookie,
                            int result) {
    if (result == FREESPACE_SUCCESS) {
        freespace_decodeBodyFrame(buffer, length, &cachedBodyFrame);
    }
}

static FreespaceDeviceId initializeFreespace() {
    uint8_t buffer[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct freespace_DataMotionControl d;
    FreespaceDeviceId device;
    int numIds;
    int rc;

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
	return 1;
    }

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("freespaceInputThread: Didn't find any devices.\n");
        exit(1);
    }

    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error opening device: %d\n", rc);
        exit(1);
    }
    freespace_setReceiveCallback(device, receiveCallback, NULL);

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error flushing device: %d\n", rc);
        exit(1);
    }

    memset(&cachedBodyFrame, 0, sizeof(cachedBodyFrame));

    d.enableBodyMotion = 1;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 1;
    d.enableMouseMovement = 0;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Could not send message: %d.\n", rc);
        }
    } else {
        printf("freespaceInputThread: Could not encode message.\n");
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/

    return device;
}

static void finalizeFreespace(FreespaceDeviceId device) {
    uint8_t buffer[FREESPACE_MAX_OUTPUT_MESSAGE_SIZE];
    struct freespace_DataMotionControl d;
    int rc;

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("\n\nfreespaceInputThread: Cleaning up...\n");
    d.enableBodyMotion = 0;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 1;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Could not send message: %d.\n", rc);
        }
    } else {
        printf("freespaceInputThread: Could not encode message.\n");
    }

    freespace_closeDevice(device);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/

    freespace_exit();
}

int main(int argc, char* argv[]) {
    struct freespace_BodyFrame body;
    FreespaceDeviceId device;

    printVersionInfo(argv[0]);

    memset(&body, 0, sizeof(struct freespace_BodyFrame));

    addControlHandler();
    device = initializeFreespace();

    // Run the game loop
    while (!quit) {
        // Get input.
        freespace_perform();

        body = cachedBodyFrame;

        // Run game logic.

        // Render.
        printf("\r%d: Current accel = %d, %d, %d          ",
               body.sequenceNumber,
               body.linearAccelX,
               body.linearAccelY,
               body.linearAccelZ);
        fflush(stdout);

        // Wait for "vsync"
        usleep(16000);
    }

    finalizeFreespace(device);
    return 0;
}
