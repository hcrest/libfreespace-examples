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

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include "appControlHandler.h"

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

struct InputLoopState {
    pthread_t thread_;
    pthread_mutex_t lock_;
    int quit_;
    int initialized_;

    struct freespace_BodyFrame body_;
};

static void* inputThreadFunction(void*);

static void initInputThread(struct InputLoopState* state) {
    memset(state, 0, sizeof(struct InputLoopState));

    pthread_mutex_init(&state->lock_, NULL);

    // Start the input thread
    pthread_create(&state->thread_, NULL, inputThreadFunction, state);
}

static void stopInputThread(struct InputLoopState* state) {
    // Signal the thread to stop and wait until it does.
    state->quit_ = 1;
    pthread_join(state->thread_, NULL);
    pthread_mutex_destroy(&state->lock_);
}

static void getBodyFrameFromInputThread(struct InputLoopState* state,
                                        struct freespace_BodyFrame* body) {

    pthread_mutex_lock(&state->lock_);

    *body = state->body_;

    // Reset the accumulators.
    state->body_.deltaX = 0;
    state->body_.deltaY = 0;
    state->body_.deltaWheel = 0;

    pthread_mutex_unlock(&state->lock_);
}

static void* inputThreadFunction(void* arg) {
    struct InputLoopState* state = (struct InputLoopState*) arg;
    struct freespace_DataMotionControl d;
    FreespaceDeviceId device;
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int numIds;
    int rc;

    // Initialize the freespace library
    freespace_init();

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

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error flushing device: %d\n", rc);
        exit(1);
    }

    d.enableBodyMotion = 1;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 1;
    d.enableMouseMovement = 0;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Could not send message: %d.\n", rc);
        }
    } else {
        printf("freespaceInputThread: Could not encode message.\n");
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/

    state->initialized_ = 1;
    while (!state->quit_) {
        struct freespace_BodyFrame body;
        int length;

        rc = freespace_read(device, buffer, sizeof(buffer), 1000 /* 1 second timeout */, &length);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Error reading: %d. Trying again after a second...\n", rc);
            sleep(1);
            continue;
        }

        // Check if this is a body frame message.
        if (freespace_decodeBodyFrame((int8_t*) buffer, length, &body) == FREESPACE_SUCCESS) {
            pthread_mutex_lock(&state->lock_);

            // Update state fields.
            state->body_.button1 = body.button1;
            state->body_.button2 = body.button2;
            state->body_.button3 = body.button3;
            state->body_.button4 = body.button4;
            state->body_.button5 = body.button5;

            state->body_.sequenceNumber = body.sequenceNumber;

            state->body_.linearAccelX = body.linearAccelX;
            state->body_.linearAccelY = body.linearAccelY;
            state->body_.linearAccelZ = body.linearAccelZ;

            state->body_.angularVelX = body.angularVelX;
            state->body_.angularVelY = body.angularVelY;
            state->body_.angularVelZ = body.angularVelZ;

            // Update accumulation fields
            state->body_.deltaX += body.deltaX;
            state->body_.deltaY += body.deltaY;
            state->body_.deltaWheel += body.deltaWheel;

            pthread_mutex_unlock(&state->lock_);
        }
    }

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("\n\nfreespaceInputThread: Cleaning up...\n");
    d.enableBodyMotion = 0;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 1;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
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

    // Exit the thread.
    return 0;
}


int main(int argc, char* argv[]) {
    struct InputLoopState inputLoop;
    struct freespace_BodyFrame body;

    printVersionInfo(argv[0]);

    addControlHandler();

    initInputThread(&inputLoop);

    // Run the game loop
    while (!quit) {
        // Get input.
        getBodyFrameFromInputThread(&inputLoop, &body);

        // Run game logic.

        // Render.
        printf("\r%d: Current accel = %d, %d, %d          ", body.sequenceNumber, body.linearAccelX, body.linearAccelY, body.linearAccelZ);
        fflush(stdout);

        // Wait for "vsync"
        usleep(16000);
    }


    stopInputThread(&inputLoop);

    return 0;
}
