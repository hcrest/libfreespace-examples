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

#include "math/quaternion.h"
#include "math/vec3.h"
#include "math/math.h"

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

    struct freespace_UserFrame user_;
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

static void getUserFrameFromInputThread(struct InputLoopState* state,
                                        struct freespace_UserFrame* user) {

    pthread_mutex_lock(&state->lock_);

    *user = state->user_;

    // Reset the accumulators.
    state->user_.deltaX = 0;
    state->user_.deltaY = 0;
    state->user_.deltaWheel = 0;

    pthread_mutex_unlock(&state->lock_);
}

static void getEulerAnglesFromUserFrame(struct freespace_UserFrame user, Vec3f* eulerAngles) {
    Quaternion q;
    q_quatFromUserFrame(&q, user);

    // The Freespace quaternion gives the rotation in terms of rotating the world around the object
    // We take the conjugate to get the rotation in the object's reference frame.
    q_conjugate(&q, q);

    // Normalize to get a unit quaternion
    q_normalize(&q, q);

    // Convert quaternion to Euler angles
    q_toEulerAngles(eulerAngles, q);
}

static void* inputThreadFunction(void* arg) {
    struct InputLoopState* state = (struct InputLoopState*) arg;
    struct freespace_DataMotionControl d;
    FreespaceDeviceId device;
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int numIds;
    int rc;

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
    exit(1);
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

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error flushing device: %d\n", rc);
        exit(1);
    }

    d.enableBodyMotion = 0;
    d.enableUserPosition = 1;
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
        struct freespace_UserFrame user;
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
        if (freespace_decodeUserFrame((int8_t*) buffer, length, &user) == FREESPACE_SUCCESS) {
            pthread_mutex_lock(&state->lock_);

            // Update state fields.
            state->user_.button1 = user.button1;
            state->user_.button2 = user.button2;
            state->user_.button3 = user.button3;
            state->user_.button4 = user.button4;
            state->user_.button5 = user.button5;

            state->user_.sequenceNumber = user.sequenceNumber;

            state->user_.linearPosX = user.linearPosX;
            state->user_.linearPosY = user.linearPosY;
            state->user_.linearPosZ = user.linearPosZ;

            state->user_.angularPosA = user.angularPosA;
            state->user_.angularPosB = user.angularPosB;
            state->user_.angularPosC = user.angularPosC;
            state->user_.angularPosD = user.angularPosD;

            // Update accumulation fields
            state->user_.deltaX += user.deltaX;
            state->user_.deltaY += user.deltaY;
            state->user_.deltaWheel += user.deltaWheel;

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
    struct freespace_UserFrame user;
    Vec3f eulerAngles;

    printVersionInfo(argv[0]);

    addControlHandler();

    initInputThread(&inputLoop);

    // Run the game loop
    while (!quit) {
        // Get input.
        getUserFrameFromInputThread(&inputLoop, &user);

        // Run game logic.
        getEulerAnglesFromUserFrame(user, &eulerAngles);

        // Render.
        printf("\r%d: Current Rotation = roll: %4f, pitch: %4f, yaw: %4f         ", user.sequenceNumber, RADIANS_TO_DEGREES(eulerAngles.x), RADIANS_TO_DEGREES(eulerAngles.y), RADIANS_TO_DEGREES(eulerAngles.z));
        fflush(stdout);

        // Wait for "vsync"
        usleep(16000);
    }


    stopInputThread(&inputLoop);

    return 0;
}
