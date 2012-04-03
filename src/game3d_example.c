/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2012, Hillcrest Laboratories, Inc.
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
#include "appControlHandler.h"

#include <math.h>
#include "math/quaternion.h"
#include "math/vec3.h"

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>


#ifdef WIN32
#include <windows.h>
#include "win32/pthread_win32.h"

#define M_PI    3.141592654
#else
#include <unistd.h>
#include <pthread.h>
#endif

#define RADIANS_TO_DEGREES(rad) ((float) rad * (float) (180.0 / M_PI))
#define DEGREES_TO_RADIANS(deg) ((float) deg * (float) (M_PI / 180.0))

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

static void getEulerAnglesFromUserFrame(const struct freespace_UserFrame* user,
                                        struct Vec3f* eulerAngles) {
    struct Quaternion q;
    q_quatFromUserFrame(&q, user);

    // The Freespace quaternion gives the rotation in terms of
    // rotating the world around the object. We take the conjugate to
    // get the rotation in the object's reference frame.
    q_conjugate(&q, &q);

    // Convert quaternion to Euler angles
    q_toEulerAngles(eulerAngles, &q);
}

static void* inputThreadFunction(void* arg) {
    struct InputLoopState* state = (struct InputLoopState*) arg;
    struct freespace_message message;
    FreespaceDeviceId device;
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

    memset(&message, 0, sizeof(message));
    if (FREESPACE_SUCCESS == freespace_isNewDevice(device)) {
        message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
        message.dataModeControlV2Request.packetSelect = 3;
        message.dataModeControlV2Request.modeAndStatus |= 0 << 1;
    } else {
        message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
        message.dataModeRequest.enableUserPosition = 1;
        message.dataModeRequest.inhibitPowerManager = 1;
    }
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/

    state->initialized_ = 1;
    while (!state->quit_) {
        rc = freespace_readMessage(device, &message, 1000 /* 1 second timeout */);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Error reading: %d. Trying again after a second...\n", rc);
#ifdef WIN32
            Sleep(1);
#else
            sleep(1);
#endif
            continue;
        }

        // Check if this is a user frame message.
        if (message.messageType == FREESPACE_MESSAGE_USERFRAME) {
            pthread_mutex_lock(&state->lock_);

            // Update state fields.
            state->user_.button1 = message.userFrame.button1;
            state->user_.button2 = message.userFrame.button2;
            state->user_.button3 = message.userFrame.button3;
            state->user_.button4 = message.userFrame.button4;
            state->user_.button5 = message.userFrame.button5;

            state->user_.sequenceNumber = message.userFrame.sequenceNumber;

            state->user_.linearPosX = message.userFrame.linearPosX;
            state->user_.linearPosY = message.userFrame.linearPosY;
            state->user_.linearPosZ = message.userFrame.linearPosZ;

            state->user_.angularPosA = message.userFrame.angularPosA;
            state->user_.angularPosB = message.userFrame.angularPosB;
            state->user_.angularPosC = message.userFrame.angularPosC;
            state->user_.angularPosD = message.userFrame.angularPosD;

            // Update accumulation fields
            state->user_.deltaX += message.userFrame.deltaX;
            state->user_.deltaY += message.userFrame.deltaY;
            state->user_.deltaWheel += message.userFrame.deltaWheel;

            pthread_mutex_unlock(&state->lock_);
        }
    }

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("\n\nfreespaceInputThread: Cleaning up...\n");
    memset(&message, 0, sizeof(message));
    if (FREESPACE_SUCCESS == freespace_isNewDevice(device)) {
        message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
        message.dataModeControlV2Request.packetSelect = 1;
    } else {
        message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
        message.dataModeRequest.enableMouseMovement = 1;
    }
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
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
    struct Vec3f eulerAngles;

    printVersionInfo(argv[0]);

    addControlHandler();

    initInputThread(&inputLoop);

    // Run the game loop
    while (!quit) {
        // Get input.
        getUserFrameFromInputThread(&inputLoop, &user);

        // Run game logic.
        getEulerAnglesFromUserFrame(&user, &eulerAngles);

        // Render.
        printf("\r%d: roll: %0.4f, pitch: %0.4f, yaw: %0.4f         ",
               user.sequenceNumber,
               RADIANS_TO_DEGREES(eulerAngles.x),
               RADIANS_TO_DEGREES(eulerAngles.y),
               RADIANS_TO_DEGREES(eulerAngles.z));
        fflush(stdout);

        // Wait for "vsync"
#ifdef WIN32
        Sleep(1);
#else
        usleep(16000);
#endif
    }


    stopInputThread(&inputLoop);

    return 0;
}
