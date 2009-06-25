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

#ifdef WIN32
#include "stdafx.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#else
#define _GNU_SOURCE // for ppoll
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

struct DeviceData {
    FreespaceDeviceId id;

    int msgReadCurrent;
    int msgReadLast;
    int msgReadDelta;
    int msgReadError;

    int msgSendCurrent;
    int msgSendLast;
    int msgSendDelta;
    int msgSendError;

    int sequenceNumber;
    int lostPackets;
};

#define MAX_NUMBER_DEVICES 50
struct DeviceData devices[MAX_NUMBER_DEVICES];

#ifdef WIN32
static struct _timeb lastTime;
static struct _timeb nextTime;

/**
 * Initialize the 1 second timer.
 */
void timer_initialize() {
    _ftime_s( &lastTime );
}

/**
 * Determine if the timer period has elapsed.
 * @return true if 1 second or more has elapsed, false otherwise.
 */
int timer_step() {
    int retVal;
    _ftime_s( &nextTime );
    retVal = ((nextTime.time - lastTime.time) * 1000 + nextTime.millitm - lastTime.millitm > 1000);
    if (retVal) {
        lastTime.time++;
    }
    return retVal;
}
#else
#include <sys/time.h>
#include <unistd.h>

static time_t lastTime;
void timer_initialize() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    lastTime = tv.tv_sec;
}

int timer_step() {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    if (tv.tv_sec != lastTime) {
        lastTime = tv.tv_sec;
        return 1;
    } else {
        return 0;
    }
}
#endif

static int useBodyFrame = 1;
static int sendMessageEachLoop = 0;

static int isSDAReport(const char* buffer, int len) {
    return len > 2 && buffer[0] == 0x08 && buffer[1] == 0x01;
}

static void receiveCallback(FreespaceDeviceId id,
                            const char* buffer,
                            int length,
                            void* cookie,
                            int result) {
    int idx;
    int isProcessed = 0;
    struct freespace_message s;
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id == id) {
            isProcessed = 1;
            devices[idx].msgReadCurrent++;
            if (result != FREESPACE_SUCCESS) {
                devices[idx].msgReadError++;
            }
	    freespace_decode_message((const int8_t*) buffer, length, &s);
	    if (useBodyFrame && s.messageType == FREESPACE_MESSAGE_BODYFRAME) {	          
	        if (devices[idx].sequenceNumber + 1 != s.bodyFrame.sequenceNumber && devices[idx].sequenceNumber != 0) {
		    devices[idx].lostPackets = s.bodyFrame.sequenceNumber - devices[idx].sequenceNumber;
		}
		devices[idx].sequenceNumber = s.bodyFrame.sequenceNumber;
	    } else if (!useBodyFrame && isSDAReport(buffer, length)) {
	        ++devices[idx].sequenceNumber;
	    } else if (devices[idx].sequenceNumber == 0) {
	        freespace_printMessage(stdout, buffer, length);
	    }
	    break;
        }
    }
    if (isProcessed == 0) {
        printf("ERROR: could not find read device %d\n", id);
    }
}

static void sendCallback(FreespaceDeviceId id,
                         void* cookie,
                         int result) {
    int idx;
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id == id) {
            devices[idx].msgSendCurrent++;
            if (result != FREESPACE_SUCCESS) {
                devices[idx].msgSendError++;
            }
            break;
        }
    }
    if (idx >= MAX_NUMBER_DEVICES) {
        printf("ERROR: could not find send device %d\n", id);
    }
}

static void initDevice(FreespaceDeviceId id) {
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    struct freespace_DataMotionControl d;
    int rc;
    int idx;

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    freespace_setReceiveCallback(id, receiveCallback, NULL);

    rc = freespace_openDevice(id);
    if (rc != 0) {
        printf("Error opening device.\n");
        return;
    }

    // Display the device information.
    //printDeviceInfo(id);
    rc = freespace_flush(id);
    if (rc != 0) {
        printf("Error flushing device.\n");
        return;
    }

    // Add the device to our list
    rc = 0;
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id < 0) {
            rc = 1;
            devices[idx].id = id;
            devices[idx].msgReadCurrent = 0;
            devices[idx].msgReadLast = 0;
            devices[idx].msgReadDelta = 0;

            devices[idx].msgSendCurrent = 0;
            devices[idx].msgSendLast = 0;
            devices[idx].msgSendDelta = 0;

            devices[idx].sequenceNumber = 0;
            devices[idx].lostPackets = 0;
            printf("Added device %d to index %d\n", id, idx);
            break;
        }
    }
    if (rc == 0) {
        printf("Could not add device.\n");
    }

    if (useBodyFrame) {
        d.enableBodyMotion = 1;
	d.enableUserPosition = 0;
	d.inhibitPowerManager = 1;
	d.enableMouseMovement = 0;
	d.disableFreespace = 0;
	rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    } else {
        // You have to know what you're doing to make sense of this
        buffer[0] = 34;
        buffer[1] = 0x20;
        rc = 2;
    }

    if (rc > 0) {
        rc = freespace_sendAsync(id, buffer, rc, 1000, sendCallback, NULL);
        //rc = freespace_send(id, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        }
    } else {
        printf("Could not encode message %d\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/
}

static void cleanupDevice(FreespaceDeviceId id) {
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    struct freespace_DataMotionControl d;
    int rc;
    int idx;

    // Remove the device from our list.
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id == id) {
            devices[idx].id = -1;
            break;
        }
    }
    
    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
     printf("%d> Sending message to enable mouse motion data.\n", id);
    d.enableBodyMotion = 0;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 1;
    d.disableFreespace = 0;
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(id, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        } else {
	    //flush it to clear the DMC response message that might come in
	    freespace_flush(id);
#ifdef WIN32
	    Sleep(1);
#else
	    usleep(1000);
#endif
	    freespace_flush(id);
	}
    } else {
        printf("Could not encode message.\n");
    }
    //close it
    printf("%d> Cleaning up...\n", id);
    freespace_closeDevice(id);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/
}

static void sendMessage(FreespaceDeviceId id) {
    int rc;
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];

    rc = freespace_encodeProductIDRequest((int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_sendAsync(id, buffer, rc, 1000, sendCallback, NULL);
    }
    rc = freespace_encodeBatteryLevelRequest((int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_sendAsync(id, buffer, rc, 1000, sendCallback, NULL);
    }
#if 0 //should only send this when changing modes
    if (useBodyFrame) {
        struct freespace_DataMotionControl d;
        // Ensure in body frame mode.
        d.enableBodyMotion = 1;
	d.enableUserPosition = 0;
	d.inhibitPowerManager = 1;
	d.enableMouseMovement = 0;
	d.disableFreespace = 0;
	rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    } else {
        // You have to know what you're doing to make sense of this
        buffer[0] = 34;
        buffer[1] = 0x20;
        rc = 2;
    }
    if (rc > 0) {
        rc = freespace_sendAsync(id, buffer, rc, 1000, sendCallback, NULL);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        }
    } else {
        printf("Could not encode message.\n");
    }
#endif
}

static void hotplugCallback(enum freespace_hotplugEvent event, FreespaceDeviceId id, void* cookie) {
    if (event == FREESPACE_HOTPLUG_REMOVAL) {
        printf("Closing removed device %d\n", id);
        cleanupDevice(id);
    } else if (event == FREESPACE_HOTPLUG_INSERTION) {
        //printf("Opening newly inserted device %d\n", id);
        initDevice(id);
    }
}

int main(int argc, char* argv[]) {
    int idx;

    for (idx = 1; idx < argc; ++idx) {
        if (strcmp(argv[idx], "--use-multifsm-board") == 0) {
	    useBodyFrame = 0;
	} else if (strcmp(argv[idx], "--send-message-each-loop") == 0) {
	    sendMessageEachLoop = 1;
	}
    }

    addControlHandler();

    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        devices[idx].id = -1;
    }

    // Initialize the freespace library
    freespace_init();
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);
    freespace_perform();
    freespace_syncFileDescriptors();

    timer_initialize();
    while (!quit) {
        freespace_perform();
#ifdef WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
        // Process and display the statistics every second.
        if (timer_step()) {
            int offset = 0;
            int deviceCount = 0;
            for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
                if (devices[idx].id >= 0) {
                    deviceCount++;
                }
            }
            printf("Connected to %d devices:\n", deviceCount);
            for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
                int tmp;
                if (devices[idx].id < 0) {
                    continue;
                }
		if (useBodyFrame) {
		    tmp = devices[idx].msgReadCurrent;
		    devices[idx].msgReadDelta = tmp - devices[idx].msgReadLast;
		    devices[idx].msgReadLast = tmp;

		    tmp = devices[idx].msgSendCurrent;
		    devices[idx].msgSendDelta = tmp - devices[idx].msgSendLast;
		    devices[idx].msgSendLast = tmp;

		    printf("[%02d: %02d %03d %05d %d]   ", devices[idx].id, devices[idx].msgSendDelta,
			   devices[idx].msgReadDelta, devices[idx].lostPackets, devices[idx].sequenceNumber);
		} else {
		    //print the read/send errored and count (sequenceNumber)
		    printf("[%02d: %02d %02d %05d - %d %d]   ", devices[idx].id, devices[idx].msgReadError, 
			   devices[idx].msgSendError, devices[idx].sequenceNumber, devices[idx].msgReadCurrent, devices[idx].msgSendCurrent);
		}
                offset++;
                if (offset >= 3) {
                    printf("\n");
                    offset = 0;
                }

                // Send out our message request message(s)
		if (sendMessageEachLoop) {
		    sendMessage(devices[idx].id);
		}
            }
            printf("\n");
        }
    }

    printf("Cleaning up all devices...\n");
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id < 0) {
            continue;
        }
        cleanupDevice(devices[idx].id);
    }
    freespace_exit();

    return 0;
}
