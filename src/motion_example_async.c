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

#ifdef _WIN32
#include "win32/stdafx.h"
#include <stdio.h>
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
#include <freespace/freespace_util.h>
#include "appControlHandler.h"
#include <string.h>

#define TIMEOUT_MAX 1000

#if defined __GLIBC__
#if __GLIBC_PREREQ(2, 4)
#define HAS_PPOLL
#endif
#endif

#ifdef _WIN32

static HANDLE waitHandles[MAXIMUM_WAIT_OBJECTS];
static DWORD waitHandleCount = 0;

static void init_waitset() {
}

/**
 * add_pollfd
 * This function is passed to freespace_setFileDescriptorCallbacks.
 * It is called when a device handle needs to be added to the poll
 * list.
 */
static void add_pollfd(FreespaceFileHandleType fd, short events) {
    DWORD i;

	// Check if already in array
    for (i = 0; i < waitHandleCount; i++) {
        if (waitHandles[i] == fd) {
            return;
        }
    }

	// Check if array is full
    if (waitHandleCount >= MAXIMUM_WAIT_OBJECTS) {
        printf("Waiting for too many handles!!!\n");
        return;
    }

	// Add new handle to array
    waitHandles[waitHandleCount] = fd;
    waitHandleCount++;

    printf("Added a handle %p. Total handles = %d\n", fd, waitHandleCount);
}

/**
 * remove_pollfd
 * This function is passed to freespace_setFileDescriptorCallbacks.
 * It is called when a device handle needs to be removed from the
 * poll list.
 */
static void remove_pollfd(FreespaceFileHandleType fd) {
    DWORD i;

	// Look for fd in array
    for (i = 0; i < waitHandleCount; i++) {
        if (waitHandles[i] == fd) {
            break;
        }
    }

	// If fd was found in array, remove it and shift the array down if necessary
    if (i != waitHandleCount) {
        memmove(&waitHandles[i], &waitHandles[i + 1], (waitHandleCount - i - 1) * sizeof(HANDLE));
        waitHandleCount--;
    }

    printf("Removed a handle %p. Total handles = %d\n", fd, waitHandleCount);
}

#else
static struct pollfd* fds;
static nfds_t nfds;
static nfds_t maxfds;

static void init_waitset() {
    nfds = 0;
    maxfds = 5;
    fds = (struct pollfd*) malloc(maxfds * sizeof(struct pollfd));
}

static void add_pollfd(FreespaceFileHandleType fd, short events) {
    int i;


    for (i = 0; i < nfds; i++) {
        if (fds[i].fd == fd) {
            fds[i].events = events;
            return;
        }
    }

    if (nfds == maxfds) {
        maxfds += 5;
        fds = (struct pollfd*) realloc(fds, maxfds * sizeof(struct pollfd));
    }
    fds[nfds].fd = fd;
    fds[nfds].events = events;
    fds[nfds].revents = 0;

    nfds++;
}

static void remove_pollfd(FreespaceFileHandleType fd) {
    int i;

    for (i = 0; i < nfds; i++) {
        if (fds[i].fd == fd) {
            break;
        }
    }

    if (i != nfds) {
        memmove(&fds[i], &fds[i + 1], (nfds - i - 1) * sizeof(struct pollfd));
        nfds--;
    }
}
#endif

/**
 * sendCallback
 * This function is passed to freespace_sendMessageAsync.
 * It receives the response from the message sent by the
 * sendMessageAsync.
 */
static void sendCallback(FreespaceDeviceId id, void* cookie, int result) {
	if (result != FREESPACE_SUCCESS) {
		printf("Error %d.\n", result);
        if (result == FREESPACE_ERROR_NOT_FOUND) {
            freespace_closeDevice(id);
        }
	}
}
/**
 * receiveMessageCallback
 * This function is passed to freespace_setReceiveMessageCallback.
 * It performs error handling or just status updates based on the
 * value of result.
 */
static void receiveMessageCallback(FreespaceDeviceId id,
                            struct freespace_message * message,
                            void* cookie,
                            int result) {
	struct MultiAxisSensor acc;
	int rc;

    if (result == FREESPACE_ERROR_TIMEOUT ||
        result == FREESPACE_ERROR_INTERRUPTED) {
        // Both timeout and interrupted are ok.
        // Timeout happens if there aren't any events for a second.
        // Interrupted happens if you type CTRL-C or if you
        // type CTRL-Z and background the app on Linux.
        return;
    }

	// This indicates which device we are receiving
	printf("%d> ", id);

	// If the result was not a success there is a
	// problem with the device
    if (result != FREESPACE_SUCCESS) {
		printf("Error %d.\n", result);
		if (result == FREESPACE_ERROR_NOT_FOUND) {
            freespace_closeDevice(id);
        }
        return;
    }

	// If the message type is a MotionEngine output then use
	// the freespace_util to extract the data
	if (message->messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
        rc = freespace_util_getAccNoGravity(&(message->motionEngineOutput), &acc);
        if (rc == 0) {
            printf ("X: % 6.2f, Y: % 6.2f, Z: % 6.2f\n", acc.x, acc.y, acc.z);
        }
    } else if (message != 0) {
        freespace_printMessage(stdout, message);
	}
}

/**
 * initDevice
 * This function opens a handle to the device corresponding to
 * the given deviceId. It registers the device with the message
 * callback function and sets it up for streaming motion data.
 */
static void initDevice(FreespaceDeviceId id) {
    struct freespace_message message;
    struct freespace_DataModeRequest * req;
    struct freespace_DataModeControlV2Request * req2;
    int rc;

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/

    printf("Found a device. Trying to open it...\n");
    rc = freespace_openDevice(id);
    if (rc != 0) {
        printf("Error opening device.\n");
        return;
    }

    // Display the device information.
    printDeviceInfo(id);

    printf("Opened. Flushing any queued messages.\n");
    rc = freespace_flush(id);
    if (rc != 0) {
        printf("Error flushing device.\n");
        return;
    }

	// Register the receive message callback function
    freespace_setReceiveMessageCallback(id, receiveMessageCallback, NULL);

    // Configure the device for motion outputs
    printf("Sending message to enable motion data.\n");
    memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;        // MotionEngine Outout
    message.dataModeControlV2Request.modeAndStatus |= 0 << 1; // Set full motion
    message.dataModeControlV2Request.formatSelect = 0;        // MEOut format 0
    message.dataModeControlV2Request.ff0 = 1;                 // Pointer fields
	message.dataModeControlV2Request.ff2 = 1;                 // Acceleration fields
    
    rc = freespace_sendMessageAsync(id, &message, TIMEOUT_MAX, sendCallback, NULL);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }

    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/
}

/**
 * hotplugCallback
 * This function is called whenever a freespace device is inserted 
 * or removed from the USB.
 */
static void hotplugCallback(enum freespace_hotplugEvent event, FreespaceDeviceId id, void* cookie) {
    if (event == FREESPACE_HOTPLUG_REMOVAL) {
        printf("Closing removed device %d\n", id);
        freespace_closeDevice(id);
    } else if (event == FREESPACE_HOTPLUG_INSERTION) {
        printf("Opening newly inserted device %d\n", id);
        initDevice(id);
    }
}

/**
 * main
 * This example collects motion from multiple devices simultaneously
 * using the asynchronous
 */
int main(int argc, char* argv[]) {
    FreespaceDeviceId deviceIds[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int numIds;	// The number of device Ids found
    int rc;		// Return Code

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

    init_waitset();
#ifdef HAS_PPOLL
    {
        // Signal handling when working with ppoll.
        sigset_t blockset;
        sigemptyset(&blockset);
        sigaddset(&blockset, SIGINT);
        sigaddset(&blockset, SIGHUP);
        sigprocmask(SIG_BLOCK, &blockset, NULL);
    }
#endif

	// This will set the hotplug callback and catch the initial devices
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);
    freespace_setFileDescriptorCallbacks(add_pollfd, remove_pollfd);
    freespace_syncFileDescriptors();
    freespace_perform();

    while (!quit) {
        int timeoutMs;

        freespace_getNextTimeout(&timeoutMs);

        // Adjust the maximum timeout to allow for Ctrl-C handling
        if (timeoutMs < 0 || timeoutMs > TIMEOUT_MAX) {
            timeoutMs = TIMEOUT_MAX;
        }
#ifdef _WIN32
        {
			// This polls all the handles in waitHandles and waits for timeoutMs.
            DWORD bResult = WaitForMultipleObjects(waitHandleCount, waitHandles, FALSE, (unsigned int) timeoutMs);
            if (bResult == WAIT_FAILED) {
                printf("Error from WaitForMultipleObjects\n");
                break;
            }
        }
#else
#ifndef HAS_PPOLL
        {
            int ready;
            // Look into the signal handler race condition that ppoll fixes
            // and either live with it or fix it here.
            ready = poll(fds, nfds, timeoutMs);
            if (ready < 0) {
                printf("Error from poll\n");
                break;
            }
        }
#else
        {
            int ready;
            struct timespec tmspec;
            sigset_t emptyset;

            sigemptyset(&emptyset);
            if (timeoutMs >= 0) {
                tmspec.tv_sec = timeoutMs / 1000;
                tmspec.tv_nsec = (timeoutMs % 1000) * 1000000;
                ready = ppoll(fds, nfds, &tmspec, &emptyset);
            } else {
                ready = ppoll(fds, nfds, NULL, &emptyset);
            }
            if (ready < 0) {
                printf("Error from ppoll\n");
                break;
            }
        }
#endif
#endif
		// Once poll is completed this function services all of the
		// handles in wait_handles
        freespace_perform();
    }

	// Clean up all the devices at the end.
	// This turns them all into mouse mode.
    printf("Cleaning up all devices...\n");
    rc = freespace_getDeviceList(deviceIds, FREESPACE_MAXIMUM_DEVICE_COUNT, &numIds);
    if (rc == FREESPACE_SUCCESS) {
        int i;
        for (i = 0; i < numIds; i++) {
            // Close communications with the device
			printf("%d> Cleaning up...\n", i);
			freespace_closeDevice(deviceIds[i]);
        }
    } else {
        printf("Error getting device list.\n");
    }

	// Cleanup the library
    freespace_exit();

    return 0;
}
