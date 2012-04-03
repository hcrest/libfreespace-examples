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

static void add_pollfd(FreespaceFileHandleType fd, short events) {
    DWORD i;

    for (i = 0; i < waitHandleCount; i++) {
        if (waitHandles[i] == fd) {
            return;
        }
    }

    if (waitHandleCount >= MAXIMUM_WAIT_OBJECTS) {
        printf("Waiting for too many handles!!!\n");
        return;
    }

    waitHandles[waitHandleCount] = fd;
    waitHandleCount++;

    printf("Added a handle %p. Total handles = %d\n", fd, waitHandleCount);
}

static void remove_pollfd(FreespaceFileHandleType fd) {
    DWORD i;

    for (i = 0; i < waitHandleCount; i++) {
        if (waitHandles[i] == fd) {
            break;
        }
    }

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

static void receiveMessageCallback(FreespaceDeviceId id,
                            struct freespace_message * message,
                            void* cookie,
                            int result) {
    printf("%d> ", id);
    if (result == FREESPACE_SUCCESS) {
        if (message != 0) {
            freespace_printMessage(stdout, message);
        }
    } else {
        printf("Error %d.\n", result);
        if (result == FREESPACE_ERROR_NOT_FOUND) {
            freespace_closeDevice(id);
        }
    }
}

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


    freespace_setReceiveMessageCallback(id, receiveMessageCallback, NULL);

    printf("Sending message to enable body-frame motion data.\n");
    memset(&message, 0, sizeof(message));
    if (FREESPACE_SUCCESS == freespace_isNewDevice(id)) {
        message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
        req2 = &message.dataModeControlV2Request;
        req2->packetSelect = 2;
        req2->modeAndStatus |= 0 << 1;
    } else {
        message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
        req = &(message.dataModeRequest);
        req->enableBodyMotion = 1;
        req->inhibitPowerManager = 1;
    }
    rc = freespace_sendMessageAsync(id, &message, 1000, NULL, NULL);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/
}

static void cleanupDevice(FreespaceDeviceId id) {
    struct freespace_message message;
    int rc;

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("%d> Sending message to enable mouse motion data.\n", id);
    memset(&message, 0, sizeof(message));
    if (FREESPACE_SUCCESS == freespace_isNewDevice(id)) {
        message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
        message.dataModeControlV2Request.packetSelect = 1;
    } else {
        message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
        message.dataModeRequest.enableMouseMovement = 1;
    }

    rc = freespace_sendMessage(id, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }

    printf("%d> Cleaning up...\n", id);
    freespace_closeDevice(id);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/
}

static void hotplugCallback(enum freespace_hotplugEvent event, FreespaceDeviceId id, void* cookie) {
    if (event == FREESPACE_HOTPLUG_REMOVAL) {
        printf("Closing removed device %d\n", id);
        freespace_closeDevice(id);
    } else if (event == FREESPACE_HOTPLUG_INSERTION) {
        printf("Opening newly inserted device %d\n", id);
        initDevice(id);
    }
}


int main(int argc, char* argv[]) {
    FreespaceDeviceId deviceIds[FREESPACE_MAXIMUM_DEVICE_COUNT];
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
        freespace_perform();
    }

    printf("Cleaning up all devices...\n");
    rc = freespace_getDeviceList(deviceIds, FREESPACE_MAXIMUM_DEVICE_COUNT, &numIds);
    if (rc == FREESPACE_SUCCESS) {
        int i;
        for (i = 0; i < numIds; i++) {
            cleanupDevice(deviceIds[i]);
        }
    } else {
        printf("Error getting device list.\n");
    }

    freespace_exit();

    return 0;
}
