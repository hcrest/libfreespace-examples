/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2010, Hillcrest Laboratories, Inc.
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

#define TIMEOUT_MAX 1000

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

static int sendMessageEachLoop = 0;

static void receiveStructCallback(FreespaceDeviceId id,
                                  struct freespace_message* m,
                                  void* cookie,
                                  int result) {
    int idx;
    int isProcessed = 0;
    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        if (devices[idx].id == id) {
            isProcessed = 1;
            devices[idx].msgReadCurrent++;
            if (result != FREESPACE_SUCCESS) {
                devices[idx].msgReadError++;
            }
            if (m == 0) {
                break;
            }
            if (m->messageType == FREESPACE_MESSAGE_BODYFRAME) {	          
                if (devices[idx].sequenceNumber + 1 != m->bodyFrame.sequenceNumber && devices[idx].sequenceNumber != 0) {
                    devices[idx].lostPackets = m->bodyFrame.sequenceNumber - devices[idx].sequenceNumber;
                }
                devices[idx].sequenceNumber = m->bodyFrame.sequenceNumber;
            } else if (devices[idx].sequenceNumber == 0) {
                freespace_printMessageStruct(stdout, m);
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
    struct freespace_message message;
    int rc;
    int idx;

    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    freespace_setReceiveStructCallback(id, receiveStructCallback, NULL);

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

    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
    message.dataModeRequest.enableBodyMotion = 1;
    message.dataModeRequest.inhibitPowerManager = 1;

    rc = freespace_sendMessageStructAsync(id, &message, 0, 1000, sendCallback, NULL);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/
}

static void cleanupDevice(FreespaceDeviceId id) {
    struct freespace_message message;
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
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
    message.dataModeRequest.enableMouseMovement = 1;
    rc = freespace_sendMessageStruct(id, &message, 0);
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
    //close it
    printf("%d> Cleaning up...\n", id);
    freespace_closeDevice(id);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/
}

static void sendMessage(FreespaceDeviceId id) {
    struct freespace_message message;
    int rc;

    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_PRODUCTIDREQUEST;
    rc = freespace_sendMessageStructAsync(id, &message, 0, 1000, sendCallback, NULL);

    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_BATTERYLEVELREQUEST;
    rc = freespace_sendMessageStructAsync(id, &message, 0, 1000, sendCallback, NULL);
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
    int rc;

    printVersionInfo(argv[0]);

    for (idx = 1; idx < argc; ++idx) {
        if (strcmp(argv[idx], "--send-message-each-loop") == 0) {
            sendMessageEachLoop = 1;
        }
    }

    addControlHandler();

    for (idx = 0; idx < MAX_NUMBER_DEVICES; idx++) {
        devices[idx].id = -1;
    }

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

    timer_initialize();
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
                tmp = devices[idx].msgReadCurrent;
                devices[idx].msgReadDelta = tmp - devices[idx].msgReadLast;
                devices[idx].msgReadLast = tmp;

                tmp = devices[idx].msgSendCurrent;
                devices[idx].msgSendDelta = tmp - devices[idx].msgSendLast;
                devices[idx].msgSendLast = tmp;

                printf("[%02d: %02d %03d %05d %d]   ", devices[idx].id, devices[idx].msgSendDelta,
                devices[idx].msgReadDelta, devices[idx].lostPackets, devices[idx].sequenceNumber);
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
