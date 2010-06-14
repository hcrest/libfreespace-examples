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
#ifdef WIN32
#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#else
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"


#define BUFFER_LENGTH 1024

static void receiveStructCallback(FreespaceDeviceId id,
                                  struct freespace_message* m,
                                  void* cookie,
                                  int result) {
    int rc;
    struct FreespaceDeviceInfo info;

    if (result == FREESPACE_SUCCESS && m != NULL) {
        if (m->messageType == FREESPACE_MESSAGE_PRODUCTIDRESPONSE) {
            rc = freespace_getDeviceInfo(id, &info);
            if (rc != FREESPACE_SUCCESS) {
                return;
            }

            struct freespace_ProductIDResponse* pr = &(m->productIDResponse);

            if (pr->deviceClass == 1) {
                // Print out information including software version
                printf("Device ID: %d\n   Device = %s\n   Software Version of dongle = %d.%d.%d\n",
                       id, info.name, pr->swVersionMajor, pr->swVersionMinor, pr->swVersionPatch);
            } else if (pr->deviceClass == 2) {
                // Print out information including software version
                printf("Device ID: %d\n   Device = %s\n   Software Version of handheld = %d.%d.%d\n",
                       id, info.name, pr->swVersionMajor, pr->swVersionMinor, pr->swVersionPatch);
            }
        }
    } else {
        if (result == FREESPACE_ERROR_NOT_FOUND) {
            freespace_closeDevice(id);
        }
    }
}


/**
 * Callback that requests the firmware version of any devices that have been
 * inserted into the system.
 * Implements freespace_hotplugCallback
 * @param event The type of event.
 * @param id The affected device.
 * @param cookie Not used for this application.
 */
void hotplugCallback(enum freespace_hotplugEvent event,
                     FreespaceDeviceId id,
                     void* cookie) {
    int rc;
    struct freespace_message m;
    struct FreespaceDeviceInfo info;

    if (event == FREESPACE_HOTPLUG_INSERTION) {
        rc = freespace_openDevice(id);
        if (rc != FREESPACE_SUCCESS) {
            printf("Error opening device.\n");
            return;
        }

        freespace_setReceiveStructCallback(id, receiveStructCallback, NULL);
        rc = freespace_getDeviceInfo(id, &info);

        memset(&m, 0, sizeof(m));
        m.messageType = FREESPACE_MESSAGE_PRODUCTIDREQUEST;
        if (rc == FREESPACE_SUCCESS && info.hVer == 2) {
            rc = freespace_sendMessageStruct(id, &m, 1);
        }
        rc = freespace_sendMessageStruct(id, &m, 0);
        if (rc != FREESPACE_SUCCESS) {
            printf("Error sending productID request\n");
            return;
        }

    }
}

int main(int argc, char* argv[]) {
    int numIds;
    int deviceIds[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int rc;

    addControlHandler();

    // Initialize the freespace library
    freespace_init();

    // Set the callback to catch the initial devices.
    printf("Detecting the Freespace devices already connected...\n");
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);
    freespace_perform();

    printf("Waiting for Freespace devices to be inserted...\n");
    printf("Type Ctrl-C to exit\n");
    while (!quit) {
        // Easy event loop - just poll freespace_perform periodically
        // rather than waiting on select or WaitForMultipleObjects
#ifdef WIN32
        Sleep(100);
#else
        sleep(1);
#endif

        // Callbacks are called from within the perform call.
        freespace_perform();
    }

    printf("Exiting\n");
    printf("Cleaning up all devices...\n");
    rc = freespace_getDeviceList(deviceIds, FREESPACE_MAXIMUM_DEVICE_COUNT, &numIds);
    if (rc == FREESPACE_SUCCESS) {
        int i;
        for (i = 0; i < numIds; i++) {
            freespace_closeDevice(deviceIds[i]);
        }
    } else {
        printf("Error getting device list.\n");
    }

    freespace_exit();

    return 0;
}
