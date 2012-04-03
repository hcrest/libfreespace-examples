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
#include <windows.h>
#include <stdio.h>
#else
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#endif

#include <freespace/freespace.h>
#include "appControlHandler.h"

#define BUFFER_LENGTH 1024

/**
 * Callback that displays the devices that have been inserted into and removed
 * from the system.
 * Implements freespace_hotplugCallback
 * @param event The type of event.
 * @param id The affected device.
 * @param cookie Not used for this application.
 */
void hotplugCallback(enum freespace_hotplugEvent event,
                     FreespaceDeviceId id,
                     void* cookie) {
    int rc;

    if (event == FREESPACE_HOTPLUG_INSERTION) {
        printf("Device Inserted: %d\n", id);
        rc = printDeviceInfo(id);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not display device info: %d\n", rc);
        }
    } else {
        printf("Device Removed: %d\n", id);
    }
}

int main(int argc, char* argv[]) {
    int rc;

    printVersionInfo(argv[0]);

    addControlHandler();

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
        return 1;
    }

    // Set the callback to catch the initial devices.
    printf("Detecting the Freespace devices already connected...\n");
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);

    printf("Waiting for Freespace devices to be inserted or removed...\n");
    printf("Type Ctrl-C to exit\n");
    while (!quit) {
        // Easy event loop - just poll freespace_perform periodically
        // rather than waiting on select or WaitForMultipleObjects
#ifdef _WIN32
        Sleep(100);
#else
        sleep(1);
#endif

        // Callbacks are called from within the perform call.
        freespace_perform();
    }

    printf("Exiting\n");
    freespace_exit();

    return 0;
}
