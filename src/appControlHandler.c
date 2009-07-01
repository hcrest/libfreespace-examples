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

#ifdef _WIN32
#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#else
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#endif

#include "appControlHandler.h"

int quit = 0;

void printVersionInfo(const char* appname) {
    printf("%s: Using libfreespace %s\n",
           appname,
           freespace_version());
}

int printDeviceInfo(FreespaceDeviceId id) {
    struct FreespaceDeviceInfo info;
    int rc;

    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    printf("Device = %s\n   Vendor ID  = 0x%x (%d)\n   Product ID = 0x%x (%d)\n",
           info.name, info.vendor, info.vendor, info.product, info.product);
    return FREESPACE_SUCCESS;
}

#ifdef _WIN32
static BOOL CtrlHandler(DWORD fdwCtrlType) {
    quit = 1;
    return TRUE;
}

void addControlHandler() {
    if (!SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE )) {
        printf("Could not install control handler\n");
    }
}

#else

static void sighandler(int num) {
    quit = 1;
}
void addControlHandler() {
    // Set up the signal handler to catch
    // CTRL-C and clean up gracefully.
    struct sigaction setmask;
    sigemptyset(&setmask.sa_mask);
    setmask.sa_handler = sighandler;
    setmask.sa_flags = 0;

    sigaction(SIGHUP, &setmask, NULL);
    sigaction(SIGINT, &setmask, NULL);
}
#endif
