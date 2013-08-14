/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2013, Hillcrest Laboratories, Inc.
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
#include <stdlib.h>
#include <signal.h>
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

#include <string.h>


/**
 * printFlags
 * This function checks for a change in Activity
 * Classification, or for a Power Management
 * Event Flag, and prints the meaning for each
 * flag.
 * ACPM - The MultiAxisSensor holding the most recent
 * Activity Classification and Power Management Flags.
 * lastACPM - The MultiAxisSensor holding the previous
 * Activity Classification and Power Management Flags.
 */
void printFlags(struct MultiAxisSensor ACPM, struct MultiAxisSensor lastACPM) {
	// Activity classification is in the x-coordinate
	int actClass = (int)ACPM.x;
	int lastActClass = (int)lastACPM.x;
	// Power management flags are in the y-coordinate
	int powerFlags = (int)ACPM.y;
	int lastPowerFlags = (int)lastACPM.y;
	// These are for formatting
	int needComma = 0;
	int needNewLine = 0;
	// If the activity class has changed then display it
	if (actClass != lastActClass) {
		printf("Activity Class: ");
		switch (actClass) {
		case 0: printf("Unknown.    "); break;
		case 1: printf("On table.   "); break;
		case 2: printf("Stationary. "); break;
		case 3: printf("Stable.     "); break;
		case 4: printf("Motion.     "); break;
		}
		needNewLine = 1;
	}
	// If the power management flags are not empy and have changed
	// then display them
	if (powerFlags != 0 && powerFlags != lastPowerFlags) {
		printf("Event: ");
		// LSB is "on table" flag
		if ((powerFlags) % 2) {
			printf("On table");
			needComma = 1;
		}

		// bit 1 is "stable" flag
		if ((powerFlags >> 1) % 2) {
			if (needComma) {
				printf(", is stable");
			} else {
				printf("Is stable");
				needComma = 1;
			}
		}

		// bit 2 is "motion detected by accelerometer" flag
		if ((powerFlags >> 2) % 2) {
			if (needComma) {
				printf(", motion detected by accelerometer");
			} else {
				printf("Motion detected by accelerometer");
				needComma = 1;
			}
		}

		// MSB is "motion detected by MotionEngine" flag
		if ((powerFlags >> 3) % 2) {
			if (needComma) {
				printf(", motion detected by MotionEngine");
			} else {
				printf("MotionDetected by MotionEngine");
				needComma = 1;
			}
		}
		printf(".");
		needNewLine = 1;
	}
	if (needNewLine)
		printf("\n");
}



/**
 * main
 * This example uses the synchronous API to
 *  - find a device
 *  - open the device found
 *  - configure the device to output Activity Classification
 *  and Power Management Flags
 *  - Read the responses and print their meanings
 * This example assume the device is already connected.
 */
int main(int argc, char* argv[]) {
    struct freespace_message message;
    FreespaceDeviceId device;
    int numIds; // The number of device ID found
    int rc; // Return code
    struct MultiAxisSensor ACPM;
	struct MultiAxisSensor lastACPM = {0, 0, 0, 0};

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

    printf("Scanning for Freespace devices...\n");
     // Get the ID of the first device in the list of availble devices
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("Didn't find any devices.\n");
        return 1;
    }

    printf("Found a device. Trying to open it...\n");
    // Prepare to communicate with the device found above
    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error opening device: %d\n", rc);
        return 1;
    }

    // Display the device information.
    printDeviceInfo(device);

    // Make sure any old messages are cleared out of the system
    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error flushing device: %d\n", rc);
        return 1;
    }

    // Configure the device for motion outputs
    printf("Sending message to enable motion data.\n");
    memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;        // MotionEngine Output
    message.dataModeControlV2Request.modeAndStatus |= 4 << 1; // Set full motion on
    message.dataModeControlV2Request.formatSelect = 1;        // MEOut format 1
    message.dataModeControlV2Request.ff7 = 1;                 // ActClass/PowerMgmt
    
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("Could not send message: %d.\n", rc);
    }
        
    // A loop to read messages
    printf("Listening for messages.\n");
    while (!quit) {
        rc = freespace_readMessage(device, &message, 100);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            // Both timeout and interrupted are ok.
            // Timeout happens if there aren't any events for a second.
            // Interrupted happens if you type CTRL-C or if you
            // type CTRL-Z and background the app on Linux.
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("Error reading: %d. Quitting...\n", rc);
            break;
        }

		// With each packet print the ACPM if it has changed
        if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
            rc = freespace_util_getActClass(&message.motionEngineOutput, &ACPM);
            if (rc == 0) {
                printFlags(ACPM, lastACPM);
				lastACPM = ACPM;
            }
        }
    }

    // Close communications with the device
    printf("Cleaning up...\n");
    freespace_closeDevice(device);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/

    // Cleanup the library
    freespace_exit();

    return 0;
}
