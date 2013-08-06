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
#define Sleep(x) usleep((x)*1000)
#endif

#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

#include <string.h>
#include <time.h>


#define MOTION_INTERVAL 2
#define MAX_WAIT_SECS 2
#define BUFFER_SIZE 32


/**
 * getTimeStamp
 * Calculates a time stamp in
 * seconds based on the number of clock
 * ticks that have elapsed.
 * return - The time stamp in seconds.
 */
double getTimeStamp() {
	clock_t t;
	t = clock();
	return ((double)t)/((double)CLOCKS_PER_SEC);
}

/**
 * getSensorString
 * Gets the name of a sensor from it's index
 * sensor - The sensor index - see the HCOMM document
 * for more information.
 * str - A string to hold the return value.
 */
void getSensorString(int sensor, char* str) {
	switch (sensor) {
	case 0: strcpy(str, "Accelerometer"); break;
	case 1: strcpy(str, "Gyroscope"); break;
	case 2: strcpy(str, "Magnetometer"); break;
	case 3: strcpy(str, "Ambient Light Sensor"); break;
	case 4: strcpy(str, "Pressure Sensor"); break;
	case 5: strcpy(str, "Proximity Sensor"); break;
	case 6: strcpy(str, "Sensor Fusion"); break;
	default: strcpy(str, ""); break;
	}
}

/**
 * sendSetSensorPeriodMessage
 * Sends a message to change the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * period - The desired period of the sensor in us.
 * return - 1 if successful, 0 if failed
 */
int sendSetSensorPeriodMessage(FreespaceDeviceId device, int sensor, int period) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.commit = 1;	// Need to commit change in order to work
	message.sensorPeriodRequest.get = 0;  // We are setting, not getting
	message.sensorPeriodRequest.sensor = sensor; // Sensor index - see the HCOMM doc for more info
	message.sensorPeriodRequest.period = period; // Period in us

	return freespace_sendMessage(device, &message);
}

/**
 * sendGetSensorPeriodMessage
 * Sends a message to read the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * return - 1 if successful, 0 if failed
 */
int sendGetSensorPeriodMessage(FreespaceDeviceId device, int sensor) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.get = 1;  // We are getting, not setting
	message.sensorPeriodRequest.sensor = sensor; // Sensor index - see HCOMM doc for more info

	return freespace_sendMessage(device, &message);
}

/**
 * sendMotionRequestMessage
 * Sends a message to the Freespace device to start
 * streaming motion data.
 * device - The Freespace Device ID of the device
 * return - 1 if successful, 0 if failed
 */
int sendMotionRequestMessage(FreespaceDeviceId device) {
	struct freespace_message message;

	// Configure the device for motion outputs
    printf("Sending message to enable motion data.\n");
    memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;        // MotionEngine Outout
    message.dataModeControlV2Request.modeAndStatus |= 4 << 1; // Set full motion - always on
	message.dataModeControlV2Request.formatSelect = 1;
    
    return freespace_sendMessage(device, &message);
}

/**
 * getInput
 * Function for collecting an integer input from
 * the user.
 * val - A pointer to where to store the input.
 * return - 1 if received valid input, 0 if did
 * not receive valid input.
 */
int getInput(int* val) {
	char buf[BUFFER_SIZE];
	
	fgets(buf, BUFFER_SIZE, stdin);
	if (buf[0] == '\n') return 0;
	else {
		return sscanf(buf, "%d", val);
	}
}

/**
 * printDataRate
 * Calculates and prints the average data rate based
 * on the given data.
 * elapsedTime - The interval the data was collected
 * samples - The amount of samples collected during
 * the interval.
 */
void printDataRate(double elapsedTime, int samples) {
	printf("Measured data period: %6.2f us", elapsedTime*1e6 / (double)samples);
	printf(" (%d packets in %f seconds)\n", samples, elapsedTime);
}

/**
 * waitForPeriodResponse
 * Waits on a freespace message response to a
 * sensor period message. Waits up to MAX_WAIT_SECS
 * before giving up.
 * device - The Freespace Device ID of the device
 * sensorValue - Pointer where the sensor index is stored
 * periodValue - Pointer where the period value is stored
 * return - -1 if message error, 0 if timeout, 1 if successful
 */
int waitForPeriodResponse(FreespaceDeviceId device, int* sensorValue, int* periodValue) {
	double lastTime = 0;
	int rc = 0;
	struct freespace_message message;
	
	lastTime = getTimeStamp();
	while (rc != FREESPACE_ERROR_INTERRUPTED) {
		rc = freespace_readMessage(device, &message, 100);
		if (rc == FREESPACE_ERROR_TIMEOUT) {
			// Both timeout and interrupted are ok.
			// Timeout happens if there aren't any events for a second.
			// Interrupted happens if you type CTRL-C or if you
			// type CTRL-Z and background the app on Linux.

			if (getTimeStamp() > lastTime + MAX_WAIT_SECS) {
				return 0;
			}
			continue;
		}
		if (rc != FREESPACE_SUCCESS) {
			printf("Error reading: %d. Quitting...\n", rc);
			return -1;
		}
		// Check if the sensor has given us a Sensor Period response
		if (message.messageType == FREESPACE_MESSAGE_SENSORPERIODRESPONSE) {
			*sensorValue = message.sensorPeriodResponse.sensor;
			*periodValue = message.sensorPeriodResponse.period;
			return 1;
		} else {
			if (getTimeStamp() > lastTime + MAX_WAIT_SECS) {
				return 0;
			}
		}
	}
	return 0;
}

/**
 * main
 * This example uses the synchronous API to
 *  - find a device
 *  - open the device found
 *  - read the sensor period values from the device
 *  - send a new sensor fusion period value to the device
 *  - stream motion data and measure the period
 * This example assume the device is already connected.
 */
int main(int argc, char* argv[]) {
	struct freespace_message message;
    FreespaceDeviceId device;
    int numIds; // The number of device ID found
    int rc; // Return code
	double lastTime = 0; // Used to measure intervals
	int ticks = 0;	// Tracks the number of packets received
	int index = 0;	// Loop variable
	char str[BUFFER_SIZE]; //Holds sensor names
	int sensor = 0;	// Holds the sensor number
	int period = 0; // Holds the period of the sensor
	int inputValue = 0; // Holds the user input period
	int fusionTimedOut = 0; // Flag to indicate sensor fusion timeout
	int madeChange = 0; // Flag to indicate user changed a value
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

	// First we gather information about our sensors.
	printf("\nSensors:\n");
	for (index = 0;index < 7;index++) {
		if (quit) break;

		// Request the sensor period information
		rc = sendGetSensorPeriodMessage(device, index);
		if (rc != FREESPACE_SUCCESS) {
			printf("Could not send message: %d.\n", rc);
			return -1;
		}

		// Wait for a response
		rc = waitForPeriodResponse(device, &sensor, &period);
		if (rc == -1) { // Indicates message failure
			printf("Error getting sensor period info.\n");
			return 1;
		} else if (rc == 0) { // Indicates timeout
			getSensorString(index, str);
			printf("%d. %s TIMED OUT.\n", index, str);

			// If sensor fusion timed out make value invalid
			if (index == 6) fusionTimedOut = 1;
		} else { // Indicates success
			getSensorString(sensor, str);
			printf("%d. %s", sensor, str);
			if (period != 0)
				printf(" @ %d us.\n", period);
			else
				printf(" disabled.\n");
		}
	}
	printf("\n");

	// Next we ask the user if they want to choose a sensor fusion value
	// Skip this if sensor fusion timed out earlier
	if (!quit && !fusionTimedOut) {
		rc = -1;
		while (!quit && rc < 0) {
			inputValue = 0;
			printf("New sensor fusion period (Enter to leave unchanged): ");
			rc = getInput(&inputValue);
			// This gives the computer a chance to detect SIGINT
			// Why there is a delay? I don't know.
			Sleep(5);
			if (rc == 1) {
				if (inputValue >= 0) {
					// If user provides valid value
					// then we send the new value
					madeChange = 1;
				} else rc = -1;
			} else if (!quit) {
				printf("Using default Sensor Fusion period.\n\n");
			}
		}
	}

	if (!quit && madeChange) {
		// Send the new value to the device
		printf("Setting Sensor Fusion period to %d us.\n\n", inputValue);
		rc = sendSetSensorPeriodMessage(device, 6, inputValue);
		if (rc != FREESPACE_SUCCESS) {
			printf("Could not send message: %d.\n", rc);
			return 1;
		}
		
		// Wait for a response
		rc = waitForPeriodResponse(device, &sensor, &period);
		if (rc == -1) { // Indicates message failure
			printf("Error getting sensor period info.\n");
			return 1;
		} else if (rc == 0) { // Indicates timeout
			getSensorString(6, str);
			printf("6. %s TIMED OUT.\n", str);
		} else { // Indicates success
			getSensorString(sensor, str);
			printf("%d. %s @ %d us.\n", sensor, str, period);
		}

		// Give the device time to commit changes
		Sleep(500);

		// Update sensor information
		printf("\nSensors:\n");
		for (index = 0;index < 7;index++) {
			if (quit) break;
			// Request the sensor period information
			rc = sendGetSensorPeriodMessage(device, index);
			if (rc != FREESPACE_SUCCESS) {
				printf("Could not send message: %d.\n", rc);
				return -1;
			}

			// Wait for a response
			rc = waitForPeriodResponse(device, &sensor, &period);
			if (rc == -1) { // Indicates message failure
				printf("Error getting sensor period info.\n");
				return 1;
			} else if (rc == 0) { // Indicates timeout
				getSensorString(index, str);
				printf("%d. %s TIMED OUT.\n", index, str);
			} else { // Indicates success
				getSensorString(sensor, str);
				printf("%d. %s", sensor, str);
				if (period != 0)
					printf(" @ %d us.\n", period);
				else
					printf(" disabled.\n");
			}
		}
		printf("\n");
	}

	// Request the device stream motion to measure data rate
	if (!quit) {
		rc = sendMotionRequestMessage(device);
		if (rc != FREESPACE_SUCCESS) {
			printf("Could not send message: %d.\n", rc);
			return 1;
		}
	}

	// A loop to read messages
	lastTime = getTimeStamp();
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
		if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
			ticks++;
		}
		if ((getTimeStamp() - lastTime) > MOTION_INTERVAL) {
			printDataRate((getTimeStamp() - lastTime), ticks);
			ticks = 0;
			lastTime = getTimeStamp();
		}
	}

    // Close communications with the device
    printf("Cleaning up...\n");
    freespace_closeDevice(device);

    // Cleanup the library
    freespace_exit();

    return 0;
}
