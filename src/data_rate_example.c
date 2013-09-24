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
#include <sys/time.h>
#include <unistd.h>
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

// The sensor period to set the sensors to
#define SENSOR_PERIOD 10000
// The interval we use to calculate average sample period
#define MOTION_INTERVAL 2
// Number of average sample period measurements we will take
#define MAX_ITERATIONS 5
// The names corresponding to the sensor indices
const char * const SENSOR_NAMES[] = {	
					"Accelerometer", 
					"Gyroscope", 
					"Magnetometer", 
					"Ambient Light Sensor",
					"Pressure Sensor",
					"Proximity Sensor",
					"Sensor Fusion"         };

/**
 * getTimeStamp
 * Calculates a time stamp in
 * seconds based on the number of clock
 * ticks that have elapsed.
 * return - The time stamp in seconds.
 */
double getTimeStamp() {
#ifdef _WIN32
	clock_t t;
	t = clock();
	return ((double)t)/((double)CLOCKS_PER_SEC);
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (double)tv.tv_usec/1e6 + (double)tv.tv_sec;
#endif
}

/**
 * sendSetSensorPeriodMessage
 * Sends a message to change the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * period - The desired period of the sensor in us.
 * commit - 0 to write without commit, 1 to commit changes
 * return - The return code of the message
 */
int sendSetSensorPeriodMessage(FreespaceDeviceId device, int sensor, int period, int commit) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.commit = commit;	// Need to commit change in order to work
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
 * return - The return code of the message
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
 * return - the return code of the message
 */
int sendMotionRequestMessage(FreespaceDeviceId device) {
	struct freespace_message message;

	// Configure the device for motion outputs
    printf("Sending message to enable motion data.\n");
    memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8;        // MotionEngine Outout
    message.dataModeControlV2Request.mode = 4; // Set full motion - always on
	message.dataModeControlV2Request.formatSelect = 1;
    
    return freespace_sendMessage(device, &message);
}

/**
 * waitForPeriodResponse
 * Waits on a freespace message response to a
 * sensor period message. Waits up to MAX_WAIT_SECS
 * before giving up.
 * device - The Freespace Device ID of the device
 * sensorValue - Pointer where the sensor index is stored
 * periodValue - Pointer where the period value is stored
 * return - FREESPACE_SUCCESS if successful, or a FREESPACE_ERROR otherwise
 */
int waitForPeriodResponse(FreespaceDeviceId device, int* sensorValue, int* periodValue) {
	int rc = 0;
	struct freespace_message message;

	// Keep looping if we get FREESPACE_SUCCESS but no SensorPeriodResponse
	while (rc == FREESPACE_SUCCESS) {
		rc = freespace_readMessage(device, &message, 200);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}
		// Check if the sensor has given us a Sensor Period response
		if (message.messageType == FREESPACE_MESSAGE_SENSORPERIODRESPONSE) {
			if (sensorValue != NULL)
				*sensorValue = message.sensorPeriodResponse.sensor;
			if (periodValue != NULL)
				*periodValue = message.sensorPeriodResponse.period;
			return FREESPACE_SUCCESS;
		}
	}
	return 0;
}

/**
 * printSensorInfo
 * Prints the sensor period information for a device's sensors.
 * It sends a getSensorPeriod message for every sensor, then waits
 * for the responses, each containing the period for a sensor.
 * device - The Freespace Device ID of the device
 * return - FREESPACE_SUCCESS if successful, or a FREESPACE_ERROR otherwise
 */
int printSensorInfo(FreespaceDeviceId device) {
	int rc;
	int index;
	int sensor;
	int period;

	// Update sensor information
	printf("\nSensors:\n");
	for (index = 0;index < 7;index++) {
		// Request the sensor period information
		rc = sendGetSensorPeriodMessage(device, index);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}

		// Wait for a response
		rc = waitForPeriodResponse(device, &sensor, &period);

		if (rc == FREESPACE_ERROR_TIMEOUT) { // Indicates timeout
			printf("     %d. %s TIMED OUT.\n", index, SENSOR_NAMES[index]);
		} else if (rc == FREESPACE_SUCCESS) {
			printf("     %d. %s", sensor, SENSOR_NAMES[index]);
			if (period != 0)
				printf(" @ %d us.\n", period);
			else
				printf(" disabled.\n");
		} else {
			return rc;
		}
	}
	return FREESPACE_SUCCESS;
	printf("\n");
}

/**
 * main
 * This example uses the synchronous API to
 *  - find a device
 *  - open the device found
 *  - read the sensor period values from the device
 *  - send a new value for the sensor periods on the device
 *  - stream motion data and measure the period
 * This example assume the device is already connected.
 */
int main(int argc, char* argv[]) {
	struct freespace_message message;
    FreespaceDeviceId device;
    int numIds; // The number of device ID found
    int rc; // Return code
	double lastTime = 0; // Used to measure intervals
	int iterations = 0;	// Tracks the number of measurements we make
	int packets = 0;	// Tracks the number of packets received
	int i = 0;	// Loop variable
	int sensor = 0;	// Holds the sensor number
	int period = 0; // Holds the period of the sensor
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

	// Print out the sensor info
	rc = printSensorInfo(device);
	if (rc != FREESPACE_SUCCESS) {
		printf("Error getting sensor info: %d\n", rc);
		return 1;
	}

	// Set all sensors to 20ms
	printf("\nChanging sensor periods to %d...\n", SENSOR_PERIOD);
	for (i = 0;i < 7;i++) {
		if (i == 6) {	// If we are on last sensor we need to commit
			rc = sendSetSensorPeriodMessage(device, i, SENSOR_PERIOD, 1);
		} else {
			rc = sendSetSensorPeriodMessage(device, i, SENSOR_PERIOD, 0);
		}
		if (rc != FREESPACE_SUCCESS) {
			printf("Could not send message: %d.\n", rc);
			return 1;
		}
		
		// Wait for a response to the change
		rc = waitForPeriodResponse(device, &sensor, &period);
		if (rc == FREESPACE_ERROR_TIMEOUT) {
			printf("%s timed out.\n", SENSOR_NAMES[i]);
		} else if (rc != FREESPACE_SUCCESS) {
			printf("Failed with error code: %d.\n", rc);
			return 1;
		}
	}

	// Give the device time to commit changes
	Sleep(500);

	// Print out the new sensor info
	rc = printSensorInfo(device);
	if (rc != FREESPACE_SUCCESS) {
		printf("Error getting sensor info: %d\n", rc);
		return 1;
	}

	printf("\n");

	//Send message to request motion
	sendMotionRequestMessage(device);

	// A loop to read motion messages
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

		// If we receive a motion packet, increment the packet count
		if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
			packets++;
		}

		/* Every MOTION_INTERVAL, calculate the average
		 * sample period and display it.			*/
		if ((getTimeStamp() - lastTime) > MOTION_INTERVAL) {
			printf("Measured data period: %6.2f us", 
				(getTimeStamp() - lastTime)*1e6 / (double)packets);
			printf(" (%d packets in %f seconds)\n", packets, 
				(getTimeStamp() - lastTime));
			packets = 0;
			lastTime = getTimeStamp();
			iterations++;
		}

		/* After MAX_ITERATIONS of calculating the average
		 * sample period, finish.    */
		if (iterations > MAX_ITERATIONS) {
			break;
		}
	}

    // Close communications with the device
    printf("Cleaning up...\n");
    freespace_closeDevice(device);

    // Cleanup the library
    freespace_exit();

    return 0;
}
