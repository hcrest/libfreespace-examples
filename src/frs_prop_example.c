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
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#define Sleep(x) usleep((x)*1000)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

#include <string.h>

#define BUFFER_LENGTH 1024

struct deviceInfo {
	FreespaceDeviceId id;
	int deviceType;
	int recordIndex;
	struct deviceInfo* next;
};

struct deviceInfo* deviceInfoList = NULL;

int sendFRSReadRequest(FreespaceDeviceId device, 
					   struct deviceInfo* pDeviceInfo);

/**
 * Sends an FRS read request based on the given FRS message type
 * @param device the device the message is from
 * @param pDeviceInfo structure to hold device information and records
 * @param message The freespace_message to parse
 */
int handleFRSData(FreespaceDeviceId device, 
				  struct deviceInfo* pDeviceInfo, 
				  struct freespace_message message) {
	int i = 0;	// loop vars
	struct deviceInfo* iter;
	int readCompleted = 0;

	printf("    FRS 0x%X : ", message.fRSReadResponse.FRStype);
	switch(message.fRSReadResponse.status) {
	case 0: printf("No error.\n"); break;
	case 1: printf("Unrecognized FRS type.\n"); break;
	case 2: printf("Busy.\n"); break;
	case 3: 
		printf("Read completed.\n"); 
		readCompleted = 1;
		break;
	case 4: printf("Offset out of range.\n"); break;
	case 5: printf("Record empty.\n"); break;
	case 6: 
		printf("Read block completed.\n");
		readCompleted = 1;
		break;
	case 7: 
		printf("Read block completed and read record completed.\n"); 
		readCompleted = 1;
		break;
	default: printf("Not recognized.\n");
	}
	if (readCompleted) {
		if (message.fRSReadResponse.dataLength > 0) {
			printf("    Data: 0x%X", message.fRSReadResponse.data[0]);
			for (i = 1;i < message.fRSReadResponse.dataLength;i++) {
				printf(", 0x%X", message.fRSReadResponse.data[i]);
			}
			printf("\n");
		}
		for (iter = deviceInfoList;iter != NULL;iter = iter->next) {
			if (iter->id == device) {
				iter->recordIndex++;
				return sendFRSReadRequest(device, iter);
			}
		}
	}
	return FREESPACE_SUCCESS;
}

/**
 * Sends an FRS read request based on the given FRS message type
 * @param device the device the message is from
 * @param pDeviceInfo structure to hold device information and records
 * @param FRSType the type of record to request
 * @param result FREESPACE_SUCCESS if a packet was received; else error code
 */
int sendFRSReadRequest(FreespaceDeviceId device, 
					   struct deviceInfo* pDeviceInfo) {
	// Create and send a FRS read request message to the device
	struct freespace_message message ;
	uint16_t* pBlockSize = NULL;
	uint16_t* pFRSType = NULL;
	uint16_t* pOffset = NULL;

    memset(&message, 0, sizeof(struct freespace_message)); // Start with a clean message struct

	if (pDeviceInfo->deviceType != 1 && pDeviceInfo->deviceType != 2) // Unknown device
		return FREESPACE_ERROR_INVALID_DEVICE; // Indicates an error

	message.messageType = FREESPACE_MESSAGE_FRSREADREQUEST;
	pBlockSize = &message.fRSReadRequest.BlockSize;
	pFRSType = &message.fRSReadRequest.FRStype;
	pOffset = &message.fRSReadRequest.readOffset;

	// Next we fill the message with our request
	switch (pDeviceInfo->recordIndex) {
	case 0:	// Tracking number record
		printf("Getting tracking number...\n");
		*pBlockSize = 5;
		*pFRSType = 0x4B4B;
		*pOffset = 0;
		break;
	case 1:	// SCD record
		printf("Getting scd record...\n");
		*pBlockSize = 5;
		*pFRSType = 0x7979;
		*pOffset = 1;
		break;
	case 2:	// Enable record
		printf("Getting enable record...\n");
		*pBlockSize = 5;
		*pFRSType = 0x5627;
		*pOffset = 1;
		break;
	default:
		*pBlockSize = 5;
	}

	if (pDeviceInfo->deviceType == 1) {		// Means it is a dongle
		// To communicate with a dongle set the dest field to 1
		message.dest = 1;
		return freespace_sendMessageAsync(device, &message, 100, NULL, 0);
	} else {
		// Allow message.dest to have the default value of 0. This will cause the message
		// to go to the remote or the module
		return freespace_sendMessageAsync(device, &message, 100, NULL, 0);
	}
}

/**
 * Callback that handles product ID response messages received from devices
 * @param id the device the message is from
 * @param message a pointer to the message to send
 * @param cookie not used in this example
 * @param result FREESPACE_SUCCESS if a packet was received; else error code
 */
static void receiveMessageCallback(FreespaceDeviceId id,
                                   struct freespace_message* message,
                                   void* cookie,
                                   int result) {
                                       
    struct freespace_ProductIDResponse* pr = &(message->productIDResponse);
	struct deviceInfo* thisDevice;
	struct deviceInfo* iter;
	int rc = 0;
    
    if (result == FREESPACE_SUCCESS && message != NULL) {
        if (message->messageType == FREESPACE_MESSAGE_PRODUCTIDRESPONSE) {
            printf("Received product ID response message from device ID: %d\n", id);
            if (pr->deviceClass == 1) {
                printf("    Device class:   dongle\n");
            } else if (pr->deviceClass == 2) {
                printf("    Device class:   handheld\n");
            } else {
                printf("    Device class:   unknown\n");
            }	


			thisDevice =  (struct deviceInfo*)malloc(sizeof(struct deviceInfo));
			memset(thisDevice, 0, sizeof(struct deviceInfo));
			thisDevice->id = id;
			thisDevice->deviceType = pr->deviceClass;
			thisDevice->recordIndex = 0;
			thisDevice->next = NULL;
			printf("Beginning FRS Property reading.\n");
			rc = sendFRSReadRequest(id, thisDevice);
			if (rc == FREESPACE_SUCCESS) {
				if (deviceInfoList == NULL) {
					deviceInfoList = thisDevice;
				} else {
					for(iter = deviceInfoList;iter->next != NULL;iter = iter->next);
					iter->next = thisDevice;
				}
			} else {
                printf("Error sending FRS read request. Error code %d\n", rc);
                free(thisDevice);
            }
		} else if (message->messageType == FREESPACE_MESSAGE_FRSREADRESPONSE) {
			for(iter = deviceInfoList;iter->next != NULL;iter = iter->next) {
				if (iter->id == id) {
					break;
				}
			}
			if (iter->id == id) {
				handleFRSData(id, iter, *message);
			}
		}
    } else if (result == FREESPACE_ERROR_NO_DATA) {
        printf("Message with no data received from device ID %d.\n", id);
    } else {
        printf("Problem with message received from device ID %d.\n", id);
    }
}


/**
 * Callback that 
 *  - displays the devices that have been inserted into and removed from the system.
 *  - requests the firmware version of any devices that have been inserted into the system.
 * Implements freespace_hotplugCallback
 * @param event The type of event.
 * @param id The affected device.
 * @param cookie Not used for this application.
 */
void hotplugCallback(enum freespace_hotplugEvent event,
                     FreespaceDeviceId id,
                     void* cookie) {
    int rc;
    struct freespace_message message;
	struct FreespaceDeviceInfo info;
	struct deviceInfo* iter;
	struct deviceInfo* del;

    switch (event) {

        case FREESPACE_HOTPLUG_INSERTION:
            // Get and print USB HID information about the device.
            printf("Device Inserted: %d\n", id);
            rc = printDeviceInfo(id);
            if (rc != FREESPACE_SUCCESS) {
                printf("Could not display device info: %d\n", rc);
                return;
            }

            // Open the device to be able to communicate with it
            rc = freespace_openDevice(id);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error opening device.\n");
                return;
            }

			rc = freespace_getDeviceInfo(id, &info);
			if (rc != FREESPACE_SUCCESS) {
				printf("    Error getting device info. Error code %d\n", rc);
				return;
			}
			printf("    HID Version = %d\n", info.hVer);

            // Set the handler that handles messages received from the device.
            freespace_setReceiveMessageCallback(id, receiveMessageCallback, NULL);

            // Create and send a product ID request message to the device
            memset(&message, 0, sizeof(message)); // Start with a clean message struct
            message.messageType = FREESPACE_MESSAGE_PRODUCTIDREQUEST;
            // Allow message.dest to have the default value of 0. This will cause the message
            // to go to the remote or the module
            rc = freespace_sendMessageAsync(id, &message, 100, NULL, 0);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error sending productID request\n");
                return;
            }
            // To communicate with a dongle set the dest field to 1
            message.dest = 1;
            rc = freespace_sendMessageAsync(id, &message, 100, NULL, 0);
            if (rc != FREESPACE_SUCCESS) {
                printf("Error sending productID request\n");
                return;
            }

            break;

        case FREESPACE_HOTPLUG_REMOVAL:
            printf("Device Removed: %d\n", id);
			if (deviceInfoList != NULL) {
				for(iter = deviceInfoList;iter->next != NULL;iter = iter->next) {
					if (iter->id == id) {	// It is the head
						deviceInfoList = iter->next;
						free(iter);
						break;
					} else if (iter->next != NULL) {	// Not the head, in the list
						if (iter->next->id == id) {
							del = iter->next;
							iter->next = iter->next->next;
							free(del);
							break;
						}
					} else {	// Not in the list

					}
				}
			}
            break;

        default:
            printf("Unrecognized freespace_hotplugEvent.\n");
            break;
    }
}

/**
 * main
 * This example uses the asynchronous API to
 *  - detect removal or additon of devices
 *  - get the product ID information from added devices
 * Devices may or may not be connected when this example is launched
 * Devices may be add/removed (pugged/unplugged) while this example is running.
 */
int main(int argc, char* argv[]) {
    int numIds;
    int deviceIds[FREESPACE_MAXIMUM_DEVICE_COUNT];
    int rc;
    
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

    // Set the callback to catch the initial devices.
    printf("Detecting the Freespace devices already connected...\n");
    freespace_setDeviceHotplugCallback(hotplugCallback, NULL);
    freespace_perform();

    printf("Waiting for Freespace devices to be inserted...\n");
    printf("Type Ctrl-C to exit\n");
    while (!quit) {
        // Easy event loop - just poll freespace_perform periodically
        // rather than waiting on select or WaitForMultipleObjects
        Sleep(100);

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
