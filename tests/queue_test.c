
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include <freespace/freespace_printers.h>
#include "appControlHandler.h"

#ifdef WIN32
#include "win32/stdafx.h"
#include <windows.h>
#include <stdio.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif


static int errorExit(const char* message, int rc) {
    printf("%s : %d\n", message, rc);
    return 1;
}

static void printUnknown(const char* name, const char* buffer, int length) {
    int i;
    printf("%s(", name);
    for (i = 0; i < length; ++i) {
        printf("%02x ", (uint8_t) buffer[i]);
    }
    printf(")\n");
}

static void printStats(FreespaceDeviceId device, char* buffer, int expectedMessageType) {
    //vars
    int rc;
    struct timeval loopStart;
    int foundMessage = 0;
    int length = 0;
    int timeup = 0;
    int startSec;
    struct freespace_message msg;

    
    //we know the buffer is big enough here..
    buffer[0] = 0x07;
    buffer[1] = 0x16;
    buffer[2] = 0x01;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;

    //send it
    rc = freespace_send(device, buffer, 8);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error sending loop request %d\n", rc);
	return;
    }
    
    
    gettimeofday(&loopStart, NULL);
    startSec = (int) loopStart.tv_sec;
    //read it at most 5 seconds
    while (!timeup && !foundMessage) {
        do {
            rc = freespace_read(device, buffer, FREESPACE_MAX_INPUT_MESSAGE_SIZE, 0, &length);
	    if (rc == FREESPACE_SUCCESS) {
	        if (length == 27 && buffer[0] == 0x08 && buffer[1] == 0x16 && buffer[2] == 0x01) {
		    //print it
		    printf("Printing Loop response\n");
		    printUnknown("Loop response", buffer, length);
		    foundMessage = 1;
		    break;
		} else {
		    rc = freespace_decode_message((int8_t*) buffer, length, &msg);
		    if (msg.messageType == expectedMessageType) {
		        printf("GOT THE MESSAGE I MISSED!!!\n");
		    }
		}
	    }
	} while (rc == FREESPACE_SUCCESS);

	if (!foundMessage) {
	    gettimeofday(&loopStart, NULL);
	    if (loopStart.tv_sec - startSec > 5) {
	        timeup = 1;
	    }
	}
    }

    if (!foundMessage) {
        printf("Received no loop response..\n");
    }

    //now for the other
    buffer[0] = 0x07;
    buffer[1] = 0x16;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;

    //send it
    rc = freespace_send(device, buffer, 8);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error sending loop request %d\n", rc);
	return;
    }

    foundMessage = 0;
    length = 0;
    timeup = 0;
    gettimeofday(&loopStart, NULL);
    startSec = (int) loopStart.tv_sec;
    //read it at most 5 seconds
    while (!timeup && !foundMessage) {
        do {
            rc = freespace_read(device, buffer, FREESPACE_MAX_INPUT_MESSAGE_SIZE, 0, &length);
	    if (rc == FREESPACE_SUCCESS) {
	        if (length == 27 && buffer[0] == 0x08 && buffer[1] == 0x16 && buffer[2] == 0) {
		    //print it
		    printf("Printing Dongle response\n");
		    printUnknown("Dongle Response", buffer, length);
		    foundMessage = 1;
		    break;
		}
	    }
	} while (rc == FREESPACE_SUCCESS);

	if (!foundMessage) {
	    gettimeofday(&loopStart, NULL);
	    if (loopStart.tv_sec - startSec > 5) {
	        timeup = 1;
	    }
	}
    }

    if (!foundMessage) {
        printf("Did not read the dongle response\n");
    }
}

static int cleanup(FreespaceDeviceId device, int closeDev, const char* message, int rc) {
    if (closeDev) {
        char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
        struct freespace_DataMotionControl d;
        d.enableBodyMotion = 0;
	d.enableUserPosition = 0;
	d.inhibitPowerManager = 0;
	d.enableMouseMovement = 1;
	d.disableFreespace = 0;
	int rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
	if (rc > 0) {
            rc = freespace_send(device, buffer, rc);
	    if (rc != FREESPACE_SUCCESS) {
                printf("Could not send message: %d.\n", rc);
	    }
	}
        freespace_closeDevice(device);
    }
    freespace_exit();
    return errorExit(message, rc);
}

static int sendControlRequest(FreespaceDeviceId device, char* messageBuffer, int msgBuffLen, int useProductId) {
    int rc = 0;
    if (!useProductId) {
        rc = freespace_encodeBatteryLevelRequest((int8_t*) messageBuffer, msgBuffLen);
    } else {
        rc = freespace_encodeProductIDRequest((int8_t*) messageBuffer, msgBuffLen);
    }
    if (rc > 0) {
        rc = freespace_send(device, messageBuffer, rc);       
    } else {
        printf("Error encoding control level msg\n");
    }
    return rc;
}

//QueueTest <sleep ms> --enable-user --enable-body --enable-control
int main(int argc, char** argv) {

    if (argc < 2) {
        printf("First argument must be time in ms (minimum 25ms) for sleep\n");
	printf("Optional: --enable-user --enable-body --enable-control\n");
        return 1;
    }

    //configuration
    int sleepIntervalMs = atoi(argv[1]);
    int enableControl = 0;
    int enableUserFrame = 0;
    int enableBodyFrame = 0;
    int enableMouse = 0;
    int useProductId = 0;
    //default is 50ms delay
    int controlMessageDelayMS = 50;
    int i;

    
    if (sleepIntervalMs < 0) {
        printf("Forcing sleep Interval to 1 ms");
	sleepIntervalMs = 1;
    }
   

    //hardcoded time in seconds for the test
    //could change it to be an input arg
    int testTime = 10;

    addControlHandler();

    for (i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--enable-user") == 0) {
	    enableUserFrame = 1;	  
        } else if (strcmp(argv[i], "--enable-body") == 0) {
	    enableBodyFrame = 1;
        } else if (strcmp(argv[i], "--enable-control") == 0) {
	    enableControl = 1;
        } else if (strcmp(argv[i], "--enable-mouse") == 0) {
	    enableMouse = 1;
	} else if (strcmp(argv[i], "--enable-productId") == 0) {
	    useProductId = 1;
	    enableControl = 1;
	} else if (strncmp(argv[i], "--control-delay=", 16) == 0) {
	    //make sure its good..
	    char* ctrlDelay = argv[i];
	    if (strlen(ctrlDelay) > 16) {
	        ctrlDelay += 16;
		controlMessageDelayMS = atoi(ctrlDelay);
		enableControl = 1;
	    }
	}
	
    }
    
    
    printf("Beginning test with parameters:\n  Sleep ms: %d, Enable Body Frame: %d, Enable User Frame: %d, Enable Mouse: %d\n",
	   sleepIntervalMs, enableBodyFrame, enableUserFrame, enableMouse);
    printf("  Enable Control: %d, Use product Id: %d, Min delay b/t control sends: %dms\n", 
	   enableControl, useProductId, controlMessageDelayMS);
    //init freespace
    int rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        return errorExit("Error initializing freespace", rc);
    }

    //find a freespace dev
    FreespaceDeviceId device;
    int numIds = 0;
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (rc != FREESPACE_SUCCESS) {
        return cleanup(device, 0, "Error retrieving device list", rc);
    }
    if (numIds == 0) {
        return cleanup(device, 0, "Found no devices", 0);
    }
    
    //open the device
    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        return cleanup(device, 0, "Error opening device", rc);
    }

    //flush it
    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        return cleanup(device, 1, "Error flushing device", rc);
    }

    //enable body frame and user frame
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    struct freespace_DataMotionControl dmc;
    dmc.enableBodyMotion = enableBodyFrame;
    dmc.enableUserPosition = enableUserFrame;
    dmc.inhibitPowerManager = 1;
    dmc.enableMouseMovement = enableMouse;
    dmc.disableFreespace = 0;

    rc = freespace_encodeDataMotionControl(&dmc, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(device, buffer, rc);
	if (rc != FREESPACE_SUCCESS) {	
	    return cleanup(device, 1, "Error sending Data Motion Control msg", rc);
	}
    } else {
        return cleanup(device, 1, "Error encoding Data Motion Control msg", rc);
    }

    //if we enable both user and body frame, the seqNumber increments 
    //for the same type of message will be 2, else 1
    int expectedSequenceNumberDiff = enableBodyFrame + enableUserFrame;

    //some stats to maintain
    int numControlMsgsSent = 0;
    int numControlMsgsRecv = 0;
    int lastBodyFrameSeqNumber = -1;
    int lastUserFrameSeqNumber = -1;
    int currBodyFrameSeqNumber = -1;
    int currUserFrameSeqNumber = -1;
    int length = 0;
    int sleepUsec = sleepIntervalMs * 1000;    
    //default is to use battery
    int expectedResponsesPerRequest = 1;
    int expectedControlMessageType = FREESPACE_MESSAGE_BATTERYLEVEL;
    if (useProductId) {
        expectedResponsesPerRequest = 2;
        expectedControlMessageType = FREESPACE_MESSAGE_PRODUCTIDRESPONSE;
    }

    int ctrlMessageDelayUSec = controlMessageDelayMS * 1000;

    int numControlMessagesOutstanding = 0;
    int maxControlMessagesOutstanding = 0;
    int numControlMessageSendSkipped = 0;

    struct freespace_message msg;

    //loop control
    int timeup = 0;
    struct timeval loopStartTime;
    struct timeval currTime;

    //the control message time if needed
    struct timeval lastControlSendTime;

    if (enableControl) {
        rc = sendControlRequest(device, buffer, sizeof(buffer), useProductId);
	if (rc != FREESPACE_SUCCESS) {
	    return cleanup(device, 1, "Unable to send control request", rc);
	}
	++numControlMsgsSent;	
	numControlMessagesOutstanding += expectedResponsesPerRequest;
	//always one on the first shot
	maxControlMessagesOutstanding += expectedResponsesPerRequest;
        gettimeofday(&lastControlSendTime, NULL);
    }

    printf("Beginning test for about %d seconds\n", testTime);
    printf("The test assumes constant movement of the Freespace device\n");

    gettimeofday(&loopStartTime, NULL);
    int errCount = 0;

    //we should get at least 3 messages in a tight loop
    //since first could be the old message in the HW register
    //with some unknown sequence # / timestamp.
    //
    //sequence #s are incremented based on the 125 Hz clock
    //of the freespace device.  Can't expect the message with
    //sequence # N to be followed by N + 1 after a timeout/sleep
    int numBodyFrameMessages = 0;
    int numUserFrameMessages = 0;

    while (!quit && !timeup) {
        //read until we'd time out
        numBodyFrameMessages = 0;
	numUserFrameMessages = 0;
	do {
	    rc = freespace_read(device, buffer, sizeof(buffer), 0, &length);
	    if (rc == FREESPACE_SUCCESS) {
	        rc = freespace_decode_message((int8_t*) buffer, length, &msg);
		if (rc != FREESPACE_SUCCESS) {
		    printf("Error decoding message.\n");
		    freespace_printMessage(stdout, buffer, length);
		    //set it to SUCCESS so we don't exit the loop
		    rc = FREESPACE_SUCCESS;
		    continue;
		}
	        if (msg.messageType == FREESPACE_MESSAGE_BODYFRAME && enableBodyFrame) {
		    lastBodyFrameSeqNumber = currBodyFrameSeqNumber;
	            currBodyFrameSeqNumber = msg.bodyFrame.sequenceNumber;
		    ++numBodyFrameMessages;
		    //printf("bf seq# %d\n", msg.bodyFrame.sequenceNumber);
	        } else if (msg.messageType == FREESPACE_MESSAGE_USERFRAME && enableUserFrame) {
		    lastUserFrameSeqNumber = currUserFrameSeqNumber;
	            currUserFrameSeqNumber = msg.userFrame.sequenceNumber;
		    ++numUserFrameMessages;
		    //printf("uf seq# %d\n", msg.userFrame.sequenceNumber);
	        } else if (msg.messageType == expectedControlMessageType && enableControl) {
	            ++numControlMsgsRecv;
		    numControlMessagesOutstanding -= expectedResponsesPerRequest;
		    if (numControlMessagesOutstanding < 0) {
		        numControlMessagesOutstanding = 0;
		    }
	        } else {
	            //here means we got an unexpected message
	            //return cleanup(device, 1, "Received unexpected message", msg.messageType);
		    printf("Received unexpected message %d\n", msg.messageType);
		    continue;
	        }
	    }
	    //else we break out
	} while (!quit && rc == FREESPACE_SUCCESS);
	
	//evaluate the rc from the loop that just exited
	if (rc == FREESPACE_ERROR_TIMEOUT) {
	    //make sure the differentials are as expected.
	    if (enableBodyFrame && numBodyFrameMessages >= 3 &&
	    (currBodyFrameSeqNumber - lastBodyFrameSeqNumber) != expectedSequenceNumberDiff) {
	        return cleanup(device, 1, "Unexpected body frame sequence differential after timeout", currBodyFrameSeqNumber - lastBodyFrameSeqNumber);
	    }
	    if (enableUserFrame && numUserFrameMessages >= 3 &&
		(currUserFrameSeqNumber - lastUserFrameSeqNumber) != expectedSequenceNumberDiff) {
	        return cleanup(device, 1, "Unexpected user frame sequence differential after timeout", currUserFrameSeqNumber - lastUserFrameSeqNumber);
	    }

	} else {
	    ++errCount;
	    if (errCount == 10) {
	        return cleanup(device, 1, "Error during read", rc);
	    } else {
	        printf("Read Error #%d (%d)\n", errCount, rc);
	        continue;
	    }
	}

	//sleep and send the control message if we need to
	//we'll only send them at most once every 50ms
#ifdef WIN32
    Sleep(1);
#else
    usleep(sleepUsec);
#endif

	gettimeofday(&currTime, NULL);
	if (enableControl && numControlMessagesOutstanding < (9 * expectedResponsesPerRequest)) {	    
	    long secDiff = currTime.tv_sec - lastControlSendTime.tv_sec;
	    long usecDiff = currTime.tv_usec - lastControlSendTime.tv_usec;
	    if (usecDiff < 0) {
	        usecDiff += 1000000;
		--secDiff;
	    }
	    usecDiff += 1000000 * (secDiff);
	    if (usecDiff > ctrlMessageDelayUSec) {
	        //send another 
	        rc = sendControlRequest(device, buffer, sizeof(buffer), useProductId);
		if (rc != FREESPACE_SUCCESS) {
		    return cleanup(device, 1, "Error sending control request", rc);
		}
		++numControlMsgsSent;
		numControlMessagesOutstanding += expectedResponsesPerRequest;
		if (numControlMessagesOutstanding > maxControlMessagesOutstanding) {
		    maxControlMessagesOutstanding = numControlMessagesOutstanding;
		}
		lastControlSendTime = currTime;
	    }
	} else if (enableControl && numControlMessagesOutstanding >= (9 * expectedResponsesPerRequest)) {
	    ++numControlMessageSendSkipped;
	}
	if (currTime.tv_sec - loopStartTime.tv_sec > testTime) {
	    timeup = 1;
	}
    }

    //leave a little cleanup time to receive any control
    //messages

    //hardcoded but could be an input arg
    int cleanupTimeSec = 1;
    if (enableControl) {
        printf("Finishing up and reading for %d seconds to receive any control messages\n", cleanupTimeSec);
	//here we should try to finish reading the rest of the control messages
	//read for about 5 second or so - currTime should have the approximate loop end
	timeup = 0;
	long startSec = currTime.tv_sec;
	while (!quit && !timeup) {
            //read until we timeout
	    do {
	        rc = freespace_read(device, buffer, sizeof(buffer), 0, &length);
		if (rc == FREESPACE_SUCCESS) {
		    rc = freespace_decode_message((int8_t*) buffer, length, &msg);
		    if (rc == FREESPACE_SUCCESS) {
		        //We really only care about the control messages in this loop
		        if (msg.messageType == expectedControlMessageType) {
			    ++numControlMsgsRecv;			    
			    numControlMessagesOutstanding -= expectedResponsesPerRequest;
			    if (numControlMessagesOutstanding < 0) {
			        numControlMessagesOutstanding = 0;
			    }
			}		
		    } else {
		        printf("Error decoding message.\n");
			freespace_printMessage(stdout, buffer, length);
			//set it to SUCCESS so we don't exit the loop
			rc = FREESPACE_SUCCESS;
			continue;
		    }
		}
		//else we'll break out of the inner loop
	    } while (!quit && rc == FREESPACE_SUCCESS);
	    
	    gettimeofday(&currTime, NULL);
	    if (currTime.tv_sec - startSec > cleanupTimeSec) {
	        timeup = 1;
	    }
	}
    }
   
    //print the stats
    printf("********\n");
    if (enableBodyFrame || enableUserFrame) {
        printf("Expecting sequence # difference of %d between similar types\n", expectedSequenceNumberDiff);
	if (enableBodyFrame) {
	    printf("Last two body frame sequence nums: %d %d - %s\n", lastBodyFrameSeqNumber, currBodyFrameSeqNumber,
		   (currBodyFrameSeqNumber - lastBodyFrameSeqNumber) == expectedSequenceNumberDiff ? "Pass" : "Fail" );
	}
	if (enableUserFrame) {
	    printf("Last two user frame sequence nums: %d %d - %s\n", lastUserFrameSeqNumber, currUserFrameSeqNumber,
		   (currUserFrameSeqNumber - lastUserFrameSeqNumber) == expectedSequenceNumberDiff ? "Pass": "Fail" );
	}
    }

    if (enableControl) {
        printf("\nExpecting %d Control responses per request\n", expectedResponsesPerRequest);
        printf("Sent %d control messages and received %d responses - %s\n", numControlMsgsSent, numControlMsgsRecv,
	       (numControlMsgsSent * expectedResponsesPerRequest) == numControlMsgsRecv ? "Pass" : "Fail" );
	printf("Max # of outstanding control messages: %d\n", maxControlMessagesOutstanding);
	printf("Number of times we skipped sending the control message: %d\n", numControlMessageSendSkipped);
    }
    printf("********\n");

    if (enableControl && numControlMsgsSent * expectedResponsesPerRequest != numControlMsgsRecv) {
        printf("Failed control test - collecting stats\n");
	printStats(device, buffer, expectedControlMessageType);
    }
    cleanup(device, 1, "Test finished.", 0);
    return 0;

}

