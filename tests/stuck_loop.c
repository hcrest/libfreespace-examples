//Flow of test:
//Flush the Mouse 
//Init libfreespace
//Send DMC to enable body frame
//Read messages
//Tell user to hold down button
//Stop reading
//Enable mouse movement
//Close device / freespace
//Tell user to release button
//start up freespace
//send DMC to enable body frame
//Read Messages
//Should be stuck if bad..

#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>
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
#include <pthread.h>
#include <unistd.h>
#endif


//close freespace and the device
static int cleanupAndCloseDevice(FreespaceDeviceId device) {
    printf("Sending message to enable mouse motion data.\n");
    struct freespace_DataMotionControl d;
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
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
    } else {
        printf("Could not encode message.\n");
    }

    freespace_closeDevice(device);
    freespace_exit();

    return rc;
}



//init freespace and open a device for body frame read
static int initFreeSpaceAndOpenDevice(FreespaceDeviceId* device) {
    int numIds = 0;
    freespace_init();
    
    int rc = freespace_getDeviceList(device, 1, &numIds);
    if (numIds == 0) {
        printf("Didn't find any devices.\n");
	freespace_exit();
        return -1;
    }

    printf("Found a device. Trying to open it...\n");
    rc = freespace_openDevice(*device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error opening device: %d\n", rc);
	freespace_exit();
        return rc;
    }

    // Display the device information.
    printDeviceInfo(*device);

    rc = freespace_flush(*device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Error flushing device: %d\n", rc);
	freespace_closeDevice(*device);
	freespace_exit();
        return rc;
    }

    printf("Sending message to enable body-frame motion data.\n");
    struct freespace_DataMotionControl d;
    d.enableBodyMotion = 1;
    d.enableUserPosition = 0;
    d.inhibitPowerManager = 0;
    d.enableMouseMovement = 0;
    d.disableFreespace = 0;

    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    rc = freespace_encodeDataMotionControl(&d, (int8_t*) buffer, sizeof(buffer));
    if (rc > 0) {
        rc = freespace_send(*device, buffer, rc);
        if (rc != FREESPACE_SUCCESS) {
            printf("Could not send message: %d.\n", rc);
        }
    } else {
        printf("Could not encode message.\n");
    }
    
    return FREESPACE_SUCCESS;
}

#define READ_COUNT_LIMIT 50

static int flushMouse(char* mouseToCat) {
    int fd = -1;
    pid_t catPid = fork();
    if (catPid == 0) {
        //child - exec the cat
        char* argv[3];
	argv[0] = "cat";
	argv[1] = mouseToCat;
	argv[2] = NULL;
        printf("Flushing mouse %s\n", mouseToCat);
	fd = open("/dev/null", O_WRONLY | O_APPEND);
	dup2(fd, STDOUT_FILENO);
	dup2(fd, STDERR_FILENO);
        int res = execvp("/bin/cat", argv);
	printf("res: %d\n", res);
	close(fd);
	fd = -1;
	_exit(0);
    } else if (catPid > 0) {
        //parent
        printf("Parent sleeping for 5 seconds before killing child\n");
#ifdef WIN32
    Sleep(5);
#else
    usleep(5000000);
    kill(catPid, SIGINT);
#endif
	if (fd != -1) {
	    close(fd);
	}
    } else {
        printf("Error forking.  Might be stuck.\n");
    }
    return 0;
}

int main(int argc, char** argv) {
    FreespaceDeviceId device;

    if (argc < 2) {
        printf("First argument must be path of the event-mouse to flush\n");
	return 1;
    }
    addControlHandler();

    printf("Please make sure the loop is the only mouse device.\n");
    
    printf("Flushing the mouse - DO NOT HOLD THE LOOP!\n");
    flushMouse(argv[1]);

    int rc = initFreeSpaceAndOpenDevice(&device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Initializing and opening device failed.  Abort.\n");
	return 1;
    }
    
    printf("Please pickup the loop, hold a button, and keep moving it around\n");

    //now we read messages
    char buffer[FREESPACE_MAX_INPUT_MESSAGE_SIZE];
    int readCount = 0;    
    int timeoutCount = 0;
    int length = 0;
    int errCount = 0;
    struct freespace_message decoded;
    while (!quit && readCount < READ_COUNT_LIMIT) {
        if (timeoutCount > 10) {
	    printf("Timed out 10 times in a row.  Abort.\n");
	    cleanupAndCloseDevice(device);
	    return 1;
        }
	if (errCount > 10) {
	    printf("Erred 10 times in a row.  Abort.\n");
	    cleanupAndCloseDevice(device);
	    return 1;
	}
	//printf("read\n");
	rc = freespace_read(device, buffer, FREESPACE_MAX_INPUT_MESSAGE_SIZE, 1000, &length);
	//printf("read out\n");
	if (rc == FREESPACE_ERROR_TIMEOUT) {
	    printf("timeout\n");
	    ++timeoutCount;
	    readCount = 0;
	} else {
	    timeoutCount = 0;
	    rc = freespace_decode_message((int8_t*) buffer, length, &decoded);
	    if (rc != FREESPACE_SUCCESS) {
	        printf("Erred\n");
	        errCount++;
		readCount = 0;
	    } else {
	        errCount = 0;
		//freespace_printMessage(stdout, buffer, length);
		if (decoded.messageType == FREESPACE_MESSAGE_BODYFRAME) {
		    if (decoded.bodyFrame.button1) {
		        ++readCount;
		    }
		}
	    }
	}
    }

    if (quit) {
        printf("Cleaning up.\n");
	cleanupAndCloseDevice(device);
	return 1;
    }

    //here means our readCount limit was reached
    //now we cleanup and close and then reopen
    cleanupAndCloseDevice(device);
        
    printf("****\nOK! Please release the button.. you have 5 seconds\n");
    printf("Please keep moving the mouse until the test exits (or tells you to)\n");
#ifdef WIN32
    Sleep(5);
#else
    usleep(5000000);
#endif

    //open it again
    rc = initFreeSpaceAndOpenDevice(&device);
    if (rc != FREESPACE_SUCCESS) {
        printf("Initializing and opening device (2nd time) failed. Abort\n");
	return 1;
    }
    
    //read now..
    readCount = 0;
    errCount = 0;
    timeoutCount = 0;
    int displayedWarning = 0;
    while (!quit && readCount < READ_COUNT_LIMIT) {
        if (timeoutCount > 10) {
	    printf("Timed out 10 times in a row.  Fail!\n");
	    cleanupAndCloseDevice(device);
	    return 1;
        }
	if (errCount > 10) {
	    printf("Erred 10 times in a row.  Abort.\n");
	    cleanupAndCloseDevice(device);
	    return 1;
	}
	if (!displayedWarning) {
	    printf("If you see only this line for over 10 seconds, the loop is stuck and CTRL-C is your only option.\n");
	    displayedWarning = 1;
	}
	rc = freespace_read(device, buffer, FREESPACE_MAX_INPUT_MESSAGE_SIZE, 1000, &length);
	if (rc == FREESPACE_ERROR_TIMEOUT) {
	    ++timeoutCount;
	    readCount = 0;
	} else {
	    timeoutCount = 0;
	    rc = freespace_decode_message((int8_t*) buffer, length, &decoded);
	    if (rc != FREESPACE_SUCCESS) {
	        errCount++;
		readCount = 0;
	    } else {
	        errCount = 0;
		if (decoded.messageType == FREESPACE_MESSAGE_BODYFRAME) {
		    ++readCount;
		}
	    }
	}
    }

    if (!quit) {
        printf("Read %d body frames.  Passed!\n", readCount);
    }

    cleanupAndCloseDevice(device);

    return 0;
}
