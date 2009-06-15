# This file is part of libfreespace-examples.
# 
# Copyright (c) 2009, Hillcrest Laboratories, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions 
# are met:
# 
#     * Redistributions of source code must retain the above copyright 
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright 
#       notice, this list of conditions and the following disclaimer in 
#       the documentation and/or other materials provided with the 
#       distribution.
#     * Neither the name of the Hillcrest Laboratories, Inc. nor the names 
#       of its contributors may be used to endorse or promote products 
#       derived from this software without specific prior written 
#       permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

EXTRA_DIST = communication_example.vcproj discovery_example.vcproj motion_example.vcproj \
             motion_example_async.vcproj multiple_device_example.vcproj examples.sln stdafx.h

bin_PROGRAMS = motion_example motion_example_async discovery_example communication_example multiple_device_example gameloop_example gameloop_example2

motion_example_SOURCES = motion_example.c appControlHandler.c appControlHandler.h
motion_example_async_SOURCES = motion_example_async.c appControlHandler.c appControlHandler.h
communication_example_SOURCES = communication_example.c appControlHandler.c appControlHandler.h
discovery_example_SOURCES = discovery_example.c appControlHandler.c appControlHandler.h
multiple_device_example_SOURCES = multiple_device_example.c appControlHandler.c appControlHandler.h
gameloop_example_SOURCES = gameloop_example.c appControlHandler.c appControlHandler.h
gameloop_example2_SOURCES = gameloop_example2.c appControlHandler.c appControlHandler.h

gameloop_example_LDADD = -lpthread

LDADD = -lfreespace
AM_CFLAGS = -Wall
