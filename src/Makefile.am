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

APP_CTRL_SRCS = appControlHandler.c appControlHandler.h
MATH_SRCS = math/quaternion.c math/quaternion.h math/vec3.h

EXTRA_DIST = stdafx.h

bin_PROGRAMS = motion_example motion_example_async discovery_example communication_example multiple_device_example \
               gameloop_example gameloop_example2 game3d_example firmware_version_example

motion_example_SOURCES = motion_example.c $(APP_CTRL_SRCS)
motion_example_async_SOURCES = motion_example_async.c $(APP_CTRL_SRCS)
communication_example_SOURCES = communication_example.c $(APP_CTRL_SRCS)
discovery_example_SOURCES = discovery_example.c $(APP_CTRL_SRCS)
multiple_device_example_SOURCES = multiple_device_example.c $(APP_CTRL_SRCS)
gameloop_example_SOURCES = gameloop_example.c $(APP_CTRL_SRCS)
gameloop_example2_SOURCES = gameloop_example2.c $(APP_CTRL_SRCS)
game3d_example_SOURCES = game3d_example.c $(APP_CTRL_SRCS) $(MATH_SRCS)
firmware_version_example_SOURCES = firmware_version_example.c $(APP_CTRL_SRCS)

gameloop_example_LDADD = -lpthread
game3d_example_LDADD = -lm

LDADD = -lfreespace
AM_CFLAGS = -Wall
