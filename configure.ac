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

AC_INIT([libfreespace-examples],
        [0.6])
AC_CONFIG_AUX_DIR(config)
AM_CONFIG_HEADER(config.h)
AM_INIT_AUTOMAKE()

AC_PROG_CC
AC_PROG_INSTALL

AC_STDC_HEADERS
AC_PROG_RANLIB

# Check for libfreespace
AC_CHECK_HEADERS([freespace/freespace.h], [], [AC_MSG_FAILURE([*** freespace.h header not found. Check CPPFLAGS])])
AC_CHECK_LIB(freespace, freespace_init, [], [AC_MSG_FAILURE([*** libfreespace not found. Check LDFLAGS])])

# Check for poll support for motion_example_async
AC_CHECK_HEADERS([poll.h])

# Enable CPP tests
AC_ARG_WITH(tests, [  --with-tests Build C++ Tests])

AC_CONFIG_FILES([
   Makefile
   README
   doc/Makefile
   m4/Makefile
   src/Makefile
   win/Makefile
   win/vs2005/Makefile
   win/vs2008/Makefile
   win/vs2010/Makefile
])

if test "$with_tests" == "yes"
then
   TESTS_DIR=tests
   AC_CONFIG_FILES([
      tests/Makefile
   ])   
fi
AC_SUBST([TESTS_DIR])

AC_OUTPUT
