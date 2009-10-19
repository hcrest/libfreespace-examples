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

#ifndef FREESPACE_MATH_H_
#define FREESPACE_MATH_H_


#include <freespace/freespace.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////

#define PI 3.1415927f

#define RADIANS_TO_DEGREES(rad) ((float)rad * 180.0f / PI)
#define DEGREES_TO_RADIANS(deg) ((floag)deg * PI / 180.0f)

////////////////////////////////////////////////////////////////
// Float math
#define fabsf(a)      ((float) fabs((double) a))
#define fsqrtf(a)     ((float) sqrt((double) a))
#define fsinf(a)      ((float) sin((double) a))
#define fcosf(a)      ((float) cos((double) a))
#define ftanf(a)      ((float) tan((double) a))
#define facosf(a)     ((float) acos((double) a))
#define fasinf(x,y)   ((float) asin((double) x, (double) y))
#define fatan2f(x, y) ((float) atan2((double) x, (double) y))
#define fmodf(x,y)    ((float) fmod((double) x, (double) y))
////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_MATH_H_ */
