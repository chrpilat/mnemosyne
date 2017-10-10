/**
 * @copyright
 * Copyright (c) 2017 - SLD Group @ Columbia University. All Rights Reserved.
 *
 * This file is part of Mnemosyne.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   time_utils.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Functions for timing purposes.
 *
 */
#ifndef _TIME_UTILS_HPP_
#define _TIME_UTILS_HPP_

#include <boost/lexical_cast.hpp>

#include <sys/times.h>
#include <unistd.h>

#if defined(_SC_CLK_TCK)
#define TIMES_TICKS_PER_SEC sysconf(_SC_CLK_TCK)
#elif defined(CLK_TCK)
#define TIMES_TICKS_PER_SEC CLK_TCK
#elif defined(HZ)
#define TIMES_TICKS_PER_SEC HZ
#else
#define TIMES_TICKS_PER_SEC 60
#endif

inline long int p_cpu_time()
{
   long t;
   struct tms now;
   clock_t    ret = times(&now);
   if (ret == static_cast<clock_t>(-1))
      now.tms_utime = now.tms_stime = now.tms_cutime = now.tms_cstime = ret = 0;
   t = (long(now.tms_utime) * 1000) / (TIMES_TICKS_PER_SEC) + (long(now.tms_cutime) * 1000) / (TIMES_TICKS_PER_SEC);
   return t;
}

inline std::string print_cpu_time(long int t)
{
   std::string ost;
   ost = boost::lexical_cast<std::string>(t / 1000) + ".";
   long centisec = (t % 1000) / 10;
   if (centisec < 10)
      ost += "0" + boost::lexical_cast<std::string>(centisec);
   else
      ost += boost::lexical_cast<std::string>(centisec);
   return ost;
}

#define START_TIME(time_var) time_var=p_cpu_time()

#define STOP_TIME(time_var) time_var=p_cpu_time()-(time_var)

#endif
