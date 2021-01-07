/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_AM4377_TCP_HARDWARE_H_
#define ROS_AM4377_TCP_HARDWARE_H_

#include <sys/time.h>
#include "EthernetClient.h"

class AM4377Hardware {
public:
  AM4377Hardware()
  {
  }

  void setConnection(unsigned char a,unsigned char b,unsigned char c,unsigned char d, int port = 11411)
  {
	addr_a = a;
	addr_b = b;
	addr_c = c;
	addr_d = d;
	serverPort_ = port;
  }

  void init()
  {
    	if(tcp_.connected())
	{
		tcp_.stop();
    	}
	tcp_.begin();
	tcp_.connect(addr_a, addr_b, addr_c, addr_d, serverPort_);
  }

  int read()
  {
	if(tcp_.connected())
	{
		return tcp_.read();
	}
	else
	{
		tcp_.stop();
		tcp_.begin();
		tcp_.connect(addr_a, addr_b, addr_c, addr_d, serverPort_);
	}
    	return -1;
  }

  void write(const uint8_t* data, int length)
  {
    tcp_.write(data, length);
  }

  unsigned long time()
  {
	unsigned long msec = 0;
    	struct timeval msectime;
    	gettimeofday(&msectime, NULL);
    	msec =msectime.tv_sec * 1000 + msectime.tv_usec / 1000;
	return msec;
  }

  bool connected()
  {
    return tcp_.connected();
  }

protected:
  EthernetClient tcp_;
  unsigned char addr_a;
  unsigned char addr_b;
  unsigned char addr_c;
  unsigned char addr_d;
  uint16_t serverPort_;
};

#endif
