/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>


#include <math.h> //fabs

namespace test_nodelet
{

class ConsoleTest : public nodelet::Nodelet
{
private:
  virtual void onInit()
  {
    NODELET_DEBUG("DEBUG output");
    NODELET_DEBUG_STREAM("DEBUG" << " output");
    NODELET_DEBUG_ONCE("DEBUG output");
    NODELET_DEBUG_STREAM_ONCE("DEBUG" << " output");
    NODELET_DEBUG_COND(1 == 1, "DEBUG output");
    NODELET_DEBUG_STREAM_COND(1 == 1, "DEBUG" << " output");
    NODELET_DEBUG_COND(1 == 0, "DEBUG output unseen");
    NODELET_DEBUG_STREAM_COND(1 == 0, "DEBUG" << " output unseen");
    NODELET_DEBUG_THROTTLE(10.0, "DEBUG output");
    NODELET_DEBUG_STREAM_THROTTLE(10.0, "DEBUG" << " output");
    //TODO TEST FILTERS
    //NODELET_DEBUG_FILTER(10.0, "DEBUG output");
    //NODELET_DEBUG_STREAM_FILTER(10.0, "DEBUG" << " output");

    NODELET_INFO("INFO output");
    NODELET_INFO_STREAM("INFO" << " output");
    NODELET_INFO_ONCE("INFO output");
    NODELET_INFO_STREAM_ONCE("INFO" << " output");
    NODELET_INFO_COND(1 == 1, "INFO output");
    NODELET_INFO_STREAM_COND(1 == 1, "INFO" << " output");
    NODELET_INFO_COND(1 == 0, "INFO output unseen");
    NODELET_INFO_STREAM_COND(1 == 0, "INFO" << " output unseen");
    NODELET_INFO_THROTTLE(10.0, "INFO output");
    NODELET_INFO_STREAM_THROTTLE(10.0, "INFO" << " output");
    //TODO TEST FILTERS
    //NODELET_INFO_FILTER(10.0, "INFO output");
    //NODELET_INFO_STREAM_FILTER(10.0, "INFO" << " output");

    NODELET_WARN("WARN output");
    NODELET_WARN_STREAM("WARN" << " output");
    NODELET_WARN_ONCE("WARN output");
    NODELET_WARN_STREAM_ONCE("WARN" << " output");
    NODELET_WARN_COND(1 == 1, "WARN output");
    NODELET_WARN_STREAM_COND(1 == 1, "WARN" << " output");
    NODELET_WARN_COND(1 == 0, "WARN output unseen");
    NODELET_WARN_STREAM_COND(1 == 0, "WARN" << " output unseen");
    NODELET_WARN_THROTTLE(10.0, "WARN output");
    NODELET_WARN_STREAM_THROTTLE(10.0, "WARN" << " output");
    //TODO TEST FILTERS
    //NODELET_WARN_FILTER(10.0, "WARN output");
    //NODELET_WARN_STREAM_FILTER(10.0, "WARN" << " output");

    NODELET_ERROR("ERROR output");
    NODELET_ERROR_STREAM("ERROR" << " output");
    NODELET_ERROR_ONCE("ERROR output");
    NODELET_ERROR_STREAM_ONCE("ERROR" << " output");
    NODELET_ERROR_COND(1 == 1, "ERROR output");
    NODELET_ERROR_STREAM_COND(1 == 1, "ERROR" << " output");
    NODELET_ERROR_COND(1 == 0, "ERROR output unseen");
    NODELET_ERROR_STREAM_COND(1 == 0, "ERROR" << " output unseen");
    NODELET_ERROR_THROTTLE(10.0, "ERROR output");
    NODELET_ERROR_STREAM_THROTTLE(10.0, "ERROR" << " output");
    //TODO TEST FILTERS
    //NODELET_ERROR_FILTER(10.0, "ERROR output");
    //NODELET_ERROR_STREAM_FILTER(10.0, "ERROR" << " output");

    NODELET_FATAL("FATAL output");
    NODELET_FATAL_STREAM("FATAL" << " output");
    NODELET_FATAL_ONCE("FATAL output");
    NODELET_FATAL_STREAM_ONCE("FATAL" << " output");
    NODELET_FATAL_COND(1 == 1, "FATAL output");
    NODELET_FATAL_STREAM_COND(1 == 1, "FATAL" << " output");
    NODELET_FATAL_COND(1 == 0, "FATAL output unseen");
    NODELET_FATAL_STREAM_COND(1 == 0, "FATAL" << " output unseen");
    NODELET_FATAL_THROTTLE(10.0, "FATAL output");
    NODELET_FATAL_STREAM_THROTTLE(10.0, "FATAL" << " output");
    //TODO TEST FILTERS
    //NODELET_FATAL_FILTER(10.0, "FATAL output");
    //NODELET_FATAL_STREAM_FILTER(10.0, "FATAL" << " output");

    
  }

};

PLUGINLIB_DECLARE_CLASS(test_nodelet, ConsoleTest, test_nodelet::ConsoleTest, nodelet::Nodelet);
}
