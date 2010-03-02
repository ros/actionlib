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

#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include "boost/make_shared.hpp"

namespace nodelet_tutorial_math
{
class Plus : public nodelet::Nodelet
{
public:
  Plus(void):value_(0){printf("Plus created\n");};
  void init(ros::NodeHandle &node_handle, ros::NodeHandle &private_node_handle)
  {
    node_handle_ = private_node_handle;
    node_handle_.getParam("value", value_);
    pub = node_handle_.advertise<std_msgs::Float64>("out", 10);
    sub = node_handle_.subscribe("in", 10, &Plus::callback, this);

  };
  



private:
  void callback(const std_msgs::Float64::ConstPtr& input)
  {
    std_msgs::Float64 output;
    output.data= input->data + value_;
    
    std_msgs::Float64::ConstPtr out_const = boost::make_shared<const std_msgs::Float64>(output);
    pub.publish(out_const);
  };

  ros::NodeHandle node_handle_;
  ros::Publisher pub;
  ros::Subscriber sub;
  double value_;
};





PLUGINLIB_DECLARE_CLASS(nodelet_tutorial_math, Plus, nodelet_tutorial_math::Plus, nodelet::Nodelet);
}
