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

#include "nodelet/nodelet_loader.h"

using namespace nodelet;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "standalone_nodelet_manager");//, ros::init_options::AnonymousName);

  nodelet::NodeletLoader n;
  if (argc >= 3)
  {
    ros::NodeHandle nh;
    ros::M_string remappings;
    std::string nodelet_name = argv[1];
    std::string nodelet_type = argv[2];
    std::vector<std::string> my_argv;
    for (int i = 3; i < argc; i++)
      my_argv.push_back(argv[i]);
    n.load(nodelet_name, nodelet_type, remappings, my_argv);
    ROS_DEBUG("Successfully loaded nodelet of type '%s' into name '%s'\n", nodelet_name.c_str(), nodelet_name.c_str());
  }

  else if (argc != 1)
  {
    ROS_ERROR("Wrong number of arguments");
    exit(-1);
  }
  
  ros::spin ();

  return (0);
}
