/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/**
@mainpage

\author Radu Bogdan Rusu

@b nodeletcpp is a tool for loading/unloading nodelets to/from a Nodelet manager.
**/
#include <ros/ros.h>
#include "nodelet/nodelet_loader.h"
#include "nodelet/NodeletList.h"
#include "nodelet/NodeletLoad.h"
#include "nodelet/NodeletUnload.h"

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. */
template <typename Type> int
  parseArguments (int argc, char** argv, const char* str, Type &value)
{
  for (int i = 1; i < argc; ++i)
  {
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      value = boost::lexical_cast<Type>(argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

std::vector<std::string> getMyArgv (int argc, char** argv, int num_built_in_args)
{
  std::vector<std::string> my_argv;
  for (int i = num_built_in_args; i < argc; i++)
    my_argv.push_back(argv[i]);
  return my_argv;
}



class NodeletInterface
{
  public:
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Unload the nodelet */
    bool
      unloadNodelet (const char *name, const char *manager)
    {
      ROS_INFO_STREAM ("Unloading nodelet " << name << " from manager " << manager);
      
      std::string service_name = std::string (manager) + "/unload_nodelet";
      // Wait until the service is advertised
      ros::ServiceClient client = n_.serviceClient<nodelet::NodeletUnload> (service_name);
      client.waitForExistence ();

      // Call the service
      nodelet::NodeletLoad srv;
      srv.request.name = std::string (name);
      if (!client.call (srv))
      {
        ROS_ERROR ("Failed to call service!");
        return (false);
      }
      return (true);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Load the nodelet */
    bool
    loadNodelet (const std::string name, const std::string& type, const std::string& manager, const std::vector<std::string> & args)
    {
      ros::M_string remappings = ros::names::getRemappings ();
      std::vector<std::string> sources (remappings.size ()), targets (remappings.size ());
      ROS_INFO_STREAM ("Loading nodelet " << name << " of type " << type << " to manager " << manager << " with the following remappings:");
      int i = 0;
      for (ros::M_string::iterator it = remappings.begin (); it != remappings.end (); ++it, ++i)
      {
        sources[i] = (*it).first; 
        targets[i] = (*it).second;
        ROS_INFO_STREAM (sources[i] << " -> " << targets[i]);
      }

      // Get and set the parameters
      XmlRpc::XmlRpcValue param;
      std::string node_name = ros::this_node::getName ();
      n_.getParam (node_name, param);
      n_.setParam (name, param);

      std::string service_name = std::string (manager) + "/load_nodelet";

      // Wait until the service is advertised
      ros::ServiceClient client = n_.serviceClient<nodelet::NodeletLoad> (service_name);
      client.waitForExistence ();

      // Call the service
      nodelet::NodeletLoad srv;
      srv.request.name = std::string (name);
      srv.request.type = std::string (type);
      srv.request.remap_source_args = sources;
      srv.request.remap_target_args = targets;
      srv.request.my_argv = args;
      if (!client.call (srv))
      {
        ROS_ERROR ("Failed to call service!");
        return (false);
      }
      return (true);
    }
  private:
    ros::NodeHandle n_;
};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "nodelet");
  if (argc < 2)
  {
    ROS_ERROR ("Nodelet Needs a command [manager, standalone, load, unload]");
    return (-1);
  }
  std::string command = argv[1];
  

  if (command == "manager")
  {
    nodelet::NodeletLoader n;
    ros::spin();
  }
  else if (command == "standalone")
  {
    nodelet::NodeletLoader n(false);
    if (argc >= 3)
    {
      ros::NodeHandle nh;
      ros::M_string remappings; //Remappings are already applied by ROS no need to generate them.
      std::string nodelet_name = ros::this_node::getName ();
      std::string nodelet_type = argv[2];
      n.load(nodelet_name, nodelet_type, remappings, getMyArgv(argc, argv, 3));
      ROS_DEBUG("Successfully loaded nodelet of type '%s' into name '%s'\n", nodelet_name.c_str(), nodelet_name.c_str());
    }
    ros::spin();
  }

  else if (command == "load")
  {  
    if (argc < 4)
    {
      ROS_ERROR ("Nodelet load needs at least 4 args");
      return (-1);
    }
    
    NodeletInterface ni;
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName ();
    std::string type = argv[2];
    std::string manager = argv[3];
    ni.loadNodelet (name, type , manager, getMyArgv(argc, argv, 4));
    //\TODO register Ctrl-C handler for unload before shutdown
    ros::spin ();
  }
  else if (command == "unload")
  {
    NodeletInterface ni;
    ni.unloadNodelet (argv[2], argv[3]);
    
  }
  else
  {
    ROS_ERROR("Command %s unknown", command.c_str());
    return -1;
  }

  return (0);
}
