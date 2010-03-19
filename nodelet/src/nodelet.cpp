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


class NodeletArgumentParsing
{
private:
  std::string command_;
  std::string type_;
  std::string default_name_;  
  std::string manager_;
  std::vector<std::string> local_args_;    

public:
  //NodeletArgumentParsing() { };
  bool parseArgs(int argc, char** argv)
  {
    std::vector<std::string> non_ros_args;
    ros::removeROSArgs(argc, argv, non_ros_args);
    size_t used_args = 0;

    if (non_ros_args.size() > 1)
      command_ = non_ros_args[1];
    else
      return false;

    
    if (command_ == "load" && non_ros_args.size() > 3)
    {
      type_ = non_ros_args[2];
      manager_ = non_ros_args[3];
      used_args = 4;
    }
    
    
    if (non_ros_args.size() > 2)
    {
      if (command_ == "unload")
      {
        manager_ = non_ros_args[2];
        used_args = 3;
      }
      else if (command_ == "standalone")
      {
        type_ = non_ros_args[2];
        printf("type is %s\n", type_.c_str());
        used_args = 3;
      }
    }
    
    if (command_ == "manager")
      used_args = 2;
      
    for (size_t i = used_args; i < non_ros_args.size(); i++)
      local_args_.push_back(non_ros_args[i]);

    
    if (used_args > 0) return true;
    else return false;
  
  };


  std::string getCommand() const {return command_;};
  std::string getType() const {return type_;};
  std::string getManager() const {return manager_;};
  std::vector<std::string> getMyArgv () const {return local_args_;};
  std::string getDefaultName()
  {
    std::string s = type_; 
    replace(s.begin(), s.end(), '/', '_'); 
    return s;
  };
  
};
  
  
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
      return false;
    }
    return true;
  }
private:
  ros::NodeHandle n_;
};

void print_usage(int argc, char** argv)
{
  printf("Your usage: \n");
  for (int i = 0; i < argc; i++)
    printf("%s ", argv[i]);
  printf("\nnodelet usage:\n");
  printf("nodelet load pkg/Type manager - Launch a nodelet of type pkg/Type on manager manager\n");
  printf("nodelet standalone pkg/Type   - Launch a nodelet of type pkg/Type in a standalone node\n");
  //  printf("nodelet unload name manager   - Unload a nodelet a nodelet by name from manager\n");
  printf("nodelet manager               - Launch a nodelet manager node\n");

};

int main (int argc, char** argv)
{
  NodeletArgumentParsing arg_parser;  

  if (!arg_parser.parseArgs(argc, argv))
  {
    print_usage(argc, argv);
    return (-1);
  }
  std::string command = arg_parser.getCommand();
    
    
  if (command == "manager")
  {
    ros::init (argc, argv, "manager");
    nodelet::NodeletLoader n;
    ros::spin();
  }
  else if (command == "standalone")
  {
    ros::init (argc, argv, arg_parser.getDefaultName());
    
    nodelet::NodeletLoader n(false);
    ros::NodeHandle nh;
    ros::M_string remappings; //Remappings are already applied by ROS no need to generate them.
    std::string nodelet_name = ros::this_node::getName ();
    std::string nodelet_type = arg_parser.getType();
    n.load(nodelet_name, nodelet_type, remappings, arg_parser.getMyArgv());
    ROS_DEBUG("Successfully loaded nodelet of type '%s' into name '%s'\n", nodelet_name.c_str(), nodelet_name.c_str());
    
    ros::spin();
  }

  else if (command == "load")
  {  
    
    ros::init (argc, argv, arg_parser.getDefaultName());
    NodeletInterface ni;
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName ();
    std::string type = arg_parser.getType();
    std::string manager = arg_parser.getManager();
    ni.loadNodelet (name, type , manager, arg_parser.getMyArgv());
    //\TODO register Ctrl-C handler for unload before shutdown
    ros::spin ();
  }
  else if (command == "unload")
  {
    ros::init (argc, argv, arg_parser.getDefaultName());
    NodeletInterface ni;
    ni.unloadNodelet (argv[2], argv[3]);
    
  }
  else
  {
    ros::init(argc, argv, "nodelet");
    ROS_ERROR("Command %s unknown", command.c_str());
    return -1;
  }

  return (0);
}
