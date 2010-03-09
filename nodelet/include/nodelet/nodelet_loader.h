/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

/**
@mainpage

\author Tully Foote 
**/

#ifndef NODELET_NODELET_LOADER_H_
#define NODELET_NODELET_LOADER_H_

#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include <pluginlib/class_loader.h>
#include <sstream>
#include <map>
#include "boost/shared_ptr.hpp"
#include "nodelet/NodeletLoad.h"
#include "nodelet/NodeletList.h"
#include "nodelet/NodeletUnload.h"


namespace nodelet
{

  /** \brief A class which will construct and sequentially call Nodelets according to xml
   * This is the primary way in which users are expected to interact with Nodelets
   */
  class NodeletLoader
  {
    private:
      pluginlib::ClassLoader<Nodelet> loader_;
    ros::ServiceServer load_server_, unload_server_, list_server_;

    public:
      /** \brief Create the filter chain object */
      NodeletLoader(): loader_("nodelet", "nodelet::Nodelet")
      {
        std::string lib_string = "";
        std::vector<std::string> libs = loader_.getDeclaredClasses();
        for (size_t i = 0 ; i < libs.size(); ++i)
        {
          lib_string = lib_string + std::string(", ") + libs[i];
        }    
        
        ros::NodeHandle server_nh_("~");
        load_server_ = server_nh_.advertiseService("load_nodelet", &NodeletLoader::serviceLoad, this); 
        unload_server_ = server_nh_.advertiseService("unload_nodelet", &NodeletLoader::serviceUnload, this); 
        list_server_ = server_nh_.advertiseService("list", &NodeletLoader::serviceList, this); 
        ROS_DEBUG("In FilterChain ClassLoader found the following libs: %s", lib_string.c_str());
      };

      ~NodeletLoader()
      {
        clear();

      };

      bool serviceLoad(nodelet::NodeletLoad::Request &req, 
                        nodelet::NodeletLoad::Response &res)
      {
        // build map
        ros::M_string remappings;        
        if (req.remap_source_args.size() != req.remap_target_args.size())
          ROS_ERROR("Bad remapppings provided, target and source of different length");
        else
        {
          //      std::cerr<< "remapping";
          for (size_t i = 0; i < req.remap_source_args.size(); ++i)
          {
            //std::cerr<< req.remap_source_args[i] << ":=" << req.remap_target_args[i] << std::endl;
            remappings[ros::names::resolve(req.remap_source_args[i])] = ros::names::resolve(req.remap_target_args[i]);
            ROS_DEBUG("%s:%s\n", ros::names::resolve(req.remap_source_args[i]).c_str(), remappings[ros::names::resolve(req.remap_source_args[i])].c_str());
          }
        }
        res.success = load(req.name, req.type, remappings, req.my_argv);
        return res.success;
      }

      bool serviceUnload(nodelet::NodeletUnload::Request &req, 
                        nodelet::NodeletUnload::Response &res)
      {
        res.success = unload(req.name);
        return res.success;
      }

      bool serviceList(nodelet::NodeletList::Request &req, 
                       nodelet::NodeletList::Response &res)
      {

        res.nodelets = listLoadedNodelets();
        return true;
      }
    bool load(const std::string &name, const std::string& type, const ros::M_string& remappings, const std::vector<std::string> & my_argv)
      {
        if (reference_pointers_.count(name) > 0)
          {
            ROS_ERROR("Cannot load nodelet %s for one exists with that name already", name.c_str());
            return false;
          }
        //\TODO store type in string format too, or provide accessors from pluginlib
        boost::shared_ptr<Nodelet > p(loader_.createClassInstance(type));
        if (p.get() == NULL)
          return false;
        reference_pointers_[name] = p;
        ROS_DEBUG("Done loading nodelet %s", name.c_str());

        p->init (name, remappings, my_argv);
        ROS_DEBUG("Done initing nodelet %s", name.c_str());
        return true;
      };
      
      bool unload(const std::string & name)
      {
        std::vector<std::string> output;
        std::map< std::string, boost::shared_ptr<Nodelet> >::iterator it = reference_pointers_.begin();
        for (; it != reference_pointers_.end(); ++it)
        {
          if (it->first == name)
          {
            reference_pointers_.erase(it);
            return true;
          }
        }
        ROS_ERROR("Failed to find nodelet with name '%s' to unload.", name.c_str());
        return false;

        /* \TODO Not sure why this isn't working, it never matches.  
        std::map<std::string , boost::shared_ptr<Nodelet > >::iterator it = reference_pointers_.find(name);
        if ( it == reference_pointers_.end() )
        {
        }
        else
        {
          ROS_ERROR("Failed to find nodelet with name '%s' to unload.", name.c_str());
          
          return false;
        }    
        */
      };

      /** \brief Clear all nodelets from this chain */
      bool clear() 
      {
        reference_pointers_.clear();
        return true;
      };
      
      /**\brief List the names of all loaded nodelets */
      std::vector<std::string> listLoadedNodelets()
      {
        std::vector<std::string> output;
        std::map< std::string, boost::shared_ptr<Nodelet> >::iterator it = reference_pointers_.begin();
        for (; it != reference_pointers_.end(); ++it)
        {
          output.push_back(it->first);
        }
        return output;
      };
    private:
      std::map <std::string, boost::shared_ptr<Nodelet> > reference_pointers_;   ///<! A map of pointers to currently constructed nodelets

  };


};

#endif //#ifndef NODELET_NODELET_LOADER_H_

