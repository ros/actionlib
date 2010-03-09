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

#ifndef nodelet_nodelet_h
#define nodelet_nodelet_h

#include <string>
#include "ros/ros.h"
#include "ros/callback_queue.h"

#define NODELET_DEBUG(...) ROS_DEBUG_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM(...) ROS_DEBUG_STREAM_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_DEBUG_COND(cond, ...) ROS_DEBUG_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)
#define NODELET_DEBUG_STREAM_COND(cond, ...) ROS_DEBUG_STREAM_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)

#define NODELET_INFO(...) ROS_INFO_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_INFO_STREAM(...) ROS_INFO_STREAM_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_INFO_COND(cond, ...) ROS_INFO_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)
#define NODELET_INFO_STREAM_COND(cond, ...) ROS_INFO_STREAM_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)

#define NODELET_WARN(...) ROS_WARN_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_WARN_STREAM(...) ROS_WARN_STREAM_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_WARN_COND(cond, ...) ROS_WARN_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)
#define NODELET_WARN_STREAM_COND(cond, ...) ROS_WARN_STREAM_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)

#define NODELET_ERROR(...) ROS_ERROR_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_ERROR_STREAM(...) ROS_ERROR_STREAM_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_ERROR_COND(cond, ...) ROS_ERROR_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)
#define NODELET_ERROR_STREAM_COND(cond, ...) ROS_ERROR_STREAM_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)

#define NODELET_FATAL(...) ROS_FATAL_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_FATAL_STREAM(...) ROS_FATAL_STREAM_NAMED(getName().c_str(), __VA_ARGS__)
#define NODELET_FATAL_COND(cond, ...) ROS_FATAL_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)
#define NODELET_FATAL_STREAM_COND(cond, ...) ROS_FATAL_STREAM_COND_NAMED(cond, getName().c_str(), __VA_ARGS__)

namespace nodelet
{
  class Nodelet
  {
    // Protected data fields for use by the subclass.
    protected:
    const std::string& getName() const { return (nodelet_name_); }
    inline ros::NodeHandle& getNodeHandle () { return (nh_); }
    inline ros::NodeHandle& getPrivateNodeHandle () { return (private_nh_); }
    inline ros::NodeHandle getMTNodeHandle ()  { ros::NodeHandle nh = nh_; nh.setCallbackQueue(&multithreaded_callback_queue_); return (nh); }
    inline ros::NodeHandle getMTPrivateNodeHandle ()  { ros::NodeHandle nh = private_nh_; nh.setCallbackQueue(&multithreaded_callback_queue_); return (nh); }
    inline ros::CallbackQueue& getMTCallbackQueue () { return (multithreaded_callback_queue_); }
    inline std::vector<std::string> getMyArgv() const { return my_argv_;};


    // Internal storage;
    private:
      std::string nodelet_name_;
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;
    std::vector<std::string> my_argv_;

      ros::AsyncSpinner* mt_spinner_; //\TODO this should be removed
      ros::CallbackQueue multithreaded_callback_queue_;

      // Method to be overridden by subclass when starting up. 
      virtual void onInit () = 0;
      
    // Public API used for launching
    public:
    /**\brief Empty constructor required for dynamic loading */
    Nodelet (): nodelet_name_("uninitialized"), mt_spinner_ (NULL) {};
    
    /**\brief Init function called at startup
     * \param name The name of the nodelet
     * \param remapping_args The remapping args in a map for the nodelet
     * \param my_args The commandline arguments for this nodelet stripped of special arguments such as ROS arguments 
     */
    void init (const std::string& name, const ros::M_string& remapping_args, const std::vector<std::string>& my_argv)
      {
        mt_spinner_ = new ros::AsyncSpinner(0, &multithreaded_callback_queue_);
        mt_spinner_->start ();

        nodelet_name_ = name;
        nh_ = ros::NodeHandle ("", remapping_args);
        my_argv_ = my_argv;
        private_nh_ = ros::NodeHandle (name, remapping_args);
        NODELET_DEBUG ("Nodelet initializing");
        this->onInit ();
      };

    virtual ~Nodelet () { NODELET_DEBUG ("nodelet destructor.");  delete mt_spinner_;}
  };

}
#endif //nodelet_nodelet_h
