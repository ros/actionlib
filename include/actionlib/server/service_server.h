/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef ACTIONLIB__SERVER__SERVICE_SERVER_H_
#define ACTIONLIB__SERVER__SERVICE_SERVER_H_

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server.h>

#include <string>

namespace actionlib
{

class ServiceServerImp
{
public:
  ServiceServerImp() {}
  virtual ~ServiceServerImp() {}
};

class ServiceServer
{
public:
  ServiceServer(boost::shared_ptr<ServiceServerImp> server)
  : server_(server) {}

private:
  boost::shared_ptr<ServiceServerImp> server_;
};

template<class ActionSpec>
ServiceServer advertiseService(ros::NodeHandle n, std::string name,
  boost::function<bool(const typename ActionSpec::_action_goal_type::_goal_type &,
  typename ActionSpec::_action_result_type::_result_type & result)> service_cb);

template<class ActionSpec>
class ServiceServerImpT : public ServiceServerImp
{
public:
  // generates typedefs that we'll use to make our lives easier
  ACTION_DEFINITION(ActionSpec);

  typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;

  ServiceServerImpT(ros::NodeHandle n, std::string name,
    boost::function<bool(const Goal &, Result & result)> service_cb);
  void goalCB(GoalHandle g);

private:
  boost::shared_ptr<ActionServer<ActionSpec> > as_;
  boost::function<bool(const Goal &, Result & result)> service_cb_;
};

}  // namespace actionlib

// include the implementation
#include <actionlib/server/service_server_imp.h>
#endif  // ACTIONLIB__SERVER__SERVICE_SERVER_H_
