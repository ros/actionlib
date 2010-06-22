/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef FILTERS_FILTER_BASE_H_
#define FILTERS_FILTER_BASE_H_

#include <typeinfo>
#include "ros/assert.h"
#include "ros/console.h"
#include "ros/ros.h"

#include "boost/scoped_ptr.hpp"
#include <boost/algorithm/string.hpp>

namespace filters
{

typedef std::map<std::string, XmlRpc::XmlRpcValue> string_map_t;

/** \brief A Base filter class to provide a standard interface for all filters
 *
 */
template<typename T>
class FilterBase
{
public:
  /** \brief Default constructor used by Filter Factories
   */
  FilterBase():configured_(false){};

  /** \brief Virtual Destructor
   */
  virtual ~FilterBase(){};

  /** \brief Configure the filter from the parameter server 
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a different namespace.
   */
  bool configure(const std::string& param_name, ros::NodeHandle node_handle = ros::NodeHandle())
  {
    XmlRpc::XmlRpcValue config;
    if (!node_handle.getParam(param_name, config))
    {
      ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
      return false;
    }
    return configure(config);
    
  }

  /** \brief The public method to configure a filter from XML 
   * \param config The XmlRpcValue from which the filter should be initialized
   */
  bool configure(XmlRpc::XmlRpcValue& config)
  {
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    bool retval = true;

    retval = retval && loadConfiguration(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  }

  /** \brief Update the filter and return the data seperately
   * This is an inefficient way to do this and can be overridden in the derived class
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   */
  virtual bool update(const T& data_in, T& data_out)=0;

  /** \brief Get the type of the filter as a string */
  std::string getType() {return filter_type_;};

  /** \brief Get the name of the filter as a string */
  inline const std::string& getName(){return filter_name_;};


protected:

  /** \brief Pure virtual function for the sub class to configure the filter
   * This function must be implemented in the derived class.
   */
  virtual bool configure()=0;


  /** \brief Get a filter parameter as a string 
   * \param name The name of the parameter
   * \param value The string to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string& name, std::string& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      return false;
    }

    value = std::string(it->second);
    return true;
  }

  /** \brief Get a filter parameter as a double
   * \param name The name of the parameter
   * \param value The double to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, double& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      return false;
    }

    value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(it->second) : (double)(it->second);
    return true;
  }

  /** \brief Get a filter parameter as a int
   * \param name The name of the parameter
   * \param value The int to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, int& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      return false;
    }

    value = it->second;
    return true;
  }

  /** \brief Get a filter parameter as an unsigned int
   * \param name The name of the parameter
   * \param value The int to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, unsigned  int& value)
  {
    int signed_value;
    if (!getParam(name, signed_value))
      return false;
    if (signed_value < 0)
      return false;
    value = signed_value;
    return true;
  };

  /** \brief Get a filter parameter as a std::vector<double>
   * \param name The name of the parameter
   * \param value The std::vector<double> to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, std::vector<double>& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      return false;
    }

    XmlRpc::XmlRpcValue double_array = it->second;

    for (int i = 0; i < double_array.size(); ++i){
      if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        return false;
      }

      double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
      value.push_back(double_value);
    }
    
    return true;
  }

  /** \brief Get a filter parameter as a std::vector<string>
   * \param name The name of the parameter
   * \param value The std::vector<sgring> to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string&name, std::vector<std::string>& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      return false;
    }

    XmlRpc::XmlRpcValue string_array = it->second;
    
    for (unsigned int i = 0; i < string_array.size(); ++i){
      if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        return false;
      }

      value.push_back(string_array[i]);
    }

    return true;
  }

  /** \brief Get a filter parameter as a XmlRpcValue
   * \param name The name of the parameter
   * \param value The XmlRpcValue to set with the value
   * \return Whether or not the parameter of name/type was set */
  bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      return false;
    }

    value = it->second;
    return true;
  }
  
  ///The name of the filter
  std::string filter_name_;
  ///The type of the filter (Used by FilterChain for Factory construction)
  std::string filter_type_;
  /// Whether the filter has been configured.  
  bool configured_;

  ///Storage of the parsed xml parameters
  string_map_t params_;

private:
  /**\brief Set the name and type of the filter from the parameter server
   * \param param_name The parameter from which to read
   */
  bool setNameAndType(XmlRpc::XmlRpcValue& config)
  {
    if(!config.hasMember("name"))
    {
      ROS_ERROR("Filter didn't have name defined, other strings are not allowed");
      return false;
    }

    std::string name = config["name"];

    if(!config.hasMember("type"))
    {
      ROS_ERROR("Filter %s didn't have type defined, other strings are not allowed", name.c_str());
      return false;
    }

    std::string type = config["type"];

    filter_name_ = name;
    filter_type_ = type;
    ROS_DEBUG("Configuring Filter of Type: %s with name %s", type.c_str(), name.c_str());
    return true;
  }

protected:
  bool loadConfiguration(XmlRpc::XmlRpcValue& config)
  {
    if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("A filter configuration must be a map with fields name, type, and params");
      return false;
    } 

    if (!setNameAndType(config))
    {
      return false;
    }

    //check to see if we have parameters in our list
    if(config.hasMember("params"))
    {
      //get the params map
      XmlRpc::XmlRpcValue params = config["params"];

      if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("params must be a map");
        return false;
      }
      else{
        //Load params into map
        for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
        {
          ROS_DEBUG("Loading param %s\n", it->first.c_str());
          params_[it->first] = it->second;
        } 
      }
    }

    return true;    
  }
};


template <typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase():number_of_channels_(0){};
  
  /** \brief Configure the filter from the parameter server 
   * \param number_of_channels How many parallel channels the filter will process
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a different namespace.
   */
  bool configure(unsigned int number_of_channels, const std::string& param_name, ros::NodeHandle node_handle = ros::NodeHandle())
  {
    XmlRpc::XmlRpcValue config;
    if (!node_handle.getParam(param_name, config))
    {
      ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
      return false;
    }
    return configure(number_of_channels, config);
    
  }


  /** \brief The public method to configure a filter from XML 
   * \param number_of_channels How many parallel channels the filter will process
   * \param config The XmlRpcValue to load the configuration from 
   */
  bool configure(unsigned int number_of_channels, XmlRpc::XmlRpcValue& config)
  {
    ROS_DEBUG("FilterBase being configured with XmlRpc xml: %s type: %d", config.toXml().c_str(), config.getType());
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    number_of_channels_ = number_of_channels;
    ROS_DEBUG("MultiChannelFilterBase configured with %d channels", number_of_channels_);
    bool retval = true;

    retval = retval && FilterBase<T>::loadConfiguration(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  };


  /** \brief A method to hide the base class method and warn if improperly called */
  bool configure(XmlRpc::XmlRpcValue& config)
  {
    ROS_ERROR("MultiChannelFilterBase configure should be called with a number of channels argument, assuming 1");
    return configure(1, config);
  }

  virtual bool configure()=0;
  

  /** \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)=0;

  virtual bool update(const T& data_in, T& data_out)
  {
    ROS_ERROR("THIS IS A MULTI FILTER DON'T CALL SINGLE FORM OF UPDATE");
    return false;
  };


protected:
  using FilterBase<T>::configured_;
  using FilterBase<T>::filter_type_;
  using FilterBase<T>::filter_name_;

  /// How many parallel inputs for which the filter is to be configured
  unsigned int number_of_channels_;
  

};

}
#endif //#ifndef FILTERS_FILTER_BASE_H_
