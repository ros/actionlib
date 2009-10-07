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

#ifndef FILTERS_MEAN_H
#define FILTERS_MEAN_H

#include <stdint.h>
#include <cstring>
#include <stdio.h>

#include <boost/scoped_ptr.hpp>

#include "filters/filter_base.h"

namespace filters
{

/** \brief A increment filter which works on doubles.
 *
 */
template <typename T>
class IncrementFilter: public FilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  IncrementFilter();

  /** \brief Destructor to clean up
   */
  ~IncrementFilter();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update( const T & data_in, T& data_out);
  
};


template <typename T>
IncrementFilter<T>::IncrementFilter()
{
}

template <typename T>
bool IncrementFilter<T>::configure()
{
    
  return true;
}

template <typename T>
IncrementFilter<T>::~IncrementFilter()
{
}


template <typename T>
bool IncrementFilter<T>::update(const T & data_in, T& data_out)
{
  data_out = data_in + 1;  

  return true;
};

/** \brief A increment filter which works on arrays.
 *
 */
template <typename T>
class MultiChannelIncrementFilter: public MultiChannelFilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MultiChannelIncrementFilter();

  /** \brief Destructor to clean up
   */
  ~MultiChannelIncrementFilter();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update( const std::vector<T> & data_in, std::vector<T>& data_out);
  
protected:
  using MultiChannelFilterBase<T>::number_of_channels_;           ///< Number of elements per observation

  
  
};


template <typename T>
MultiChannelIncrementFilter<T>::MultiChannelIncrementFilter()
{
}

template <typename T>
bool MultiChannelIncrementFilter<T>::configure()
{
  
  return true;
}

template <typename T>
MultiChannelIncrementFilter<T>::~MultiChannelIncrementFilter()
{
}


template <typename T>
bool MultiChannelIncrementFilter<T>::update(const std::vector<T> & data_in, std::vector<T>& data_out)
{
  //  ROS_ASSERT(data_in.size() == width_);
  //ROS_ASSERT(data_out.size() == width_);
  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
  {
    ROS_ERROR("Configured with wrong size config:%d in:%d out:%d", number_of_channels_, (int)data_in.size(), (int)data_out.size());
    return false;
  }


  //Return each value
  for (uint32_t i = 0; i < number_of_channels_; i++)
  {
    data_out[i] = data_in[i] + 1;
  }

  return true;
};

}
#endif// FILTERS_INCREMENT_H
