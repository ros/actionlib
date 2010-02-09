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

///\author Tully Foote tfoote@willowgarage.com

#ifndef FILTERS_REALTIME_CIRCULAR_BUFFER_H_
#define FILTERS_REALTIME_CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <vector>

#include <algorithm>
#include <boost/circular_buffer.hpp>

namespace filters
{

/** \brief A realtime safe circular (ring) buffer.
 */
template <typename T>
class RealtimeCircularBuffer
{
private:
  RealtimeCircularBuffer();

public:
  RealtimeCircularBuffer(int size, const T& default_val):
    counter_(0), cb_(size)
  {
    for (unsigned int i = 0; i < cb_.capacity(); i++)
    {
      cb_.push_back(default_val);
    }
  };

  void push_back(const T& item)
  {
    if (cb_.capacity() == 0) return;

    if ( counter_ < cb_.size()) 
    {
      cb_[counter_] = item; 
    }
    else 
      cb_.push_back(item);
    counter_ ++;
  };
  
  void push_front(const T& item)
  {
    if (cb_.capacity() == 0) return;
    cb_.push_front(item);
    counter_ ++;
  };
  
  void clear() { counter_ = 0;};

  void set_capacity(unsigned int order, const T& value);
  
  T& front(){return cb_.front();};
  T& back()
  {
    if (counter_ < cb_.size()) 
      return cb_[counter_]; 
    else 
      return cb_.back();
  };
  
  unsigned int size(){  return std::min(counter_, (unsigned int)cb_.size());};
  bool empty(){return cb_.empty();};
  T& at(size_t index){return cb_.at(index);};
  T& operator[](size_t index){return cb_[index];}
private:

  unsigned int counter_; //<! special counter to keep track of first N times through

  boost::circular_buffer<T> cb_;
};
} //namespace filters
#endif //#ifndef REALTIME_CIRCULAR_BUFFER_H_
