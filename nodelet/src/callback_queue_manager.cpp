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

#include <nodelet/detail/callback_queue_manager.h>
#include <nodelet/detail/callback_queue.h>
#include <ros/callback_queue.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <boost/bind.hpp>

#include <ros/assert.h>

namespace nodelet
{
namespace detail
{

CallbackQueueManager::CallbackQueueManager()
: running_(true)
{
  tg_.create_thread(boost::bind(&CallbackQueueManager::managerThread, this));

  size_t num_threads = boost::thread::hardware_concurrency();
  for (size_t i = 0; i < num_threads; ++i)
  {
    tg_.create_thread(boost::bind(&CallbackQueueManager::workerThread, this));
  }
}

CallbackQueueManager::~CallbackQueueManager()
{
  running_ = false;
  waiting_cond_.notify_all();
  shared_queue_cond_.notify_all();
  tg_.join_all();
}

uint32_t CallbackQueueManager::getNumWorkerThreads()
{
  return boost::thread::hardware_concurrency();
}

void CallbackQueueManager::addQueue(const CallbackQueuePtr& queue, bool threaded)
{
  boost::mutex::scoped_lock lock(queues_mutex_);
  ROS_ASSERT(queues_.find(queue.get()) == queues_.end());

  QueueInfo& info = queues_[queue.get()];
  info.queue = queue;
  info.threaded = threaded;
}

void CallbackQueueManager::removeQueue(const CallbackQueuePtr& queue)
{
  boost::mutex::scoped_lock lock(queues_mutex_);
  ROS_ASSERT(queues_.find(queue.get()) != queues_.end());

  queues_.erase(queue.get());
}

void CallbackQueueManager::callbackAdded(const CallbackQueuePtr& queue)
{
  {
    boost::mutex::scoped_lock lock(waiting_mutex_);
    waiting_.push_back(queue);
  }

  waiting_cond_.notify_all();
}

void CallbackQueueManager::managerThread()
{
  V_Queue local_waiting;
  V_Queue single_threaded;

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(waiting_mutex_);

      while (waiting_.empty() && running_)
      {
        waiting_cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }

      local_waiting.swap(waiting_);
    }

    {
      boost::mutex::scoped_lock lock(queues_mutex_);

      V_Queue::iterator it = local_waiting.begin();
      V_Queue::iterator end = local_waiting.end();
      for (; it != end; ++it)
      {
        CallbackQueuePtr& queue = *it;

        M_Queue::iterator it = queues_.find(queue.get());
        if (it != queues_.end())
        {
          QueueInfo& info = it->second;
          if (info.threaded)
          {
            boost::mutex::scoped_lock lock(shared_queue_mutex_);
            shared_queue_.push_back(queue);
            shared_queue_cond_.notify_one();
          }
          else
          {
            single_threaded.push_back(queue);
          }
        }
      }
    }

    V_Queue::iterator it = single_threaded.begin();
    V_Queue::iterator end = single_threaded.end();
    for (; it != end; ++it)
    {
      (*it)->callOne();

      if ((*it)->callOne() == ros::CallbackQueue::TryAgain)
      {
        boost::mutex::scoped_lock lock(waiting_mutex_);
        waiting_.push_back(*it);
      }
    }

    local_waiting.clear();
    single_threaded.clear();
  }
}

void CallbackQueueManager::workerThread()
{
  V_Queue local_queues;

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(shared_queue_mutex_);

      while (shared_queue_.empty() && running_)
      {
        shared_queue_cond_.wait(lock);
      }

      if (!running_)
      {
        return;
      }

      local_queues.push_back(shared_queue_.front());
      shared_queue_.pop_front();
    }

    V_Queue::iterator it = local_queues.begin();
    V_Queue::iterator end = local_queues.end();
    for (; it != end; ++it)
    {
      CallbackQueuePtr& queue = *it;
      if (queue->callOne() == ros::CallbackQueue::TryAgain)
      {
        boost::mutex::scoped_lock lock(shared_queue_mutex_);
        shared_queue_.push_back(queue);
        shared_queue_cond_.notify_one();
      }
    }

    local_queues.clear();
  }
}

} // namespace detail
} // namespace nodelet
