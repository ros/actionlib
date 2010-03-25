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

  size_t num_threads = getNumWorkerThreads();
  thread_info_.resize(num_threads);
  for (size_t i = 0; i < num_threads; ++i)
  {
    tg_.create_thread(boost::bind(&CallbackQueueManager::workerThread, this, &thread_info_[i]));
  }
}

CallbackQueueManager::~CallbackQueueManager()
{
  running_ = false;
  waiting_cond_.notify_all();

  size_t num_threads = getNumWorkerThreads();
  for (size_t i = 0; i < num_threads; ++i)
  {
    thread_info_[i].queue_cond->notify_all();
  }

  tg_.join_all();
}

uint32_t CallbackQueueManager::getNumWorkerThreads()
{
  return boost::thread::hardware_concurrency();
}

void CallbackQueueManager::addQueue(const CallbackQueuePtr& queue, bool threaded)
{
  boost::mutex::scoped_lock lock(queues_mutex_);

  QueueInfoPtr& info = queues_[queue.get()];
  ROS_ASSERT(!info);
  info.reset(new QueueInfo);
  info->queue = queue;
  info->threaded = threaded;
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

CallbackQueueManager::ThreadInfo* CallbackQueueManager::getSmallestQueue()
{
  size_t smallest = std::numeric_limits<size_t>::max();
  uint32_t smallest_index = 0xffffffff;
  V_ThreadInfo::iterator it = thread_info_.begin();
  V_ThreadInfo::iterator end = thread_info_.end();
  for (; it != end; ++it)
  {
    ThreadInfo& ti = *it;
    boost::mutex::scoped_lock lock(*ti.queue_mutex);

    size_t size = ti.queue.size() + ti.calling;
    if (size == 0)
    {
      return &ti;
    }

    if (size < smallest)
    {
      smallest = size;
      smallest_index = it - thread_info_.begin();
    }
  }

  return &thread_info_[smallest_index];
}

void CallbackQueueManager::managerThread()
{
  V_Queue local_waiting;

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
          QueueInfoPtr& info = it->second;
          ThreadInfo* ti = 0;
          if (info->threaded)
          {
            ti = getSmallestQueue();
            boost::mutex::scoped_lock lock(*ti->queue_mutex);
            ti->queue.push_back(std::make_pair(queue, info));
          }
          else
          {
            boost::mutex::scoped_lock lock(info->st_mutex);

            ++info->in_thread;

            if (info->in_thread > 1)
            {
              ti = &thread_info_[info->thread_index];
              boost::mutex::scoped_lock lock(*ti->queue_mutex);
              ti->queue.push_back(std::make_pair(queue, info));
            }
            else
            {
              ti = getSmallestQueue();
              info->thread_index = ti - &thread_info_.front();
              boost::mutex::scoped_lock lock(*ti->queue_mutex);
              ti->queue.push_back(std::make_pair(queue, info));
            }
          }

          ti->queue_cond->notify_all();
        }
      }
    }

    local_waiting.clear();
  }
}

void CallbackQueueManager::workerThread(ThreadInfo* info)
{
  std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> > local_queues;

  while (running_)
  {
    {
      boost::mutex::scoped_lock lock(*info->queue_mutex);

      while (info->queue.empty() && running_)
      {
        info->queue_cond->wait(lock);
      }

      if (!running_)
      {
        return;
      }

      info->calling += info->queue.size();
      info->queue.swap(local_queues);
    }

    std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> >::iterator it = local_queues.begin();
    std::vector<std::pair<CallbackQueuePtr, QueueInfoPtr> >::iterator end = local_queues.end();
    for (; it != end; ++it)
    {
      CallbackQueuePtr& queue = it->first;
      QueueInfoPtr& qi = it->second;
      if (queue->callOne() == ros::CallbackQueue::TryAgain)
      {
        callbackAdded(queue);
      }

      if (!qi->threaded)
      {
        boost::mutex::scoped_lock lock(qi->st_mutex);
        --qi->in_thread;
      }
    }

    local_queues.clear();

    info->calling = 0;
  }
}

} // namespace detail
} // namespace nodelet
