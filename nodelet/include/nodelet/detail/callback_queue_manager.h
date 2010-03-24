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

#ifndef NODELET_CALLBACK_QUEUE_MANAGER_H
#define NODELET_CALLBACK_QUEUE_MANAGER_H

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <vector>
#include <deque>

namespace nodelet
{
namespace detail
{
class CallbackQueue;
typedef boost::shared_ptr<CallbackQueue> CallbackQueuePtr;

class CallbackQueueManager
{
public:
  CallbackQueueManager();
  ~CallbackQueueManager();

  void addQueue(const CallbackQueuePtr& queue, bool threaded);
  void removeQueue(const CallbackQueuePtr& queue);
  void callbackAdded(const CallbackQueuePtr& queue);

  uint32_t getNumWorkerThreads();

private:
  void managerThread();
  void workerThread();

  struct QueueInfo
  {
    QueueInfo()
    : threaded(false)
    {}

    CallbackQueuePtr queue;
    bool threaded;
  };

  typedef boost::unordered_map<CallbackQueue*, QueueInfo> M_Queue;
  M_Queue queues_;
  boost::mutex queues_mutex_;

  typedef std::vector<CallbackQueuePtr> V_Queue;
  V_Queue waiting_;
  boost::mutex waiting_mutex_;
  boost::condition_variable waiting_cond_;
  boost::thread_group tg_;

  // TODO: mrsw lockfree queue.  shared_queue_mutex_ has the potential for a lot of contention
  typedef std::deque<CallbackQueuePtr> D_Queue;
  D_Queue shared_queue_;
  boost::mutex shared_queue_mutex_;
  boost::condition_variable shared_queue_cond_;

  bool running_;
};

} // namespace detail
} // namespace nodelet

#endif // NODELET_CALLBACK_QUEUE_MANAGER_H
