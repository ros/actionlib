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
#include <ros/time.h>
#include <ros/console.h>

#include <boost/thread.hpp>

#include <gtest/gtest.h>

using namespace nodelet;
using namespace nodelet::detail;

class SingleThreadedCallback : public ros::CallbackInterface
{
public:
  SingleThreadedCallback(boost::barrier* bar)
  : success(true)
  , calls(0)
  , inited_(false)
  , barrier_(bar)
  {}

  ros::CallbackInterface::CallResult call()
  {
    ros::WallDuration(0.1).sleep();
    barrier_->wait();

    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!inited_)
      {
        initial_call_id = boost::this_thread::get_id();
        inited_ = true;
      }

      if (initial_call_id != boost::this_thread::get_id())
      {
        success = false;
      }

      ++calls;
    }

    return Success;
  }

  bool success;
  uint32_t calls;
  boost::thread::id initial_call_id;

private:
  bool inited_;
  boost::mutex mutex_;
  boost::barrier* barrier_;
};
typedef boost::shared_ptr<SingleThreadedCallback> SingleThreadedCallbackPtr;

TEST(CallbackQueueManager, singleThreaded)
{
  CallbackQueueManager man;
  CallbackQueuePtr queue(new CallbackQueue(&man));
  man.addQueue(queue, false);

  boost::barrier bar(1);
  SingleThreadedCallbackPtr cb(new SingleThreadedCallback(&bar));
  for (uint32_t i = 0; i < 10; ++i)
  {
    queue->addCallback(cb, 0);
  }

  while (cb->calls < 10)
  {
    ros::WallDuration(0.01).sleep();
  }

  EXPECT_EQ(cb->calls, 10U);
  ASSERT_TRUE(cb->success);
}

TEST(CallbackQueueManager, multipleSingleThreaded)
{
  uint32_t thread_count = boost::thread::hardware_concurrency();
  // Need at least 2 threads here
  if (thread_count == 1)
  {
    thread_count = 2;
  }
  CallbackQueueManager man(thread_count);
  CallbackQueuePtr queue1(new CallbackQueue(&man));
  CallbackQueuePtr queue2(new CallbackQueue(&man));
  man.addQueue(queue1, false);
  man.addQueue(queue2, false);

  boost::barrier bar(2);
  SingleThreadedCallbackPtr cb1(new SingleThreadedCallback(&bar));
  SingleThreadedCallbackPtr cb2(new SingleThreadedCallback(&bar));
  for (uint32_t i = 0; i < 10; ++i)
  {
    queue1->addCallback(cb1, 1);
    queue2->addCallback(cb2, 1);
  }

  while (cb1->calls < 10 || cb2->calls < 10)
  {
    ros::WallDuration(0.01).sleep();
  }

  EXPECT_EQ(cb1->calls, 10U);
  EXPECT_EQ(cb2->calls, 10U);
  EXPECT_TRUE(cb1->success);
  EXPECT_TRUE(cb2->success);
  EXPECT_NE(cb1->initial_call_id, cb2->initial_call_id);
}

class MultiThreadedCallback : public ros::CallbackInterface
{
public:
  MultiThreadedCallback(boost::barrier* bar)
  : barrier_(bar)
  {}

  ros::CallbackInterface::CallResult call()
  {
    barrier_->wait();

    return Success;
  }

private:
  boost::barrier* barrier_;
};
typedef boost::shared_ptr<MultiThreadedCallback> MultiThreadedCallbackPtr;

TEST(CallbackQueueManager, multiThreaded)
{
  for (uint32_t j = 0; j < 1000; ++j)
  {
    //ROS_INFO_COND(j % 1000 == 0, "%d", j);
    CallbackQueueManager man(1);
    CallbackQueuePtr queue(new CallbackQueue(&man));
    man.addQueue(queue, true);

    uint32_t num_threads = man.getNumWorkerThreads();
    boost::barrier bar(num_threads + 1);

    MultiThreadedCallbackPtr cb(new MultiThreadedCallback(&bar));
    for (uint32_t i = 0; i < num_threads; ++i)
    {
      queue->addCallback(cb, 0);
    }

    bar.wait();
    queue->removeByID(0);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
