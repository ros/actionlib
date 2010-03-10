/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
/**
@mainpage

\author Radu Bogdan Rusu

@b NodeletMUX represent a mux nodelet for topics: it takes N (<=8) input topics, and publishes all of them on one output topic.
**/

#ifndef NODELET_NODELET_MUX_H_
#define NODELET_NODELET_MUX_H_

#include <nodelet/nodelet.h>
#include <boost/make_shared.hpp>
#include <message_filters/time_synchronizer.h>

namespace nodelet
{
  template <typename T, typename Filter>
  class NodeletMUX: public Nodelet
  {
    typedef typename boost::shared_ptr<T> TPtr;
    typedef typename boost::shared_ptr<const T> TConstPtr;
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Nodelet initialization routine. */
      virtual void
        onInit ()
      {
        private_nh_ = getMTPrivateNodeHandle ();
        pub_output_ = private_nh_.template advertise<T> ("output", 1);

        XmlRpc::XmlRpcValue input_topics;
        if (!private_nh_.getParam ("input_topics", input_topics))
        {
          ROS_ERROR ("[nodelet::NodeletMUX::init] Need a 'input_topics' parameter to be set before continuing!");
          return; 
        }
        // Check the type
        switch (input_topics.getType ())
        {
          case XmlRpc::XmlRpcValue::TypeArray:
          {
            if (input_topics.size () == 1)
            {
              ROS_ERROR ("[nodelet::NodeletMUX::init] Only one topic given. Does it make sense to passthrough?");
              return;
            }

            if (input_topics.size () > 8)
            {
              ROS_ERROR ("[nodelet::NodeletMUX::init] More than 8 topics passed!");
              return;
             }

            ROS_INFO_STREAM ("[nodelet::NodeletMUX::init] Subscribing to " << input_topics.size () << " user given topics as inputs:");
            for (int d = 0; d < input_topics.size (); ++d)
              ROS_INFO_STREAM (" - " << (std::string)(input_topics[d]));

            // Subscribe to the filters
            filters_.resize (input_topics.size ());
            for (int d = 0; d < input_topics.size (); ++d)
            {
              filters_[d] = boost::make_shared<Filter> ();
              filters_[d]->subscribe (private_nh_, (std::string)(input_topics[d]), 1);
            }

            switch (input_topics.size ())
            {
              case 2:
              {
                ts2_ = boost::make_shared <message_filters::TimeSynchronizer<T,T> > (3);
                ts2_->connectInput (*filters_[0], *filters_[1]);
                ts2_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input2, this, _1, _2));
                break;
              }
              case 3:
              {
                ts3_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T> > (3);
                ts3_->connectInput (*filters_[0], *filters_[1], *filters_[2]);
                ts3_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input3, this, _1, _2, _3));
                break;
              }
              case 4:
              {
                ts4_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T,T> > (3);
                ts4_->connectInput (*filters_[0], *filters_[1], *filters_[2], *filters_[3]);
                ts4_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input4, this, _1, _2, _3, _4));
                break;
              }
              case 5:
              {
                ts5_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T,T,T> > (3);
                ts5_->connectInput (*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4]);
                ts5_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input5, this, _1, _2, _3, _4, _5));
                break;
              }
              case 6:
              {
                ts6_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T,T,T,T> > (3);
                ts6_->connectInput (*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4], *filters_[5]);
                ts6_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input6, this, _1, _2, _3, _4, _5, _6));
                break;
              }
              case 7:
              {
                ts7_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T,T,T,T,T> > (3);
                ts7_->connectInput (*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4], *filters_[5], *filters_[6]);
                ts7_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input7, this, _1, _2, _3, _4, _5, _6, _7));
                break;
              }
              case 8:
              {
                ts8_ = boost::make_shared <message_filters::TimeSynchronizer<T,T,T,T,T,T,T,T> > (3);
                ts8_->connectInput (*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4], *filters_[5], *filters_[6], *filters_[7]);
                ts8_->registerCallback (boost::bind (&NodeletMUX<T,Filter>::input8, this, _1, _2, _3, _4, _5, _6, _7, _8));
                break;
              }
              default:
              {
                ROS_ERROR ("[nodelet::NodeletMUX::init] Invalid 'input_topics' parameter given!");
                return;
              }
             }
            break;
          }
          default:
          {
            ROS_ERROR ("[nodelet::NodeletMUX::init] Invalid 'input_topics' parameter given!");
            return;
          }
        }
      }

    private:

      void input2 (const TConstPtr &in1, const TConstPtr &in2)
      { pub_output_.publish (in1); pub_output_.publish (in2); }
      void input3 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); }
      void input4 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3, const TConstPtr &in4)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); pub_output_.publish (in4); }
      void input5 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3, const TConstPtr &in4, const TConstPtr &in5)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); pub_output_.publish (in4); pub_output_.publish (in5); }
      void input6 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3, const TConstPtr &in4, const TConstPtr &in5, const TConstPtr &in6)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); pub_output_.publish (in4); pub_output_.publish (in5); pub_output_.publish (in6); }
      void input7 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3, const TConstPtr &in4, const TConstPtr &in5, const TConstPtr &in6, const TConstPtr &in7)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); pub_output_.publish (in4); pub_output_.publish (in5); pub_output_.publish (in6); pub_output_.publish (in7); }
      void input8 (const TConstPtr &in1, const TConstPtr &in2, const TConstPtr &in3, const TConstPtr &in4, const TConstPtr &in5, const TConstPtr &in6, const TConstPtr &in7, const TConstPtr &in8)
      { pub_output_.publish (in1); pub_output_.publish (in2); pub_output_.publish (in3); pub_output_.publish (in4); pub_output_.publish (in5); pub_output_.publish (in6); pub_output_.publish (in7); pub_output_.publish (in8); }

      /** \brief ROS local node handle. */
      ros::NodeHandle private_nh_;
      /** \brief The output ROS publisher. */
      ros::Publisher pub_output_;

      /** \brief A vector of message filters. */
      std::vector<boost::shared_ptr<Filter> > filters_;

      /** \brief Various different synchronizers. 
        * \note We need to define one type for each increasing number of messages. This will most likely be rewritten soon using the DynamicTimeSynchronizer.
        */
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T> > ts2_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T> > ts3_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T,T> > ts4_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T,T,T> > ts5_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T,T,T,T> > ts6_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T,T,T,T,T> > ts7_;
      boost::shared_ptr<message_filters::TimeSynchronizer<T,T,T,T,T,T,T,T> > ts8_;
  };

}
#endif
