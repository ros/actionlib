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

#include <gtest/gtest.h>
#include <sys/time.h>
#include "filters/param_test.h"

using namespace filters ;



TEST(Parameters, Double)
{
  ros::NodeHandle nh;
  double epsilon = 1e-6;
  
  FilterBase<double > * filter = new ParamTest<double>  ();
  EXPECT_TRUE(filter->configure("TestDouble", nh));
  double out;
  filter -> update(out, out);
  EXPECT_NEAR(4,  out, epsilon);
}

TEST(Parameters, Int)
{
  ros::NodeHandle nh;
  
  FilterBase<int> * filter = new ParamTest<int>  ();
  EXPECT_TRUE(filter->configure("TestInt", nh));
  int out;
  filter -> update(out, out);
  EXPECT_EQ(4,  out);
}

TEST(Parameters, UInt)
{
  ros::NodeHandle nh;
  
  FilterBase<unsigned int> * filter = new ParamTest<unsigned int>  ();
  EXPECT_TRUE(filter->configure("TestUInt", nh));
  unsigned int out;
  filter -> update(out, out);
  EXPECT_EQ(4,  out);
}

TEST(Parameters, String)
{
  ros::NodeHandle nh;
  
  FilterBase<std::string> * filter = new ParamTest<std::string>  ();
  EXPECT_TRUE(filter->configure("TestString", nh));
  std::string out;
  filter -> update(out, out);
  EXPECT_STREQ("four",  out.c_str());
}

TEST(Parameters, DoubleVector)
{
  ros::NodeHandle nh;
  double epsilon = 1e-6;
  
  FilterBase<std::vector<double> > * filter = new ParamTest<std::vector<double> >  ();
  EXPECT_TRUE(filter->configure("TestDoubleVector", nh));
  std::vector<double> out;
  filter -> update(out, out);
  for (std::vector<double>::iterator it = out.begin(); it != out.end(); ++it)
    {
      EXPECT_NEAR(4,  *it, epsilon);
    }
}

TEST(Parameters, StringVector)
{
  ros::NodeHandle nh;
  
  FilterBase<std::vector<std::string> > * filter = new ParamTest<std::vector<std::string> >  ();
  EXPECT_TRUE(filter->configure("TestStringVector", nh));
  std::vector<std::string> out;
  filter -> update(out, out);
  for (std::vector<std::string>::iterator it = out.begin(); it != out.end(); ++it)
    {
      EXPECT_STREQ("four",  it->c_str());
    }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mean");
  return RUN_ALL_TESTS();
}
