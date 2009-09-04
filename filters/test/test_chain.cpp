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

#include "gtest/gtest.h"
#include "filters/filter_chain.h"



TEST(MultiChannelFilterChain, configuring){
  double epsilon = 1e-9;
  filters::MultiChannelFilterChain<double> chain("double");

  EXPECT_TRUE(chain.configure(5, "MultiChannelMeanFilterDouble5"));
 
  double input1[] = {1,2,3,4,5};
  double input1a[] = {9,9,9,9,9};//seed w/incorrect values
  std::vector<double> v1 (input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a (input1a, input1a + sizeof(input1a) / sizeof(double));

  
  EXPECT_TRUE(chain.update(v1, v1a));

  chain.clear();

  for (unsigned int i = 1; i < v1.size(); i++)
  {
    EXPECT_NEAR(input1[i], v1a[i], epsilon);
  }
}
TEST(FilterChain, configuring){
  double epsilon = 1e-9;
  filters::FilterChain<float> chain("float");
  
  EXPECT_TRUE(chain.configure("MeanFilterFloat5"));
 
  float v1 = 1;
  float v1a = 9;

  EXPECT_TRUE(chain.update(v1, v1a));

  chain.clear();

  EXPECT_NEAR(v1, v1a, epsilon);
  
  }

TEST(MultiChannelFilterChain, MisconfiguredNumberOfChannels){
  filters::MultiChannelFilterChain<double> chain("double");

  EXPECT_TRUE(chain.configure(10, "MultiChannelMedianFilterDouble5"));

  //  EXPECT_TRUE(chain.configure(10));

  double input1[] = {1,2,3,4,5};
  double input1a[] = {1,2,3,4,5};
  std::vector<double> v1 (input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a (input1a, input1a + sizeof(input1a) / sizeof(double));

  
  EXPECT_FALSE(chain.update(v1, v1a));

  chain.clear();

}

TEST(MultiChannelFilterChain, TwoFilters){
  double epsilon = 1e-9;
  filters::MultiChannelFilterChain<double> chain("double");

  EXPECT_TRUE(chain.configure(5, "TwoFilters"));
 
  double input1[] = {1,2,3,4,5};
  double input1a[] = {9,9,9,9,9};//seed w/incorrect values
  std::vector<double> v1 (input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a (input1a, input1a + sizeof(input1a) / sizeof(double));

  
  EXPECT_TRUE(chain.update(v1, v1a));

  chain.clear();

  for (unsigned int i = 1; i < v1.size(); i++)
  {
    EXPECT_NEAR(input1[i], v1a[i], epsilon);
  }
}


TEST(MultiChannelFilterChain, TransferFunction){
  double epsilon = 1e-4;
 
  filters::MultiChannelFilterChain<double> chain("double");
  EXPECT_TRUE(chain.configure(3, "TransferFunction" ));
 
  std::vector<double> in1,in2,in3,in4,in5,in6,in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in1.push_back(10.0);
  in1.push_back(10.0);
  //
  in2.push_back(70.0);
  in2.push_back(30.0);
  in2.push_back(8.0);
  //
  in3.push_back(-1.0);
  in3.push_back(5.0);
  in3.push_back(22.0);
  //
  in4.push_back(44.0);
  in4.push_back(23.0);
  in4.push_back(8.0);
  //
  in5.push_back(10.0);
  in5.push_back(10.0);
  in5.push_back(10.0);
  //
  in6.push_back(5.0);
  in6.push_back(-1.0);
  in6.push_back(5.0);
  //
  in7.push_back(6.0);
  in7.push_back(-30.0);
  in7.push_back(2.0);
  //
  out1.push_back(17.1112);
  out1.push_back(9.0285);
  out1.push_back(8.3102);
  EXPECT_TRUE(chain.update(in1, in1));
  EXPECT_TRUE(chain.update(in2, in2));
  EXPECT_TRUE(chain.update(in3, in3));
  EXPECT_TRUE(chain.update(in4, in4));
  EXPECT_TRUE(chain.update(in5, in5));
  EXPECT_TRUE(chain.update(in6, in6));
  EXPECT_TRUE(chain.update(in7, in7));

  chain.clear();

  for(unsigned int i=0; i<out1.size(); i++)
  {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}

/*
TEST(MultiChannelFilterChain, OverlappingNames){
  filters::MultiChannelFilterChain<double> chain("double");


  std::string bad_xml = "<filters> <filter type=\"MultiChannelMeanFilterDouble\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter><filter type=\"MedianFilter\" name=\"mean_test\"> <params number_of_observations=\"5\"/></filter></filters>";

  TiXmlDocument chain_def = TiXmlDocument();
  chain_def.Parse(bad_xml.c_str());
  TiXmlElement * config = chain_def.RootElement();

  EXPECT_FALSE(chain.configure(5, config));

}
*/

TEST(FilterChain, ReconfiguringChain){
  filters::FilterChain<int> chain("int");
  
  int v1 = 1;
  int v1a = 9;

  EXPECT_TRUE(chain.configure("OneIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(2, v1a);
  chain.clear();
  
  EXPECT_TRUE(chain.configure("TwoIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(3, v1a);
  chain.clear();
  
}

TEST(FilterChain, ThreeIncrementChains){
  filters::FilterChain<int> chain("int");  
  int v1 = 1;
  int v1a = 9;

  EXPECT_TRUE(chain.configure("ThreeIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(4, v1a);
  chain.clear();
    
}

TEST(FilterChain, TenIncrementChains){
  filters::FilterChain<int> chain("int");  
  int v1 = 1;
  int v1a = 9;

  EXPECT_TRUE(chain.configure("TenIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(11, v1a);
  chain.clear();
    
}
/*
TEST(MultiChannelFilterChain, ReconfiguringMultiChannelChain){
  filters::MultiChannelFilterChain<int> chain("int");
  
  int v1 = 1;
  int v1a = 9;

  EXPECT_TRUE(chain.configure("OneMultiChannelIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(2, v1a);
  chain.clear();
  
  EXPECT_TRUE(chain.configure("TwoMultiChannelIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(3, v1a);
  chain.clear();
  
}

TEST(MultiChannelFilterChain, ThreeMultiChannelIncrementChains){
  filters::MultiChannelFilterChain<int> chain("int");  
  int v1 = 1;
  int v1a = 9;

  EXPECT_TRUE(chain.configure("ThreeMultiChannelIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(4, v1a);
  chain.clear();
    
}
*/
TEST(MultiChannelFilterChain, TenMultiChannelIncrementChains){
  filters::MultiChannelFilterChain<int> chain("int");  
  std::vector<int> v1;
  v1.push_back(1);
  v1.push_back(1);
  v1.push_back(1);
  std::vector<int> v1a = v1;

  EXPECT_TRUE(chain.configure(3, "TenMultiChannelIncrements")); 
  EXPECT_TRUE(chain.update(v1, v1a));
  for (unsigned int i = 0; i < 3; i++)
  {
    EXPECT_EQ(11, v1a[i]);
  }
  chain.clear();
    
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_chain");
  return RUN_ALL_TESTS();
}
