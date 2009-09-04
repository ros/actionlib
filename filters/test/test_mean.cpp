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
#include "filters/mean.h"

using namespace filters ;

void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

void generate_rand_vectors(double scale, uint64_t runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( uint64_t i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}

TEST(MultiChannelMeanFilterDouble, ConfirmIdentityNRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  
  MultiChannelFilterBase<double > * filter = new MultiChannelMeanFilter<double>  ();
  EXPECT_TRUE(filter->configure(rows, "MultiChannelMeanFilterDouble5"));

  double input1[] = {1,2,3,4,5};
  double input1a[] = {1,2,3,4,5};
  std::vector<double> v1 (input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a (input1a, input1a + sizeof(input1a) / sizeof(double));


  for (int32_t i =0; i < rows*10; i++)
  {
    EXPECT_TRUE(filter->update(v1, v1a));

    for (int i = 1; i < length; i++)
    {
      EXPECT_NEAR(v1[i], v1a[i], epsilon);
    }
  }
}

TEST(MultiChannelMeanFilterDouble, ThreeRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  
  MultiChannelFilterBase<double > * filter = new MultiChannelMeanFilter<double> ();
  EXPECT_TRUE(filter->configure(rows, "MultiChannelMeanFilterDouble5"));

  double input1[] = {0,1,2,3,4};
  std::vector<double> v1 (input1, input1 + sizeof(input1) / sizeof(double));
  double input2[] = {1,2,3,4,5};
  std::vector<double> v2 (input2, input2 + sizeof(input2) / sizeof(double));
  double input3[] = {2,3,4,5,6};
  std::vector<double> v3 (input3, input3 + sizeof(input3) / sizeof(double));
  double input1a[] = {1,2,3,4,5};
  std::vector<double> v1a (input1a, input1a + sizeof(input1a) / sizeof(double));


  EXPECT_TRUE(filter->update(v1, v1a));
  EXPECT_TRUE(filter->update(v2, v1a));
  EXPECT_TRUE(filter->update(v3, v1a));

  for (int i = 1; i < length; i++)
  {
    EXPECT_NEAR(v2[i], v1a[i], epsilon);
  }

}

TEST(MeanFilterDouble, ConfirmIdentityNRows)
{
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;
  
  FilterBase<double > * filter = new MeanFilter<double>  ();
  EXPECT_TRUE(filter->configure("MeanFilterDouble5"));

  double input = 1;
  double output = 0;


  for (int32_t i =0; i < rows*10; i++)
  {
    EXPECT_TRUE(filter->update(input, output));
    
    for (int i = 1; i < length; i++)
    {
      EXPECT_NEAR(input, output, epsilon);
    }
  }
}

TEST(MeanFilterDouble, ThreeRows)
{
  double epsilon = 1e-6;
  
  FilterBase<double > * filter = new MeanFilter<double> ();
  EXPECT_TRUE(filter->configure("MeanFilterDouble5"));

  double input1 = 0;
  double input2 =1;
  double input3 = 2;
  double output = 3;


  EXPECT_TRUE(filter->update(input1, output));
  EXPECT_NEAR(input1, output, epsilon);
  EXPECT_TRUE(filter->update(input2, output));
  EXPECT_NEAR((input1+ input2)/2.0, output, epsilon);
  EXPECT_TRUE(filter->update(input3, output));
  EXPECT_NEAR((input1 + input2 + input3)/3, output, epsilon);


}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mean");
  return RUN_ALL_TESTS();
}
