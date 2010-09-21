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


#include <bondcpp/bond.h>
#include <gtest/gtest.h>
#include <uuid/uuid.h>
#include <ros/spinner.h>

#include <test_bond/TestBond.h>

const std::string TOPIC = "test_bond_topic";

std::string genId()
{
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

ros::ServiceClient getService()
{
  ros::NodeHandle nh;
  ros::ServiceClient cli = nh.serviceClient<test_bond::TestBond>("test_bond");
  EXPECT_TRUE(cli);
  EXPECT_TRUE(cli.waitForExistence(ros::Duration(5.0)));
  return cli;
}

TEST(ExerciseBondCpp, normal)
{
  std::string id = genId();
  bond::Bond bond(TOPIC, id);
  ros::ServiceClient cli = getService();
  EXPECT_TRUE(cli.isValid());
  EXPECT_TRUE(cli.exists());
  test_bond::TestBond::Request req;
  test_bond::TestBond::Response resp;
  req.topic = TOPIC;
  req.id = id;
  req.delay_death = ros::Duration(2.0);
  ASSERT_TRUE(cli.call(req, resp));
  bond.start();

  EXPECT_TRUE(bond.waitUntilFormed(ros::Duration(2.0)));
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(5.0)));
}

TEST(ExerciseBondCpp, remoteNeverConnects)
{
  std::string id = genId();
  // Never starts the other side of the bond
  bond::Bond bond(TOPIC, id);
  bond.start();
  EXPECT_FALSE(bond.waitUntilFormed(ros::Duration(1.0)));
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(20.0)));
}

TEST(ExerciseBondCpp, heartbeatTimeout)
{
  std::string id = genId();
  bond::Bond bond(TOPIC, id);

  ros::ServiceClient cli = getService();
  test_bond::TestBond::Request req;
  test_bond::TestBond::Response resp;
  req.topic = TOPIC;
  req.id = id;
  req.delay_death = ros::Duration(2.0);
  req.inhibit_death_message = true;
  ASSERT_TRUE(cli.call(req, resp));

  bond.start();

  EXPECT_TRUE(bond.waitUntilFormed(ros::Duration(2.0)));
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(10.0)));
}

TEST(ExerciseBondCpp, cleanLocalDeath)
{
  std::string id = genId();
  bond::Bond bond(TOPIC, id);

  ros::ServiceClient cli = getService();
  test_bond::TestBond::Request req;
  test_bond::TestBond::Response resp;
  req.topic = TOPIC;
  req.id = id;
  req.delay_death = ros::Duration(-1);
  ASSERT_TRUE(cli.call(req, resp));

  bond.start();

  EXPECT_TRUE(bond.waitUntilFormed(ros::Duration(2.0)));
  bond.breakBond();
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(2.0)));
}

TEST(ExerciseBondCpp, localDeathNoAck)
{
  std::string id = genId();
  bond::Bond bond(TOPIC, id);

  ros::ServiceClient cli = getService();
  test_bond::TestBond::Request req;
  test_bond::TestBond::Response resp;
  req.topic = TOPIC;
  req.id = id;
  req.delay_death = ros::Duration(-1);
  req.inhibit_death_message = true;
  ASSERT_TRUE(cli.call(req, resp));

  bond.start();

  EXPECT_TRUE(bond.waitUntilFormed(ros::Duration(2.0)));
  bond.breakBond();
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(5.0)));
}

TEST(ExerciseBondCpp, remoteIgnoresLocalDeath)
{
  std::string id = genId();
  bond::Bond bond(TOPIC, id);

  ros::ServiceClient cli = getService();
  test_bond::TestBond::Request req;
  test_bond::TestBond::Response resp;
  req.topic = TOPIC;
  req.id = id;
  req.delay_death = ros::Duration(-1);
  req.inhibit_death = true;
  ASSERT_TRUE(cli.call(req, resp));

  bond.start();

  EXPECT_TRUE(bond.waitUntilFormed(ros::Duration(2.0)));
  bond.breakBond();
  EXPECT_FALSE(bond.waitUntilBroken(ros::Duration(1.0)));
  EXPECT_TRUE(bond.waitUntilBroken(ros::Duration(10.0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "exercise_bondcpp", true);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  return ret;
};
