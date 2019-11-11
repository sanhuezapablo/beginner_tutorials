/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 * @file test.cpp
 * @brief Tests service call initialized properly, and checks for right output.
 * @author Pablo Sanhueza
 * @copyright 2019 Pablo Sanhueza
 */

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/set_string.h"

/**
 * @brief Testing service call. Makes sure that service call is initialized properly and outputting the right information.
 *
 * @param[in]     TESTSuite
 * @param[in]     testSetString
 *
 * @return     none
 */
TEST(TESTSuite, testSetString) {
  /// Creating NodeHandle
  ros::NodeHandle n;

  /// Create service client
  auto clt = n.serviceClient<beginner_tutorials::set_string>("set_string");

  /// Object srv for service set_string
  beginner_tutorials::set_string srv;

  /// Giving srv input variable a test string
  srv.request.input = "Test";

  /// True if service call is initialized properly
  bool checkService = clt.call(srv);
  EXPECT_TRUE(checkService);

  /// Makes input is actually outputting the right string
  EXPECT_STREQ("Test", srv.response.output.c_str());
}

