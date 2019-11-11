/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 * @file msin.cpp
 * @brief Runs all tests
 * @author Pablo Sanhueza
 * @copyright 2019 Pablo Sanhueza
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_talker");
  testing::InitGoogleTest(&argc, argv);
  /// Creating NodeHandle
  ros::NodeHandle n;
  return RUN_ALL_TESTS();
}
