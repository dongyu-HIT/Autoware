/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gtest/gtest.h>

#include "freespace_planner/astar_navi.h"

class TestClass
{
private:
  AstarNavi node_;
  ros::NodeHandle nh_;
  nav_msgs::OccupancyGrid costmap_free_, costmap_occ_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  ros::Publisher goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  ros::Publisher costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
  ros::Subscriber lane_sub_ = nh_.subscribe("lane_waypoints_array", 1, &TestClass::callbackLane, this);

public:
  geometry_msgs::PoseStamped current_pose_, goal_pose_;
  autoware_msgs::LaneArray lane_arr_;

  TestClass()
  {
    // create costmap
    costmap_free_.header.seq = 1;
    costmap_free_.header.stamp = ros::Time::now();
    costmap_free_.header.frame_id = "world";
    costmap_free_.info.map_load_time = costmap_free_.header.stamp;
    costmap_free_.info.resolution = 1;
    costmap_free_.info.width = 500;
    costmap_free_.info.height = 500;
    costmap_free_.info.origin.position.x = -costmap_free_.info.resolution * costmap_free_.info.width / 2;
    costmap_free_.info.origin.position.y = -costmap_free_.info.resolution * costmap_free_.info.height / 2;
    costmap_free_.info.origin.position.z = 0;
    costmap_free_.info.origin.orientation.x = 0;
    costmap_free_.info.origin.orientation.y = 0;
    costmap_free_.info.origin.orientation.z = 0;
    costmap_free_.info.origin.orientation.w = 1;
    costmap_free_.data = std::vector<signed char>(costmap_free_.info.width * costmap_free_.info.height, 0);

    costmap_occ_.header = costmap_free_.header;
    costmap_occ_.info = costmap_free_.info;

    for (int row = 0; row < costmap_occ_.info.height; ++row)
    {
      for (int col = 0; col < costmap_occ_.info.width; ++col)
      {
        // pass
      }
    }
  }

  ~TestClass()
  {
  }

  void broadcastTF(double x, double y, double z)
  {
    // create/broadcast tf
    tf::Transform world2map, map2velodyne;
    world2map.setOrigin(tf::Vector3(1e-3, 1e-3, 1e-3));
    tf_broadcaster_.sendTransform(tf::StampedTransform(world2map, ros::Time::now(), "world", "map"));
    map2velodyne.setOrigin(tf::Vector3(x, y, z));
    tf_broadcaster_.sendTransform(tf::StampedTransform(map2velodyne, ros::Time::now(), "map", "velodyne"));
  }

  void publishCurrentPose(double x, double y, double z)
  {
    // create/publish current pose
    current_pose_.header.seq = 1;
    current_pose_.header.stamp = ros::Time::now();
    current_pose_.header.frame_id = "world";
    current_pose_.pose.position.x = x;
    current_pose_.pose.position.y = y;
    current_pose_.pose.position.z = z;
    current_pose_.pose.orientation.w = 1;
    current_pose_pub_.publish(current_pose_);
  }

  void publishGoalPose(double x, double y, double z)
  {
    // create/publish current pose
    goal_pose_.header.seq = 1;
    goal_pose_.header.stamp = ros::Time::now();
    goal_pose_.header.frame_id = "world";
    goal_pose_.pose.position.x = x;
    goal_pose_.pose.position.y = y;
    goal_pose_.pose.position.z = z;
    goal_pose_.pose.orientation.w = 1;
    goal_pose_pub_.publish(goal_pose_);
  }

  void publishCostmapFree()
  {
    costmap_pub_.publish(costmap_free_);
  }

  void callbackLane(const autoware_msgs::LaneArray& msg)
  {
    lane_arr_ = msg;
  }

  void update()
  {
    node_.update();
  }
};

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }
  TestClass test_;
};

TEST_F(TestSuite, CheckPlanFree)
{
  test_.broadcastTF(0.1, 0.1, 0.1);
  test_.publishCurrentPose(0, 0, 0);
  test_.publishGoalPose(10, 0, 0);
  test_.publishCostmapFree();

  ros::spinOnce();
  ros::WallDuration(0.3).sleep();
  test_.update();
  ros::WallDuration(0.3).sleep();
  ros::spinOnce();

  const geometry_msgs::Pose& cpose = test_.current_pose_.pose;
  const geometry_msgs::Pose& gpose = test_.goal_pose_.pose;
  const autoware_msgs::LaneArray& lane_arr = test_.lane_arr_;
  const std::vector<autoware_msgs::Lane>& lanes = lane_arr.lanes;
  ASSERT_NE(lanes.size(), 0) << "Lanes size is zero. It should be not zero";
  const std::vector<autoware_msgs::Waypoint>& wps = lanes[0].waypoints;
  ASSERT_NE(wps.size(), 0) << "Waypoints size is zero. It should be not zero";

  bool is_correct_start = std::fabs(wps.front().pose.pose.position.x - cpose.position.x) < 1e-1 &&
                          std::fabs(wps.front().pose.pose.position.y - cpose.position.y) < 1e-1;
  ASSERT_TRUE(is_correct_start) << "Wrong waypoint start. It should be equal current pose" << wps.size();

  bool is_correct_goal = std::fabs(wps.back().pose.pose.position.x - gpose.position.x) < 1.0 &&
                          std::fabs(wps.back().pose.pose.position.y - gpose.position.y) < 1.0;
  ASSERT_TRUE(is_correct_goal) << "Wrong waypoint goal. It should be almost same with goal pose" << wps.end()->pose.pose.position.x << ", " << gpose.position.x << ", " << wps.size();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_node");
  return RUN_ALL_TESTS();
}
