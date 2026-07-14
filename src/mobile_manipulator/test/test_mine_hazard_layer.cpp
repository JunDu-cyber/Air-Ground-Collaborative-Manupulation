#include <memory>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layered_costmap.h>
#include <mobile_manipulator/MineMission.h>
#include <mobile_manipulator/mine_hazard_layer.h>
#include <tf2_ros/buffer.h>
#include <uav_truth_tracker/MineMap.h>

namespace
{

bool waitForSubscribers(const ros::Publisher& publisher, double seconds = 2.0)
{
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(seconds);
  while (ros::ok() && publisher.getNumSubscribers() == 0 &&
         ros::WallTime::now() < deadline)
    ros::WallDuration(0.01).sleep();
  return publisher.getNumSubscribers() > 0;
}

TEST(MineHazardLayer, SuspendsCarriedSourceAndRestoresAfterLockLossOrReset)
{
  tf2_ros::Buffer tf_buffer;
  costmap_2d::LayeredCostmap layered("map", false, false);
  layered.resizeMap(120, 120, 0.1, -6.0, -6.0, false);

  boost::shared_ptr<mobile_manipulator::MineHazardLayer> layer(
      new mobile_manipulator::MineHazardLayer());
  layered.addPlugin(layer);
  layer->initialize(&layered, "mine_hazard_layer", &tf_buffer);

  ros::NodeHandle nh;
  ros::Publisher map_pub = nh.advertise<uav_truth_tracker::MineMap>(
      "/mine_detection/map", 1, true);
  ros::Publisher mission_pub = nh.advertise<mobile_manipulator::MineMission>(
      "/mine_mission/status", 1, true);
  ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>(
      "/mine_hazard/reset", 1, false);
  ASSERT_TRUE(waitForSubscribers(map_pub));
  ASSERT_TRUE(waitForSubscribers(mission_pub));
  ASSERT_TRUE(waitForSubscribers(reset_pub));

  uav_truth_tracker::MineMap mine_map;
  mine_map.header.frame_id = "map";
  uav_truth_tracker::MineMapEntry mine;
  mine.id = 7;
  mine.position.x = 1.0;
  mine.position.y = 2.0;
  mine.confirmed = true;
  mine_map.mines.push_back(mine);
  map_pub.publish(mine_map);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);

  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(layered.getCostmap()->worldToMap(1.0, 2.0, mx, my));
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  // A perception dropout is not permission to drive through a known mine.
  uav_truth_tracker::MineMap empty_map;
  empty_map.header.frame_id = "map";
  map_pub.publish(empty_map);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  mobile_manipulator::MineMission mission;
  mobile_manipulator::MineMissionEntry entry;
  entry.id = 7;
  entry.state = mobile_manipulator::MineMissionEntry::CARRYING;
  mission.entries.push_back(entry);
  mission_pub.publish(mission);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_NE(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  // A new UAV map packet must not repaint the stale pickup coordinate while
  // this exact ID is physically attached to the UGV.
  map_pub.publish(mine_map);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_NE(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  // If the carrying state ends without PLACE/CLEARED, restore the conservative
  // source hazard before permitting any further automatic navigation.
  mission.entries[0].state =
      mobile_manipulator::MineMissionEntry::MANUAL_REQUIRED;
  mission_pub.publish(mission);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  mission.entries[0].state = mobile_manipulator::MineMissionEntry::CARRYING;
  mission_pub.publish(mission);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_NE(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  mission.entries[0].state = mobile_manipulator::MineMissionEntry::CLEARED;
  mission_pub.publish(mission);
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_NE(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));

  // Mission reset intentionally begins again from all mines ever confirmed in
  // this run, even if the latest perception packet happened to be empty.
  reset_pub.publish(std_msgs::Empty());
  ros::WallDuration(0.1).sleep();
  layered.updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(costmap_2d::LETHAL_OBSTACLE,
            layered.getCostmap()->getCost(mx, my));
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mine_hazard_layer_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
