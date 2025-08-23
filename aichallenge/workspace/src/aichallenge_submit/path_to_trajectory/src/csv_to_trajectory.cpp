// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "path_to_trajectory/csv_to_trajectory.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>


CsvToTrajectory::CsvToTrajectory() : Node("csv_to_trajectory_node")
, cloud_(new pcl::PointCloud<pcl::PointXYZ>())
 {
  using std::placeholders::_1;
  this->declare_parameter<std::string>("csv_file_path", "");
  this->declare_parameter<std::string>("additional_csv_file_path", "");  // 追加のCSVファイルパス
  this->declare_parameter<bool>("enable_additional_trajectory", false);  // 追加経路の有効化フラグ
  this->declare_parameter<float>("velocity_coef", 30.0f);
  this->declare_parameter<float>("trajectory_length", 100.0f);
  this->declare_parameter<float>("trajectory_margin", 2.0f);
  this->declare_parameter<float>("trajectory_rear_length", 10.0f);
  this->declare_parameter<float>("z_position", 0.0f);
  this->declare_parameter<double>("start_trajectory_distance", 100.0);

  std::string csv_file_path;
  std::string additional_csv_file_path;
  this->get_parameter("csv_file_path", csv_file_path);
  this->get_parameter("additional_csv_file_path", additional_csv_file_path);
  this->get_parameter("enable_additional_trajectory", this->enable_additional_trajectory_);
  this->get_parameter("trajectory_length", this->trajectory_length_);
  this->get_parameter("z_position", this->z_position_);
  this->get_parameter("start_trajectory_distance", this->start_trajectory_distance_);
  dynamicLoadParam();

  if (csv_file_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No CSV file path provided");
      rclcpp::shutdown();
      return;
  }
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "in_odom", qos, std::bind(&CsvToTrajectory::odomCallback, this, _1));
  this->pub_ = this->create_publisher<Trajectory>("output", 1);
  this->pub_now_point_ = this->create_publisher<std_msgs::msg::Int32>("now_waypoint", 1);
  
  // メインのCSVファイルを読み込み
  this->readCsv(csv_file_path);
  
  // 追加のCSVファイルが指定されている場合は読み込み
  if (this->enable_additional_trajectory_ && !additional_csv_file_path.empty()) {
    this->readAdditionalCsv(additional_csv_file_path);
    this->mergeTrajectories();
  }
}

void CsvToTrajectory::dynamicLoadParam(){
  this->get_parameter("velocity_coef", this->velocity_coef_);
  this->get_parameter("trajectory_margin", this->trajectory_margin_);
  this->get_parameter("trajectory_rear_length", this->trajectory_rear_length_);
}

void CsvToTrajectory::readCsv(const std::string& file_path) {
  std::ifstream file(file_path);
  std::string line;
  double old_x=0.0;
  double old_y=0.0;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') continue; // Skip empty lines and comments
    std::istringstream s(line);
    std::string field;
    std::vector<double> values;

    while (getline(s, field, ',')) {
      values.push_back(std::stod(field));
    }
    // x,y,z,yaw
    TrajectoryPoint point;
    point.pose.position.x = values[1];
    point.pose.position.y = values[2];
    point.pose.position.z = z_position_;
    //const double yaw = std::atan2(point.pose.position.y-old_y, point.pose.position.x-old_x);
    const double yaw = values[3] + M_PI/2;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = sin(yaw / 2);
    point.pose.orientation.w = cos(yaw / 2);
    point.longitudinal_velocity_mps = values[5];
    point.acceleration_mps2 = 0.0; //values[6];

    trajectory_points_.push_back(point);
    old_x = point.pose.position.x;
    old_y = point.pose.position.y;
    cloud_->points.push_back(pcl::PointXYZ(point.pose.position.x, point.pose.position.y, z_position_));
  }
  // double yaw = std::atan2(trajectory_points_.front().pose.position.y-old_y, trajectory_points_.front().pose.position.x-old_x);
  // trajectory_points_.front().pose.orientation.x = 0.0;
  // trajectory_points_.front().pose.orientation.y = 0.0;
  // trajectory_points_.front().pose.orientation.z = sin(yaw / 2);
  // trajectory_points_.front().pose.orientation.w = cos(yaw / 2);
  RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points", trajectory_points_.size());

  kdtree_.setInputCloud(cloud_);
  
}

void CsvToTrajectory::readAdditionalCsv(const std::string& file_path) {
  std::ifstream file(file_path);
  std::string line;
  double old_x = 0.0;
  double old_y = 0.0;
  
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open additional CSV file: %s", file_path.c_str());
    return;
  }
  
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') continue; // Skip empty lines and comments
    std::istringstream s(line);
    std::string field;
    std::vector<double> values;

    while (getline(s, field, ',')) {
      values.push_back(std::stod(field));
    }
    
    // x,y,z,yaw
    TrajectoryPoint point;
    point.pose.position.x = values[1];
    point.pose.position.y = values[2];
    point.pose.position.z = z_position_;
    const double yaw = values[3] + M_PI/2;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = sin(yaw / 2);
    point.pose.orientation.w = cos(yaw / 2);
    point.longitudinal_velocity_mps = values[5];
    point.acceleration_mps2 = 0.0; //values[6];

    additional_trajectory_points_.push_back(point);
    old_x = point.pose.position.x;
    old_y = point.pose.position.y;
  }
  
  RCLCPP_INFO(this->get_logger(), "Loaded %zu additional trajectory points", additional_trajectory_points_.size());
}

void CsvToTrajectory::mergeTrajectories() {
  if (additional_trajectory_points_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No additional trajectory points to merge");
    return;
  }
  
  // メインの経路情報の前に追加の経路情報を挿入
  std::vector<TrajectoryPoint> merged_trajectory;
  
  // 追加の経路情報を先頭に追加
  merged_trajectory.insert(merged_trajectory.end(), additional_trajectory_points_.begin(), additional_trajectory_points_.end());
  
  // メインの経路情報を追加
  merged_trajectory.insert(merged_trajectory.end(), trajectory_points_.begin(), trajectory_points_.end());
  
  // マージされた経路情報で置き換え
  trajectory_points_ = merged_trajectory;
  
  // ポイントクラウドを再構築
  cloud_merged_->points.clear();
  for (const auto& point : trajectory_points_) {
    cloud_merged_->points.push_back(pcl::PointXYZ(point.pose.position.x, point.pose.position.y, z_position_));
  }
  
  // KDTreeを再構築
  kdtree_merged_.setInputCloud(cloud_merged_);

  RCLCPP_INFO(this->get_logger(), "Merged trajectories: total %zu points", trajectory_points_.size());
}

void CsvToTrajectory::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  // odometryの走行距離を記録
  static double last_x = odometry->pose.pose.position.x;
  static double last_y = odometry->pose.pose.position.y;
  static double odom_distance = 0.0;

  double current_x = odometry->pose.pose.position.x;
  double current_y = odometry->pose.pose.position.y;
  double dx = current_x - last_x;
  double dy = current_y - last_y;
  odom_distance += std::sqrt(dx * dx + dy * dy);

  last_x = current_x;
  last_y = current_y;
  if (current_point_index_ >= trajectory_points_.size()) return;

  dynamicLoadParam();

  Trajectory trajectory;
  // Set trajectory header
  trajectory.header = odometry->header;

  const int K = 1; // 1点のみ取得
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  pcl::PointXYZ searchPoint(odometry->pose.pose.position.x, odometry->pose.pose.position.y, z_position_);
  //if(start_trajectory_distance_  > odom_distance && enable_additional_trajectory_){
    kdtree_merged_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
  // }
  // else{
  //   kdtree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
  // }

  int start_index = pointIdxNKNSearch[0]- std::round(trajectory_rear_length_/trajectory_margin_);
  if(start_index<0){
    start_index+=trajectory_points_.size();
  }
  const int points_num = std::round((trajectory_length_+trajectory_rear_length_)/trajectory_margin_);
  for(int i = 0; i < points_num; i++){
    int p = start_index + i;
    if(p>=trajectory_points_.size()){
      p-=(trajectory_points_.size());
    }
    trajectory.points.push_back(trajectory_points_[p]);
  }
  for(auto && p : trajectory.points){
    p.longitudinal_velocity_mps*=this->velocity_coef_;
  }
  pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
