// Copyright (c) 2019 OUXT Polaris
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

// Headers in this package
#include <pcl_apps/matching/ndt_matching_twist_estimator/ndt_matching_twist_estimator_component.hpp>

// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>

// Headers in STL
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pcl_apps::NdtMatchingTwistEstimatorComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
