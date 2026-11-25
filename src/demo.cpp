/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp_components/register_node_macro.hpp>

#include "ros2_api.h"
#include "ldlidar_driver.h"

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

class LdlidarNode : public rclcpp::Node {
 public:
  explicit LdlidarNode(const rclcpp::NodeOptions & options);
  ~LdlidarNode() override;

 private:
  void OnTimer();

  LaserScanSetting setting_;
  ldlidar::LDLidarDriver* ldlidarnode_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

LdlidarNode::LdlidarNode(const rclcpp::NodeOptions & options) : rclcpp::Node("ldlidar_published", options) {
  auto node = this;
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  int serial_port_baudrate = 0;
  ldlidar::LDType type_name;
	setting_.frame_id = "base_laser";
  setting_.laser_scan_dir = true;
  setting_.enable_angle_crop_func = false;
  setting_.angle_crop_min = 0.0;
  setting_.angle_crop_max = 0.0;
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting_.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting_.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting_.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting_.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting_.angle_crop_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting_.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting_.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting_.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting_.angle_crop_min);
  node->get_parameter("angle_crop_max", setting_.angle_crop_max);

  ldlidarnode_ = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode_->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<topic_name>: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting_.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting_.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting_.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting_.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting_.angle_crop_max);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "LDLiDAR_STL27L") {
    type_name = ldlidar::LDType::STL_27L;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    throw std::runtime_error("Error, input <product_name> is illegal.");
  }

  ldlidarnode_->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

  ldlidarnode_->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode_->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    throw std::runtime_error("ldlidar node start is fail");
  }

  if (ldlidarnode_->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    throw std::runtime_error("ldlidar communication is abnormal.");
  }

  // create ldlidar data topic and publisher
  publisher_ = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  RCLCPP_INFO(node->get_logger(), "Publish topic message:ldlidar scan data.");
  timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this] { OnTimer(); });  // 10hz
}

LdlidarNode::~LdlidarNode() {
  ldlidarnode_->Stop();
  delete ldlidarnode_;
}

void LdlidarNode::OnTimer() {
  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;
  auto node = shared_from_this();
  switch (ldlidarnode_->GetLaserScanData(laser_scan_points, 1500)){
    case ldlidar::LidarStatus::NORMAL:
      ldlidarnode_->GetLidarScanFreq(lidar_scan_freq);
      ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting_, node, publisher_);
      break;
    case ldlidar::LidarStatus::DATA_TIME_OUT:
      RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
      break;
    case ldlidar::LidarStatus::DATA_WAIT:
      break;
    default:
      break;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LdlidarNode)

void  ToLaserscanMessagePublish(ldlidar::Points2D& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = node->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }
  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 25;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);
  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        // min > max means the masked range wraps through 0 deg (e.g. 310..50)
        bool in_crop = (setting.angle_crop_min <= setting.angle_crop_max)
          ? (dir_angle >= setting.angle_crop_min && dir_angle <= setting.angle_crop_max)
          : (dir_angle >= setting.angle_crop_min || dir_angle <= setting.angle_crop_max);
        if (in_crop) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(node->get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
            index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub->publish(output);
    end_scan_time = start_scan_time;
  } 
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/