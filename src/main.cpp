/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "Arduino.h"
#include "Wire.h"

#include "SparkFun_VL53L5CX_Library.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr uint8_t kResolution = 8;

constexpr inline float DegToRad(float deg)
{
  return (deg * M_PI) / 180.0f;
}

class I2CPublisher : public rclcpp::Node
{
  public:
    I2CPublisher()
    : Node("i2cpublisher")
    , _id(0)
    {
    }

    void initialize()
    {
      get_parameter_or<uint8_t>("i2c_address", _id, 0x29); 
      get_parameter_or<std::string>("frame_id", _frameId, "depth"); 
      get_parameter_or<std::string>("topic", _topic, "points2"); 
      get_parameter_or<double>("poll", _poll, 15.0);
      get_parameter_or<bool>("debug", _debug, false);

      get_parameter_or<uint8_t>("multiplexer_address", _multiplexerId, 0); 
      std::vector<int> ports;
      std::vector<std::string> frameIds;

      if (_multiplexerId != 0)
      {
        rclcpp::Parameter portParam;
        if (get_parameter<std::vector<int>>("multiplexer_ports", portParam))
        {
          ports = portParam.as_integer_array();
        }
        else
        {
          RCLCPP_FATAL(get_logger(), "If a multiplexer address is specified, ports must be specified as well");
          rclcpp::shutdown();
          return;
        }

        rclcpp::Parameter frameIdParam;
        if (get_parameter<std::vector<int>>("frame_ids", frameIdParam))
        {
          frameIds = frameIdParam.as_string_array();
        }
        else
        {
          RCLCPP_FATAL(get_logger(), "If a multiplexer address is specified, frame ids must be specified as well");
          rclcpp::shutdown();
          return;
        }
      }
      else
      {
          ports.push_back(0); // pins unused.
          frameIds.push_back(_frameId);
      }

      if (ports.size() != frameIds.size())
      {
        RCLCPP_FATAL(get_logger(), "The number of ports must match the number of frames.");
        rclcpp::shutdown();
        return;
      }

      _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      _tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);      

      Wire.begin();
      Wire.setAddressSize(2); 
      Wire.setPageBytes(256);
      
      for (size_t i = 0; i < _ports.size(); i++)
      {
        auto imagerInstance = std::make_shared<ImagerInstance>(frameIds[i], ports[i]);
        _imagers.push_back(imagerInstance);

        switchToImager(imagerInstance.port);

        imagerInstance.imager.begin(_id, Wire);
        
        imagerInstance.imager.setResolution(kResolution * kResolution); //Enable all 64 pads
        
        imagerInstance.imager.startRanging();
      }

      _pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic, 1);
      if (_debug)
      {
        _marker = this->create_publisher<visualization_msgs::msg::Marker>("tof_debug", 1);
      }

      _timer = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(_poll), 
        std::bind(&I2CPublisher::timer_callback, this));
    }

  private:

    void switchToImager(uint8_t port)
    {
      if (_multiplexerId == 0)
      {
        return;
      }

      if (i > 7) 
      {
        return;
      }
      
      Wire.beginTransmission(_multiplexerId);
      Wire.write(1 << port);
      Wire.endTransmission();  
    }

    void timer_callback()
    {
      for (auto& imager : _imagers)
      {  
        if (imager.isDataReady() == true)
        {
          // Store in local variable in case the range fails
          VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
          memset(&measurementData, 0, sizeof(VL53L5CX_ResultsData)); 
          if (imager.getRangingData(&measurementData)) //Read distance data into array
          {
            std::memmove(measurementData, imager.measurementData, sizeof(VL53L5CX_ResultsData));
          }
        }
      }
    
      auto point_cloud = sensor_msgs::msg::PointCloud2();

      point_cloud.header.frame_id = _frameId;
      point_cloud.header.stamp = rclcpp::Clock().now();
      point_cloud.height = kResolution * _imagers.size();
      point_cloud.width = kResolution * _imagers.size();
      point_cloud.is_dense = false;
      point_cloud.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);        
      pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

      sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");

      pcd_modifier.resize(point_cloud.height * point_cloud.width);

      for (auto& imager : _imagers)
      {
        geometry_msgs::msg::TransformStamped tMsg;

        if (_multiplexerId != 0)
        {
          try 
          {
            tMsg = _tf_buffer->lookupTransform(_frameId, imager.frame, tf2::TimePointZero);
          } 
          catch (const tf2::TransformException & ex) 
          {
            RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            continue;
          }

          tf2::Transform tfTransform;
          tf2::fromMsg(tMsg.transform, tfTransform);        
        }

        for (size_t w = 0; w < kResolution; w++)
        {
          for (size_t h = 0; h < kResolution; h++)
          {
            float depth = static_cast<float>(measurementData.distance_mm[w + (kResolution * h)]);

            if (depth <= 0.0f)
            {
              *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            }
            else
            {
              constexpr float kMillimeterToMeter = 1.0f / 1000.0f;
              constexpr float kFov = DegToRad(45.0f);
              float fovPerPixel = kFov / static_cast<float>(kResolution);

              tf2:Vector pt(
                kMillimeterToMeter * cos(w * fovPerPixel - kFov / 2.0f - DegToRad(90.0f)) * depth,
                kMillimeterToMeter * sin(h * fovPerPixel - kFov / 2.0f) * depth,
                kMillimeterToMeter * depth);

              if (_multiplexerId != 0)
              {
                pt = tfTransform * pt;
              }

              *iter_x = static_cast<float>(pt.x);
              *iter_y = static_cast<float>(pt.y);
              *iter_z = static_cast<float>(pt.z);
            } 

            ++iter_x; ++iter_y; ++iter_z;
          }
        }
      }

      _pointcloud->publish(point_cloud);

      if (_debug)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = _frameId;
        marker.header.stamp = rclcpp::Time();
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_use_embedded_materials = true;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;          
        marker.scale.x = 0.01;
        marker.scale.y = 0.012;
        marker.scale.z = 0.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        geometry_msgs::msg::Point pt;
        marker.points.push_back(pt);
        pt.z = .1;
        marker.points.push_back(pt);

        _marker->publish(marker);
      }
    }

    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker;
    uint8_t _id;
    uint8_t _multiplexerId;
    double _poll;
    std::string _topic;
    std::string _frameId;
    bool _debug;

    struct ImagerInstance
    {
      ImagerInstance(std::string f, uint8_t p)
      : frameId(f)
      , port(p)
      {

      }
      SparkFun_VL53L5CX imager;
      std::string frameId;
      uint8_t port;
      VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
    };

    std::vector<std::shared_ptr<ImagerInstance>> _imagers;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");
    node->declare_parameter("multiplexer_address");
    node->declare_parameter("frame_id");
    node->declare_parameter("poll");
    node->declare_parameter("topic");
    node->declare_parameter("debug");
    node->declare_parameter("frame_ids");
    node->declare_parameter("multiplexer_ports");

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}