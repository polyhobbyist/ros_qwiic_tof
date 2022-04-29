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

#include "Arduino.h"
#include "Wire.h"

#include "SparkFun_VL53L5CX_Library.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  SparkFun_VL53L5CX myImager;
  public:
    I2CPublisher()
    : Node("i2cpublisher")
    , _id(0)
    {
    }

    void initialize()
    {
      get_parameter_or<uint8_t>("id", _id, 0x52); 

      Wire.begin();
      myImager.begin();
      
      myImager.setResolution(8*8); //Enable all 64 pads
      
      _width = sqrt(myImager.getResolution()); //Calculate printing width
      myImager.startRanging();

      _pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);    // todo configurable
  
      // TODO: make this rate configurable
      _timer = this->create_wall_timer(500ms, std::bind(&I2CPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //Poll sensor for new data
      if (myImager.isDataReady() == true)
      {
        VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
        memset(&measurementData, 0, sizeof(VL53L5CX_ResultsData));
        if (myImager.getRangingData(&measurementData)) //Read distance data into array
        {
          auto point_cloud = sensor_msgs::msg::PointCloud2();

          point_cloud.header.frame_id = "depth";   // todo frame id 
          point_cloud.header.stamp = rclcpp::Clock().now();
          point_cloud.height = _width;
          point_cloud.width = _width;
          point_cloud.is_dense = false;
          point_cloud.is_bigendian = false;

          sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
          pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

          sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");

          pcd_modifier.resize(point_cloud.height * point_cloud.width);

          for (size_t w = 0; w < _width; w++)
          {
            for (size_t h = 0; h < _width; h++)
            {
              float z = static_cast<float>(measurementData.distance_mm[w + (_width * h)]);

              if (z <= 0.0f)
              {
                *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
              }
              else
              {
                constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
                constexpr float kFovDeg = 63.0f;
                constexpr float kFov = (kFovDeg * 180.0f) / M_PI;
                float fovPerPixel = kFov / static_cast<float>(_width);


                *iter_x = kMillimeterToMeter * static_cast<float>(sin(w * fovPerPixel - kFov / 2.0f)) * z;
                *iter_y = kMillimeterToMeter * static_cast<float>(sin(h * fovPerPixel - kFov / 2.0f)) * z;
                *iter_z = kMillimeterToMeter * z;
              }

              ++iter_x; ++iter_y; ++iter_z;
            }
          }

          _pointcloud->publish(point_cloud);

        }
      }
    }
    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud;

    uint8_t _width;
  
    uint8_t _id;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}