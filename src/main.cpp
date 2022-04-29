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
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "Arduino.h"
#include "Wire.h"

#include "SparkFun_VL53L5CX_Library.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  SparkFun_VL53L5CX myImager;
  int imageResolution = 0; //Used to pretty print output
  int imageWidth = 0; //Used to pretty print output
  public:
    I2CPublisher()
    : Node("i2cpublisher")
    , _id(0) 
    {
      get_parameter_or<uint8_t>("id", _id, 0x52); 

      Wire.begin();
      myImager.begin();
      
      myImager.setResolution(8*8); //Enable all 64 pads
      
      imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
      imageWidth = sqrt(imageResolution); //Calculate printing width

      myImager.startRanging();

      image_transport::ImageTransport transport(shared_from_this());

      _depth_raw_publisher = transport.advertise("depth/image_raw", 1, true);
      _depth_raw_camerainfo_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 1);
  
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
        if (myImager.getRangingData(&measurementData)) //Read distance data into array
        {
          cv::Mat depth_frame_buffer_mat(8, 8, CV_16UC1, measurementData.distance_mm);

          auto depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();

          _depth_raw_publisher.publish(depth_image);

          sensor_msgs::msg::CameraInfo camera_info;
          camera_info.header.frame_id = "camera"; // todo make configurable
          camera_info.width = 8;
          camera_info.height = 8;
          camera_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

          _depth_raw_camerainfo_publisher->publish(camera_info);
        }
      }
    }
    rclcpp::TimerBase::SharedPtr _timer;

    image_transport::Publisher _depth_raw_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _depth_raw_camerainfo_publisher;
  
    uint8_t _id;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}