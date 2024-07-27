// Copyright <2020> [Copyright rossihwang@gmail.com]

#include "mini360_lidar.hpp"

#include <cmath>
#include <limits>
#include <rcl_interfaces/msg/parameter.hpp>

constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xaa;
constexpr uint8_t kSync3 = 0x23;
constexpr uint8_t kSync4 = 0x10;
const int value = 400;
constexpr double kIndexMultiplier = value / 360.0;

Mini360Lidar::Mini360Lidar(const rclcpp::NodeOptions &options)
    : Node("mini360_lidar_driver", options),
      io_context_(std::make_shared<drivers::common::IoContext>()),
      serial_driver_(*io_context_),
      frame_id_("laser"),
      port_("/dev/ttyUSB0"),
      state_(State::SYNC1),
      buffer_index_(0)
{
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  // scan_pub2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 10);

  frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");

  reset_data();

  using namespace drivers::serial_driver;
  using namespace std::placeholders;

  SerialPortConfig port_config(230400, FlowControl::NONE, Parity::NONE, StopBits::ONE);

  try
  {
    serial_driver_.init_port(port_, port_config);
    if (!serial_driver_.port()->is_open())
    {
      serial_driver_.port()->open();
      serial_driver_.port()->async_receive(std::bind(&Mini360Lidar::parse, this, _1, _2));
    }
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", port_.c_str(), ex.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "%s is open", port_.c_str());
}

Mini360Lidar::~Mini360Lidar()
{
  if (io_context_)
  {
    io_context_->waitForExit();
  }
}

void Mini360Lidar::parse(const std::vector<uint8_t> &buffer_raw, const size_t &bytes_transferred)
{
  std::vector<uint8_t> buffer(buffer_raw.begin(), buffer_raw.begin() + bytes_transferred);

  auto it = buffer.begin();
  while (it != buffer.end())
  {
    switch (state_)
    {
    case State::SYNC1:
      if (*it++ == kSync1)
      {
        state_ = State::SYNC2;
      }
      break;
    case State::SYNC2:
      if (*it++ == kSync2)
      {
        state_ = State::SYNC3;
      }
      else
      {
        state_ = State::SYNC1;
      }
      break;
    case State::SYNC3:
      if (*it++ == kSync3)
      {
        state_ = State::SYNC4;
      }
      else
      {
        state_ = State::SYNC1;
      }
      break;
    case State::SYNC4:
      if (*it++ == kSync4)
      {
        state_ = State::DATA;
      }
      else
      {
        state_ = State::SYNC1;
      }
      break;
    case State::DATA:
    {
      while (it != buffer.end() && buffer_index_ < 56)
      {
        buffer_[buffer_index_++] = *it++;
      }

      if (buffer_index_ < 56)
      {
        break;
      }

      buffer_index_ = 0;

      float start_angle = ((((buffer_[3] & 0x7F) << 8) + buffer_[2]) - 0x2000) / 64.0;
      float end_angle = ((((buffer_[53] & 0x7F) << 8) + buffer_[52]) - 0x2000) / 64.0;

      // RCLCPP_INFO(get_logger(), "a %f %f", start_angle, end_angle);

      float angle_res;
      if (end_angle < start_angle)
      {
        angle_res = (end_angle + 360.0 - start_angle) / 16.0;
      }
      else
      {
        angle_res = (end_angle - start_angle) / 16.0;
      }

      // RCLCPP_INFO(get_logger(), "%d %f", start_angle2, start_angle);

      for (int i = 0; i < 16; ++i)
      {
        uint16_t range;
        memcpy(&range, buffer_ + 4 + 3 * i, sizeof(range));
        uint8_t intensity = buffer_[4 + 3 * i + 2];

        double measured_angle = start_angle + angle_res * i;

        // RCLCPP_INFO(get_logger(), "%f %f %f", start_angle, measured_angle, end_angle);

        int angle_index = std::round(measured_angle * kIndexMultiplier);
        angle_index %= value;
        angle_index = value - 1 - angle_index;

        range = range & 0x3fff;

        ranges_[angle_index] = static_cast<float>(range) / 1000.0;
        intensities_[angle_index] = static_cast<float>(intensity);
      }

      if (end_angle < start_angle)
      {
        sensor_msgs::msg::LaserScan message;
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        message.angle_increment = (2.0 * M_PI) / (double)value;
        message.angle_min = 0.0;
        message.angle_max = 2.0 * M_PI - message.angle_increment;
        message.scan_time = 0.001;
        message.range_min = 0.08; // camsense x1 spec
        message.range_max = 8;    // camsense x1 spec
        message.ranges = ranges_;
        message.intensities = intensities_;
        scan_pub_->publish(message);
        // reset_data();
      }

      // if you want a fast response you can try use this

      // sensor_msgs::msg::LaserScan message;
      // message.header.stamp = now();
      // message.header.frame_id = frame_id_;
      // message.angle_increment = (2.0 * M_PI) / (double)value;
      // message.angle_min = 0.0;
      // message.angle_max = 2.0 * M_PI - message.angle_increment;
      // message.scan_time = 0.001;
      // message.range_min = 0.08; // camsense x1 spec
      // message.range_max = 8;    // camsense x1 spec
      // message.ranges = ranges_;
      // message.intensities = intensities_;
      // scan_pub2_->publish(message);

      state_ = State::SYNC1;
      break;
    }
    default:
      break;
    }
  }
}

void Mini360Lidar::reset_data()
{
  ranges_.resize(value);
  intensities_.resize(value);

  for (int i = 0; i < value; ++i)
  {
    ranges_[i] = std::numeric_limits<double>::quiet_NaN();
    intensities_[i] = 0;
  }
}
