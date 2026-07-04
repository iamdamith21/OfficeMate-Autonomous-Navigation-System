// ltme_node.cpp — ROS 2 (rclcpp) port of LitraTech's LTME-02A / LT-R / LT-I
// LiDAR driver. Ported from the upstream ROS 1 (roscpp) node; the scan
// decoding logic and every ldcp_sdk call are preserved verbatim. Only the
// ROS layer (node handle, params, publisher, services, time, logging) was
// migrated to rclcpp for ROS 2 Humble.
//
// Query services (model/serial/firmware/hardware) are exposed as
// std_srvs/Trigger — the requested value is returned in response->message —
// so no custom .srv interface generation is needed. Hibernation / wake-up /
// quit are std_srvs/Empty, matching the original.

#include <arpa/inet.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ldcp/device.h"

class LidarDriver : public rclcpp::Node
{
public:
  static constexpr const char* DEFAULT_ENFORCED_TRANSPORT_MODE = "none";
  static constexpr const char* DEFAULT_FRAME_ID = "laser";
  static constexpr bool DEFAULT_INVERT_FRAME = false;
  static constexpr int DEFAULT_SCAN_FREQUENCY = 15;
  static constexpr double ANGLE_MIN_LIMIT = -3.142;
  static constexpr double ANGLE_MAX_LIMIT = 3.142;
  static constexpr double DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
  static constexpr double DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
  static constexpr double RANGE_MIN_LIMIT = 0.05;
  static constexpr double RANGE_MAX_LIMIT_02A = 30;
  static constexpr double RANGE_MAX_LIMIT_R1 = 30;
  static constexpr double RANGE_MAX_LIMIT_R2 = 30;
  static constexpr double RANGE_MAX_LIMIT_I1 = 100;
  static constexpr double RANGE_MAX_LIMIT_I2 = 70;
  static constexpr int DEFAULT_AVERAGE_FACTOR = 1;
  static constexpr int DEFAULT_SHADOW_FILTER_STRENGTH = 50;
  static constexpr int DEFAULT_RECEIVER_SENSITIVITY_BOOST = 0;

  LidarDriver();
  void run();

private:
  using Trigger = std_srvs::srv::Trigger;
  using Empty = std_srvs::srv::Empty;

  void queryModelService(const std::shared_ptr<Trigger::Request> request,
                         std::shared_ptr<Trigger::Response> response);
  void querySerialService(const std::shared_ptr<Trigger::Request> request,
                          std::shared_ptr<Trigger::Response> response);
  void queryFirmwareVersion(const std::shared_ptr<Trigger::Request> request,
                            std::shared_ptr<Trigger::Response> response);
  void queryHardwareVersion(const std::shared_ptr<Trigger::Request> request,
                            std::shared_ptr<Trigger::Response> response);
  void requestHibernationService(const std::shared_ptr<Empty::Request> request,
                                 std::shared_ptr<Empty::Response> response);
  void requestWakeUpService(const std::shared_ptr<Empty::Request> request,
                            std::shared_ptr<Empty::Response> response);
  void quitDriverService(const std::shared_ptr<Empty::Request> request,
                         std::shared_ptr<Empty::Response> response);

  std::string device_model_;
  std::string device_address_;
  std::string enforced_transport_mode_;
  std::string frame_id_;
  bool invert_frame_;
  int scan_frequency_override_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
  int shadow_filter_strength_;
  int receiver_sensitivity_boost_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_;
  std::atomic_bool quit_driver_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Service<Trigger>::SharedPtr query_model_service_;
  rclcpp::Service<Trigger>::SharedPtr query_serial_service_;
  rclcpp::Service<Trigger>::SharedPtr query_firmware_service_;
  rclcpp::Service<Trigger>::SharedPtr query_hardware_service_;
  rclcpp::Service<Empty>::SharedPtr request_hibernation_service_;
  rclcpp::Service<Empty>::SharedPtr request_wake_up_service_;
  rclcpp::Service<Empty>::SharedPtr quit_driver_service_;
};

LidarDriver::LidarDriver()
  : rclcpp::Node("ltme_node")
  , hibernation_requested_(false)
  , quit_driver_(false)
{
  device_model_ = declare_parameter<std::string>("device_model", "");
  device_address_ = declare_parameter<std::string>("device_address", "");
  enforced_transport_mode_ = declare_parameter<std::string>("enforced_transport_mode", DEFAULT_ENFORCED_TRANSPORT_MODE);
  frame_id_ = declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);
  invert_frame_ = declare_parameter<bool>("invert_frame", DEFAULT_INVERT_FRAME);
  scan_frequency_override_ = declare_parameter<int>("scan_frequency_override", 0);
  angle_min_ = declare_parameter<double>("angle_min", ANGLE_MIN_LIMIT);
  angle_max_ = declare_parameter<double>("angle_max", ANGLE_MAX_LIMIT);
  angle_excluded_min_ = declare_parameter<double>("angle_excluded_min", DEFAULT_ANGLE_EXCLUDED_MIN);
  angle_excluded_max_ = declare_parameter<double>("angle_excluded_max", DEFAULT_ANGLE_EXCLUDED_MAX);
  range_min_ = declare_parameter<double>("range_min", RANGE_MIN_LIMIT);

  double range_max_default = RANGE_MAX_LIMIT_02A;
  if (device_model_ == "LT-R1")
    range_max_default = RANGE_MAX_LIMIT_R1;
  else if (device_model_ == "LT-R2")
    range_max_default = RANGE_MAX_LIMIT_R2;
  else if (device_model_ == "LT-I1")
    range_max_default = RANGE_MAX_LIMIT_I1;
  else if (device_model_ == "LT-I2")
    range_max_default = RANGE_MAX_LIMIT_I2;
  range_max_ = declare_parameter<double>("range_max", range_max_default);

  average_factor_ = declare_parameter<int>("average_factor", DEFAULT_AVERAGE_FACTOR);
  shadow_filter_strength_ = declare_parameter<int>("shadow_filter_strength", DEFAULT_SHADOW_FILTER_STRENGTH);
  receiver_sensitivity_boost_ = declare_parameter<int>("receiver_sensitivity_boost", DEFAULT_RECEIVER_SENSITIVITY_BOOST);

  if (device_model_.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required parameter \"device_model\"");
    exit(-1);
  }
  else if (device_model_ != "LTME-02A" &&
      device_model_ != "LT-R1" && device_model_ != "LT-R2" &&
      device_model_ != "LT-I1" && device_model_ != "LT-I2") {
    RCLCPP_ERROR(get_logger(), "Unsupported device model %s", device_model_.c_str());
    exit(-1);
  }
  if (device_address_.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required parameter \"device_address\"");
    exit(-1);
  }

  if (!(enforced_transport_mode_ == "none" || enforced_transport_mode_ == "normal" || enforced_transport_mode_ == "oob")) {
    RCLCPP_ERROR(get_logger(), "Transport mode \"%s\" not supported", enforced_transport_mode_.c_str());
    exit(-1);
  }
  if (scan_frequency_override_ != 0 &&
    (scan_frequency_override_ < 10 || scan_frequency_override_ > 30 || scan_frequency_override_ % 5 != 0)) {
    RCLCPP_ERROR(get_logger(), "Scan frequency %d not supported", scan_frequency_override_);
    exit(-1);
  }
  if (!(angle_min_ < angle_max_)) {
    RCLCPP_ERROR(get_logger(), "angle_min (%f) can't be larger than or equal to angle_max (%f)", angle_min_, angle_max_);
    exit(-1);
  }
  if (angle_min_ < ANGLE_MIN_LIMIT) {
    RCLCPP_ERROR(get_logger(), "angle_min is set to %f while its minimum allowed value is %f", angle_min_, ANGLE_MIN_LIMIT);
    exit(-1);
  }
  if (angle_max_ > ANGLE_MAX_LIMIT) {
    RCLCPP_ERROR(get_logger(), "angle_max is set to %f while its maximum allowed value is %f", angle_max_, ANGLE_MAX_LIMIT);
    exit(-1);
  }
  if (!(range_min_ < range_max_)) {
    RCLCPP_ERROR(get_logger(), "range_min (%f) can't be larger than or equal to range_max (%f)", range_min_, range_max_);
    exit(-1);
  }
  if (range_min_ < RANGE_MIN_LIMIT) {
    RCLCPP_ERROR(get_logger(), "range_min is set to %f while its minimum allowed value is %f", range_min_, RANGE_MIN_LIMIT);
    exit(-1);
  }
  if (average_factor_ <= 0 || average_factor_ > 8) {
    RCLCPP_ERROR(get_logger(), "average_factor is set to %d while its valid value is between 1 and 8", average_factor_);
    exit(-1);
  }
  if (shadow_filter_strength_ < 0 || average_factor_ > 100) {
    RCLCPP_ERROR(get_logger(), "shadow_filter_strength is set to %d while its valid value is between 0 and 100 (inclusive)", shadow_filter_strength_);
    exit(-1);
  }
  if (receiver_sensitivity_boost_ < -20 || receiver_sensitivity_boost_ > 10) {
    RCLCPP_ERROR(get_logger(), "receiver_sensitivity_boost is set to %d while the valid range is between -20 and 10 (inclusive)", receiver_sensitivity_boost_);
    exit(-1);
  }
}

void LidarDriver::run()
{
  std::unique_lock<std::mutex> lock(mutex_);

  // Best-effort sensor QoS — the ROS convention for LaserScan (matches
  // sllidar_ros2 and what rf2o / slam_toolbox / RViz subscribe with). A
  // reliable publisher here forces retransmission of the 30 Hz stream and
  // backs up over wifi, flooding RViz with tf message-filter drops.
  laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());
  query_model_service_ = create_service<Trigger>("query_model",
    std::bind(&LidarDriver::queryModelService, this, std::placeholders::_1, std::placeholders::_2));
  query_serial_service_ = create_service<Trigger>("query_serial",
    std::bind(&LidarDriver::querySerialService, this, std::placeholders::_1, std::placeholders::_2));
  query_firmware_service_ = create_service<Trigger>("query_firmware_version",
    std::bind(&LidarDriver::queryFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  query_hardware_service_ = create_service<Trigger>("query_hardware_version",
    std::bind(&LidarDriver::queryHardwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  request_hibernation_service_ = create_service<Empty>("request_hibernation",
    std::bind(&LidarDriver::requestHibernationService, this, std::placeholders::_1, std::placeholders::_2));
  request_wake_up_service_ = create_service<Empty>("request_wake_up",
    std::bind(&LidarDriver::requestWakeUpService, this, std::placeholders::_1, std::placeholders::_2));
  quit_driver_service_ = create_service<Empty>("quit_driver",
    std::bind(&LidarDriver::quitDriverService, this, std::placeholders::_1, std::placeholders::_2));

  std::string address_str = device_address_;
  std::string port_str = "2105";

  size_t position = device_address_.find(':');
  if (position != std::string::npos) {
    address_str = device_address_.substr(0, position);
    port_str = device_address_.substr(position + 1);
  }

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(), "Invalid device address: %s", device_address_.c_str());
    exit(-1);
  }

  ldcp_sdk::NetworkLocation location(address, port);
  device_ = std::unique_ptr<ldcp_sdk::Device>(new ldcp_sdk::Device(location));

  rclcpp::WallRate loop_rate(0.3);
  while (rclcpp::ok() && !quit_driver_.load()) {
    if (device_->open() == ldcp_sdk::no_error) {
      hibernation_requested_ = false;

      lock.unlock();

      RCLCPP_INFO(get_logger(), "Device opened");

      bool reboot_required = false;
      if (device_model_ == "LTME-02A" && enforced_transport_mode_ != "none") {
        std::string firmware_version;
        if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
          if (firmware_version < "0201")
            RCLCPP_WARN(get_logger(), "Firmware version %s supports normal transport mode only, "
              "\"enforced_transport_mode\" parameter will be ignored", firmware_version.c_str());
          else {
            bool oob_enabled = false;
            if (device_->isOobEnabled(oob_enabled) == ldcp_sdk::no_error) {
              if ((enforced_transport_mode_ == "normal" && oob_enabled) ||
                  (enforced_transport_mode_ == "oob" && !oob_enabled)) {
                RCLCPP_INFO(get_logger(), "Transport mode will be switched to \"%s\"", oob_enabled ? "normal" : "oob");
                device_->setOobEnabled(!oob_enabled);
                device_->persistSettings();
                reboot_required = true;
              }
            }
            else
              RCLCPP_WARN(get_logger(), "Unable to query device for its current transport mode, "
                "\"enforced_transport_mode\" parameter will be ignored");
          }
        }
        else
          RCLCPP_WARN(get_logger(), "Unable to query device for firmware version, \"enforced_transport_mode\" parameter will be ignored");
      }

      if (!reboot_required) {
        int scan_frequency = DEFAULT_SCAN_FREQUENCY;
        if (scan_frequency_override_ != 0)
          scan_frequency = scan_frequency_override_;
        else {
          if (device_->getScanFrequency(scan_frequency) != ldcp_sdk::no_error)
            RCLCPP_WARN(get_logger(), "Unable to query device for scan frequency and will use %d as the frequency value", scan_frequency);
        }

        if (shadow_filter_strength_ != DEFAULT_SHADOW_FILTER_STRENGTH) {
          if (device_->setShadowFilterStrength(shadow_filter_strength_) == ldcp_sdk::no_error)
            RCLCPP_INFO(get_logger(), "Shadow filter strength set to %d", shadow_filter_strength_);
          else
            RCLCPP_WARN(get_logger(), "Unable to set shadow filter strength");
        }

        if (receiver_sensitivity_boost_ != DEFAULT_RECEIVER_SENSITIVITY_BOOST) {
          if (device_->setReceiverSensitivityBoost(receiver_sensitivity_boost_) == ldcp_sdk::no_error) {
            RCLCPP_INFO(get_logger(), "Receiver sensitivity boost %d applied", receiver_sensitivity_boost_);
            int current_receiver_sensitivity = 0;
            if (device_->getReceiverSensitivityValue(current_receiver_sensitivity) == ldcp_sdk::no_error)
              RCLCPP_INFO(get_logger(), "Current receiver sensitivity: %d", current_receiver_sensitivity);
          }
        }

        device_->startMeasurement();
        device_->startStreaming();

        sensor_msgs::msg::LaserScan laser_scan;
        laser_scan.header.frame_id = frame_id_;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;

        auto readScanBlock = [&](ldcp_sdk::ScanBlock& scan_block) {
          if (device_->readScanBlock(scan_block) != ldcp_sdk::no_error)
            throw std::exception();
        };

        rclcpp::Time last_frame_end_timestamp;
        bool last_frame_end_valid = false;
        rclcpp::Time frame_start_timestamp, frame_end_timestamp;

        while (rclcpp::ok() && !quit_driver_.load()) {
          ldcp_sdk::ScanBlock scan_block;
          try {
            do {
              readScanBlock(scan_block);
              frame_start_timestamp = now();
            } while (scan_block.block_index != 0);

            double fov_angle_min = 0, fov_angle_max = 0;
            switch (scan_block.angular_fov) {
              case ldcp_sdk::ANGULAR_FOV_270DEG:
                fov_angle_min = -M_PI * 3 / 4;
                fov_angle_max = M_PI * 3 / 4;
                break;
              case ldcp_sdk::ANGULAR_FOV_360DEG:
                fov_angle_min = -M_PI;
                fov_angle_max = M_PI;
                break;
              default:
                RCLCPP_ERROR(get_logger(), "Unsupported FoV flag %d", scan_block.angular_fov);
                exit(-1);
            }
            angle_min_ = (angle_min_ > fov_angle_min) ? angle_min_ : fov_angle_min;
            angle_max_ = (angle_max_ < fov_angle_max) ? angle_max_ : fov_angle_max;

            int beam_count = scan_block.block_count * scan_block.block_length * 360 /
              ((scan_block.angular_fov == ldcp_sdk::ANGULAR_FOV_270DEG) ? 270 : 360);
            int beam_index_min = std::ceil(angle_min_ * beam_count / (2 * M_PI));
            int beam_index_max = std::floor(angle_max_ * beam_count / (2 * M_PI));
            int beam_index_excluded_min = std::ceil(angle_excluded_min_ * beam_count / (2 * M_PI));
            int beam_index_excluded_max = std::floor(angle_excluded_max_ * beam_count / (2 * M_PI));

            laser_scan.angle_min = (!invert_frame_) ? angle_min_ : angle_max_;
            laser_scan.angle_max = (!invert_frame_) ? angle_max_ : angle_min_;
            laser_scan.angle_increment = ((!invert_frame_) ? 1 : -1) *
                2 * M_PI / beam_count * average_factor_;

            laser_scan.ranges.resize(beam_index_max - beam_index_min + 1);
            laser_scan.intensities.resize(beam_index_max - beam_index_min + 1);

            std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), 0.0);
            std::fill(laser_scan.intensities.begin(), laser_scan.intensities.end(), 0.0);

            auto updateLaserScan = [&](const ldcp_sdk::ScanBlock& scan_block) {
              int block_size = scan_block.layers[0].ranges.size();
              for (int i = 0; i < block_size; i++) {
                int beam_index = (scan_block.block_index - scan_block.block_count / 2) * block_size + i;
                if (beam_index < beam_index_min || beam_index > beam_index_max)
                  continue;
                if (beam_index >= beam_index_excluded_min && beam_index <= beam_index_excluded_max)
                  continue;
                if (scan_block.layers[0].ranges[i] != 0) {
                  laser_scan.ranges[beam_index - beam_index_min] = scan_block.layers[0].ranges[i] * 0.002;
                  laser_scan.intensities[beam_index - beam_index_min] = scan_block.layers[0].intensities[i];
                }
                else {
                  // No return on this beam. Report 0.0 (< range_min → "invalid")
                  // rather than +inf. +inf is interpreted by slam_toolbox/Nav2
                  // as "free out to max range" and ray-traced as free space,
                  // so scattered no-return beams (specular/absorptive surfaces,
                  // ~12% here) carve radial free-space spokes straight through
                  // walls. 0.0 makes those beams simply be ignored.
                  laser_scan.ranges[beam_index - beam_index_min] = 0.0;
                  laser_scan.intensities[beam_index - beam_index_min] = 0;
                }
              }
            };

            while (scan_block.block_index != scan_block.block_count - 1) {
              updateLaserScan(scan_block);
              readScanBlock(scan_block);
              frame_end_timestamp = now();
            }
            updateLaserScan(scan_block);

            if (average_factor_ != 1) {
              int final_size = laser_scan.ranges.size() / average_factor_;
              for (int i = 0; i < final_size; i++) {
                double ranges_total = 0, intensities_total = 0;
                int count = 0;
                for (int j = 0; j < average_factor_; j++) {
                  int index = i * average_factor_ + j;
                  if (laser_scan.ranges[index] != 0) {
                    ranges_total += laser_scan.ranges[index];
                    intensities_total += laser_scan.intensities[index];
                    count++;
                  }
                }

                if (count > 0) {
                  laser_scan.ranges[i] = ranges_total / count;
                  laser_scan.intensities[i] = (int)(intensities_total / count);
                }
                else {
                  laser_scan.ranges[i] = 0;
                  laser_scan.intensities[i] = 0;
                }
              }

              laser_scan.ranges.resize(final_size);
              laser_scan.intensities.resize(final_size);
            }

            rclcpp::Duration block_duration = rclcpp::Duration::from_seconds(
              (frame_end_timestamp - frame_start_timestamp).seconds() /
              (scan_block.block_count - 1));
            frame_start_timestamp -= block_duration;

            if (last_frame_end_valid) {
              if (frame_start_timestamp < last_frame_end_timestamp)
                frame_start_timestamp = last_frame_end_timestamp;
              laser_scan.header.stamp = frame_start_timestamp;
              laser_scan.time_increment = (frame_end_timestamp - frame_start_timestamp).seconds() /
                (scan_block.block_count * scan_block.block_length) * average_factor_;
              laser_scan.scan_time = (frame_end_timestamp - frame_start_timestamp).seconds();
              laser_scan_publisher_->publish(laser_scan);
            }
            last_frame_end_timestamp = frame_end_timestamp;
            last_frame_end_valid = true;

            if (hibernation_requested_.load()) {
              device_->stopMeasurement();
              RCLCPP_INFO(get_logger(), "Device brought into hibernation");
              rclcpp::WallRate hibernation_rate(10);
              while (hibernation_requested_.load())
                hibernation_rate.sleep();
              device_->startMeasurement();
              RCLCPP_INFO(get_logger(), "Woken up from hibernation");
            }
          }
          catch (const std::exception&) {
            RCLCPP_WARN(get_logger(), "Error reading data from device");
            break;
          }
        }

        device_->stopStreaming();
      }
      else
        device_->reboot();

      lock.lock();
      device_->close();

      if (!reboot_required)
        RCLCPP_INFO(get_logger(), "Device closed");
      else
        RCLCPP_INFO(get_logger(), "Device rebooted");
    }
    else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for device... [%s]", device_address_.c_str());
      loop_rate.sleep();
    }
  }
}

void LidarDriver::queryModelService(const std::shared_ptr<Trigger::Request> /*request*/,
                                    std::shared_ptr<Trigger::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string model;
    if (device_->queryModel(model) == ldcp_sdk::no_error) {
      response->success = true;
      response->message = model;
      return;
    }
  }
  response->success = false;
}

void LidarDriver::querySerialService(const std::shared_ptr<Trigger::Request> /*request*/,
                                     std::shared_ptr<Trigger::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string serial;
    if (device_->querySerial(serial) == ldcp_sdk::no_error) {
      response->success = true;
      response->message = serial;
      return;
    }
  }
  response->success = false;
}

void LidarDriver::queryFirmwareVersion(const std::shared_ptr<Trigger::Request> /*request*/,
                                       std::shared_ptr<Trigger::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string firmware_version;
    if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
      response->success = true;
      response->message = firmware_version;
      return;
    }
  }
  response->success = false;
}

void LidarDriver::queryHardwareVersion(const std::shared_ptr<Trigger::Request> /*request*/,
                                       std::shared_ptr<Trigger::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string hardware_version;
    if (device_->queryHardwareVersion(hardware_version) == ldcp_sdk::no_error) {
      response->success = true;
      response->message = hardware_version;
      return;
    }
  }
  response->success = false;
}

void LidarDriver::requestHibernationService(const std::shared_ptr<Empty::Request> /*request*/,
                                            std::shared_ptr<Empty::Response> /*response*/)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock())
    hibernation_requested_ = true;
}

void LidarDriver::requestWakeUpService(const std::shared_ptr<Empty::Request> /*request*/,
                                       std::shared_ptr<Empty::Response> /*response*/)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock())
    hibernation_requested_ = false;
}

void LidarDriver::quitDriverService(const std::shared_ptr<Empty::Request> /*request*/,
                                    std::shared_ptr<Empty::Response> /*response*/)
{
  quit_driver_ = true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<LidarDriver>();
  RCLCPP_INFO(driver->get_logger(), "ltme_node started");

  // Spin services in the background while the blocking device read/publish
  // loop runs on this thread (mirrors the ROS 1 AsyncSpinner(1) design).
  std::thread spin_thread([driver]() {
    rclcpp::spin(driver);
  });

  driver->run();

  rclcpp::shutdown();
  if (spin_thread.joinable())
    spin_thread.join();

  return 0;
}
