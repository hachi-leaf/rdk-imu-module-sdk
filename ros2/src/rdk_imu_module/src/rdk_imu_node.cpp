/*
 * Copyright (c) 2026 Leaf. D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rcl/time.h"

extern "C" {
#include "rdkimu.h"
}

// ===================== 枚举映射函数 =====================
static rdk_imu_accel_bwp_t accel_bwp_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_accel_bwp_t> m = {
    {"OSR4", RDK_IMU_OSR4}, {"OSR2", RDK_IMU_OSR2}, {"NORMAL", RDK_IMU_NORMAL}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_NORMAL;
}

static rdk_imu_accel_odr_t accel_odr_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_accel_odr_t> m = {
    {"12_5", RDK_IMU_ACCEL_12_5}, {"25", RDK_IMU_ACCEL_25}, {"50", RDK_IMU_ACCEL_50},
    {"100", RDK_IMU_ACCEL_100},   {"200", RDK_IMU_ACCEL_200}, {"400", RDK_IMU_ACCEL_400},
    {"800", RDK_IMU_ACCEL_800},   {"1600", RDK_IMU_ACCEL_1600}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_ACCEL_400;
}

static rdk_imu_accel_range_t accel_range_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_accel_range_t> m = {
    {"3G", RDK_IMU_ACCEL_3G}, {"6G", RDK_IMU_ACCEL_6G},
    {"12G", RDK_IMU_ACCEL_12G}, {"24G", RDK_IMU_ACCEL_24G}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_ACCEL_24G;
}

static rdk_imu_gyro_range_t gyro_range_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_gyro_range_t> m = {
    {"2000DPS", RDK_IMU_GYRO_2000DPS}, {"1000DPS", RDK_IMU_GYRO_1000DPS},
    {"500DPS", RDK_IMU_GYRO_500DPS},   {"250DPS", RDK_IMU_GYRO_250DPS},
    {"125DPS", RDK_IMU_GYRO_125DPS}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_GYRO_2000DPS;
}

static rdk_imu_gyro_bandwidth_t gyro_bandwidth_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_gyro_bandwidth_t> m = {
    {"ODR2000_BW532", RDK_IMU_ODR2000_BW532}, {"ODR2000_BW230", RDK_IMU_ODR2000_BW230},
    {"ODR1000_BW116", RDK_IMU_ODR1000_BW116}, {"ODR400_BW47",   RDK_IMU_ODR400_BW47},
    {"ODR200_BW23",   RDK_IMU_ODR200_BW23},   {"ODR100_BW12",   RDK_IMU_ODR100_BW12},
    {"ODR200_BW64",   RDK_IMU_ODR200_BW64},   {"ODR100_BW32",   RDK_IMU_ODR100_BW32}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_ODR400_BW47;
}

static rdk_imu_accel_drdy_int_t accel_drdy_int_from_str(const std::string &s) {
  if (s == "INT2") return RDK_IMU_INT2;
  return RDK_IMU_INT1;
}

static rdk_imu_gyro_drdy_int_t gyro_drdy_int_from_str(const std::string &s) {
  if (s == "INT4") return RDK_IMU_INT4;
  return RDK_IMU_INT3;
}

static rdk_imu_gpio_mode_t gpio_mode_from_str(const std::string &s) {
  static const std::unordered_map<std::string, rdk_imu_gpio_mode_t> m = {
    {"PP_H", RDK_IMU_PP_H}, {"PP_L", RDK_IMU_PP_L},
    {"OD_H", RDK_IMU_OD_H}, {"OD_L", RDK_IMU_OD_L}
  };
  auto it = m.find(s);
  return (it != m.end()) ? it->second : RDK_IMU_PP_H;
}

static rdk_imu_fifo_mode_t fifo_mode_from_str(const std::string &s) {
  if (s == "DROP") return RDK_IMU_FIFO_DROP;
  return RDK_IMU_FIFO_OVERWRITE;
}

// 带宽映射 (用于协方差计算)
static double accel_bwp_to_hz(rdk_imu_accel_bwp_t bwp) {
  switch (bwp) {
    case RDK_IMU_OSR4:   return 200.0;
    case RDK_IMU_OSR2:   return 400.0;
    case RDK_IMU_NORMAL: return 800.0;
    default:             return 200.0;
  }
}

static double gyro_bw_to_hz(rdk_imu_gyro_bandwidth_t bw) {
  switch (bw) {
    case RDK_IMU_ODR2000_BW532: return 532.0;
    case RDK_IMU_ODR2000_BW230: return 230.0;
    case RDK_IMU_ODR1000_BW116: return 116.0;
    case RDK_IMU_ODR400_BW47:   return 47.0;
    case RDK_IMU_ODR200_BW23:   return 23.0;
    case RDK_IMU_ODR100_BW12:   return 12.0;
    case RDK_IMU_ODR200_BW64:   return 64.0;
    case RDK_IMU_ODR100_BW32:   return 32.0;
    default:                    return 47.0;
  }
}

// 噪声密度 (BMI088 手册值)
static constexpr double ACCEL_NOISE_DENSITY_XY = 160.0e-6 * 9.80665;   // m/s²/√Hz
static constexpr double ACCEL_NOISE_DENSITY_Z  = 190.0e-6 * 9.80665;
static constexpr double GYRO_NOISE_DENSITY     = 0.1 * (M_PI / 180.0); // rad/s/√Hz

// ===================== ROS 2 节点类 =====================
class RdkImuNode : public rclcpp::Node {
public:
  RdkImuNode() : Node("rdk_imu_node") {
    // ---------- 基础参数 ----------
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("fuse_by", "accel");
    this->declare_parameter("max_age_ns", 50000000);
    this->declare_parameter("publish_rate", 0.0);
    this->declare_parameter("auto_start", true);

    // ---------- 话题和服务名参数 ----------
    this->declare_parameter("imu_topic", "rdkimu/data");
    this->declare_parameter("enable_service_name", "~/enable");
    this->declare_parameter("disable_service_name", "~/disable");

    // ---------- IMU 配置参数 ----------
    this->declare_parameter("accel_bwp", "NORMAL");
    this->declare_parameter("accel_odr", "400");
    this->declare_parameter("accel_range", "24G");
    this->declare_parameter("gyro_range", "2000DPS");
    this->declare_parameter("gyro_bandwidth", "ODR400_BW47");
    this->declare_parameter("accel_drdy_int", "INT1");
    this->declare_parameter("accel_int_gpio_mode", "PP_H");
    this->declare_parameter("gyro_drdy_int", "INT3");
    this->declare_parameter("gyro_int_gpio_mode", "PP_H");
    this->declare_parameter("accel_drdy_gpio_chip", 4);
    this->declare_parameter("accel_drdy_gpio_line", 2);
    this->declare_parameter("gyro_drdy_gpio_chip", 3);
    this->declare_parameter("gyro_drdy_gpio_line", 12);
    this->declare_parameter("fifo_length", 256);
    this->declare_parameter("fifo_mode", "OVERWRITE");
    this->declare_parameter("irq_priority", -1);
    this->declare_parameter("irq_thread_timeout_ns", 1000000000);

    // 读取基本参数
    frame_id_     = this->get_parameter("frame_id").as_string();
    auto fuse_str = this->get_parameter("fuse_by").as_string();
    max_age_ns_   = static_cast<uint64_t>(this->get_parameter("max_age_ns").as_int());
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    bool auto_start = this->get_parameter("auto_start").as_bool();
    fuse_by_ = (fuse_str == "gyro") ? RDK_IMU_GYRO : RDK_IMU_ACCEL;

    // 读取话题与服务名
    std::string imu_topic        = this->get_parameter("imu_topic").as_string();
    std::string enable_srv_name  = this->get_parameter("enable_service_name").as_string();
    std::string disable_srv_name = this->get_parameter("disable_service_name").as_string();

    // 构建配置
    build_config_from_params(cfg_);

    // 硬件初始化
    if (!init_hardware()) {
      RCLCPP_FATAL(this->get_logger(), "Hardware init failed");
      rclcpp::shutdown();
      return;
    }

    // 计算协方差
    compute_covariance();

    // 自动使能或等待手动使能
    if (auto_start) {
      if (rdk_imu_enable(st_) != RDK_IMU_OK) {
        RCLCPP_FATAL(this->get_logger(), "Auto‑enable failed");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "IMU streaming");
    } else {
      RCLCPP_INFO(this->get_logger(), "IMU ready, call %s to start", enable_srv_name.c_str());
    }

    // 创建发布者
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

    // 创建服务
    enable_srv_ = this->create_service<std_srvs::srv::Trigger>(
        enable_srv_name,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          auto ret = rdk_imu_enable(st_);
          res->success = (ret == RDK_IMU_OK);
          res->message = res->success ? "enabled" : "enable failed";
        });

    disable_srv_ = this->create_service<std_srvs::srv::Trigger>(
        disable_srv_name,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          auto ret = rdk_imu_disable(st_);
          res->success = (ret == RDK_IMU_OK);
          res->message = res->success ? "disabled" : "disable failed";
        });

    running_ = true;
    read_thread_ = std::thread(&RdkImuNode::read_loop, this);
  }

  ~RdkImuNode() override {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    if (st_) {
      rdk_imu_disable(st_);
      rdk_imu_device_deinit(st_);
      rdk_imu_bus_deinit(st_);
      rdk_imu_destroy(st_);
    }
  }

private:
  void build_config_from_params(rdk_imu_config_t &cfg) {
    cfg.accel_bwp          = accel_bwp_from_str(this->get_parameter("accel_bwp").as_string());
    cfg.accel_odr          = accel_odr_from_str(this->get_parameter("accel_odr").as_string());
    cfg.accel_range        = accel_range_from_str(this->get_parameter("accel_range").as_string());
    cfg.gyro_range         = gyro_range_from_str(this->get_parameter("gyro_range").as_string());
    cfg.gyro_bandwidth     = gyro_bandwidth_from_str(this->get_parameter("gyro_bandwidth").as_string());
    cfg.accel_drdy_int     = accel_drdy_int_from_str(this->get_parameter("accel_drdy_int").as_string());
    cfg.accel_int_gpio_mode = gpio_mode_from_str(this->get_parameter("accel_int_gpio_mode").as_string());
    cfg.gyro_drdy_int      = gyro_drdy_int_from_str(this->get_parameter("gyro_drdy_int").as_string());
    cfg.gyro_int_gpio_mode  = gpio_mode_from_str(this->get_parameter("gyro_int_gpio_mode").as_string());
    cfg.accel_drdy_gpio_chip = static_cast<uint32_t>(this->get_parameter("accel_drdy_gpio_chip").as_int());
    cfg.accel_drdy_gpio_line = static_cast<uint32_t>(this->get_parameter("accel_drdy_gpio_line").as_int());
    cfg.gyro_drdy_gpio_chip  = static_cast<uint32_t>(this->get_parameter("gyro_drdy_gpio_chip").as_int());
    cfg.gyro_drdy_gpio_line  = static_cast<uint32_t>(this->get_parameter("gyro_drdy_gpio_line").as_int());
    cfg.fifo_length        = static_cast<uint32_t>(this->get_parameter("fifo_length").as_int());
    cfg.fifo_mode          = fifo_mode_from_str(this->get_parameter("fifo_mode").as_string());
    cfg.irq_priority       = static_cast<int32_t>(this->get_parameter("irq_priority").as_int());
    cfg.irq_thread_timeout_ns = static_cast<uint64_t>(this->get_parameter("irq_thread_timeout_ns").as_int());
  }

  bool init_hardware() {
    st_ = rdk_imu_create_default();
    if (!st_) {
      RCLCPP_ERROR(this->get_logger(), "rdk_imu_create_default failed");
      return false;
    }

    rdk_imu_bus_info_t bus;
    bus.interface = RDK_IMU_AUTO;
    bus.bus.spi.accel.speed_hz = 1000000;
    bus.bus.spi.gyro.speed_hz  = 1000000;

    if (rdk_imu_bus_init(st_, bus) != RDK_IMU_OK) {
      RCLCPP_ERROR(this->get_logger(), "bus init failed");
      return false;
    }

    if (rdk_imu_device_init(st_, cfg_) != RDK_IMU_OK) {
      RCLCPP_ERROR(this->get_logger(), "device init failed");
      return false;
    }
    return true;
  }

  void compute_covariance() {
    double acc_bw = accel_bwp_to_hz(cfg_.accel_bwp);
    double gyr_bw = gyro_bw_to_hz(cfg_.gyro_bandwidth);
    acc_cov_xy_ = std::pow(ACCEL_NOISE_DENSITY_XY, 2) * acc_bw;
    acc_cov_z_  = std::pow(ACCEL_NOISE_DENSITY_Z,  2) * acc_bw;
    gyr_cov_    = std::pow(GYRO_NOISE_DENSITY,     2) * gyr_bw;
  }

  void read_loop() {
    rclcpp::WallRate rate(publish_rate_ > 0 ? publish_rate_ : 1000);

    while (running_ && rclcpp::ok()) {
      rdk_imu_6_axis_data_t data;
      auto ret = rdk_imu_read_fused(st_, &data, fuse_by_, max_age_ns_);

      if (ret == RDK_IMU_OK) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = rclcpp::Time(data.accel.timestamp_ns, RCL_ROS_TIME);
        msg.header.frame_id = frame_id_;

        msg.linear_acceleration.x = data.accel.x;
        msg.linear_acceleration.y = data.accel.y;
        msg.linear_acceleration.z = data.accel.z;
        msg.angular_velocity.x = data.gyro.x;
        msg.angular_velocity.y = data.gyro.y;
        msg.angular_velocity.z = data.gyro.z;

        // 协方差
        msg.linear_acceleration_covariance[0] = acc_cov_xy_;
        msg.linear_acceleration_covariance[4] = acc_cov_xy_;
        msg.linear_acceleration_covariance[8] = acc_cov_z_;
        msg.angular_velocity_covariance[0] = gyr_cov_;
        msg.angular_velocity_covariance[4] = gyr_cov_;
        msg.angular_velocity_covariance[8] = gyr_cov_;
        for (int i = 0; i < 9; ++i)
          msg.orientation_covariance[i] = -1.0;

        imu_pub_->publish(msg);
      } else if (ret != RDK_IMU_FIFO_EMPTY) {
        RCLCPP_WARN(this->get_logger(), "read_fused error: %d", ret);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (publish_rate_ > 0.0) rate.sleep();
    }
  }

  rdk_imu_state_t *st_{nullptr};
  rdk_imu_device_t fuse_by_{RDK_IMU_ACCEL};
  rdk_imu_config_t cfg_;
  std::string frame_id_;
  uint64_t max_age_ns_{50000000};
  double publish_rate_{0.0};

  double acc_cov_xy_{0.0}, acc_cov_z_{0.0}, gyr_cov_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;

  std::atomic<bool> running_{false};
  std::thread read_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RdkImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}