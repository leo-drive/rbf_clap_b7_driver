//
// Created by arslan on 22.05.2022.
//
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "TermiosSerial.h"
#include "AsyncSerial.h"

#include "rclcpp/rclcpp.hpp"

#include "nmea_msgs/msg/gpgga.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <rbf_clap_b7_msgs/msg/clap_data.hpp>
#include <ntrip/ntrip_client.h>
#include "ClapB7BinaryParser.h"

#define ACCEL_SCALE_FACTOR (400 / (pow(2, 31)))
#define GYRO_SCALE_FACTOR (2160 / (pow(2, 31)))

extern int freq;

class ClapB7Driver : public rclcpp::Node
{
public:
    ClapB7Driver();

    ~ClapB7Driver() override {
        serial_boost.close();
    }

    void serial_receive_callback(const char *data, unsigned int len);

private:
    
    void timer_callback();
    void ParseDataASCII(std::string serial_data);
    void pub_ClapB7Data();
    int NTRIP_client_start();
    void publish_standart_msgs();
    void parse_gpgga(const char* data);
    void ascii_data_collector(const char* serial_data, int len);

    std::string clap_data_topic_;
    std::string imu_topic_;
    std::string nav_sat_fix_topic_;
    std::string serial_name_;
    std::string parse_type_;
    std::string ntrip_server_ip_;
    std::string username_;
    std::string password_;
    std::string mount_point_;

    int ntrip_port_;
    long baud_rate_;

    CallbackAsyncSerial serial_boost;

    std::vector<std::string> seperated_data_;
    std::string header_;
   
    rclcpp::Publisher<rbf_clap_b7_msgs::msg::ClapData>::SharedPtr pub_clap_data_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;

    ClapB7Controller clapB7Controller;
    uint8_t ntrip_status_ = 0;
    libntrip::NtripClient ntripClient;
    int t_size;
    const float g_ = 9.81f;

    rclcpp::TimerBase::SharedPtr timer_;
};
