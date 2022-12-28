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
#include "AgricInfoInterface.h"

#include "rclcpp/rclcpp.hpp"

#include "nmea_msgs/msg/gpgga.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <rbf_clap_b7_msgs/msg/clap_data.hpp>
#include <ntrip/ntrip_client.h>
#include "ClapB7BinaryParser.h"

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


    void ParseDataASCII(const char* serial_data);

    void pub_ClapB7Data();


    std::string clap_data_topic_;
    std::string serial_name_;
    std::string parse_type_;

    long baud_rate_;

    CallbackAsyncSerial serial_boost;

    std::vector<std::string> seperated_data_;
    std::string header_;

    std::string ntrip_server_ip_;
    std::string username_;
    std::string password_;
    std::string mount_point_;
    int ntrip_port_;

    rclcpp::Publisher<rbf_clap_b7_msgs::msg::ClapData>::SharedPtr pub_clap_data_;

    rclcpp::TimerBase::SharedPtr timer_;
    AgricMsg_* AgricMsg_p{};

    ClapB7Controller clapB7Controller;

    double accel_scale_factor = 400/(pow(2, 31));
    double gyro_scale_factor = 2160/(pow(2, 31));
    int freq = 0;

    libntrip::NtripClient ntripClient;
    int NTRIP_client_start();
    int t_size;
};
