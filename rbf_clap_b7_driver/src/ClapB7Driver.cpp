//
// Created by arslan on 22.05.2022.
//
#include "ClapB7Driver.hpp"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

using namespace std;
using namespace chrono_literals;
using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;

ClapB7Driver::ClapB7Driver()
    : Node("rbf_clap_b7_driver"),

      clap_data_topic_{this->declare_parameter(
                      "clap_imu_data",
                      ParameterValue{"/clap_b7_data"},
                      ParameterDescriptor{})
                              .get<std::string>()},

      serial_name_{this->declare_parameter(
                      "serial_name",
                      ParameterValue{"/dev/ttyUSB0"},
                      ParameterDescriptor{})
                              .get<std::string>()},

      parse_type_{this->declare_parameter(
                      "parse_type",
                      ParameterValue{"BINARY"},
                      ParameterDescriptor{})
                              .get<std::string>()},

      baud_rate_{this->declare_parameter(
              "baud_rate",
              ParameterValue{460800},
              ParameterDescriptor{})
                              .get<long>()},

      serial_boost(serial_name_, baud_rate_),

      ntrip_server_ip_{this->declare_parameter("ntrip_ip",
                                               ParameterValue("212.156.70.42"),
                                               ParameterDescriptor{})
                               .get<std::string>()},

      username_{this->declare_parameter("ntrip_username",
                                        ParameterValue("K073432501"),
                                        ParameterDescriptor{})
                        .get<std::string>()},

      password_{this->declare_parameter("ntrip_password",
                                        ParameterValue("GR3g4"),
                                        ParameterDescriptor{})
                        .get<std::string>()},

      mount_point_{this->declare_parameter("ntrip_mount_point",
                                           ParameterValue("VRSRTCM31"),
                                           ParameterDescriptor{})
                           .get<std::string>()},

      ntrip_port_{static_cast<int>(this->declare_parameter("ntrip_port",
                                                           ParameterValue(2101),
                                                           ParameterDescriptor{})
                                                           .get<int>())},

      pub_clap_data_{create_publisher<rbf_clap_b7_msgs::msg::ClapData>(
              clap_data_topic_, rclcpp::QoS{10}, PubAllocT{})},

      timer_{this->create_wall_timer(
            1000ms, std::bind(&ClapB7Driver::timer_callback, this))}

{
    using namespace std::placeholders;

    serial_boost.setCallback(bind(&ClapB7Driver::serial_receive_callback, this, _1, _2));

    if(parse_type_ == "BINARY") {
        ClapB7Init(&clapB7Controller, bind(&ClapB7Driver::pub_ClapB7Data, this));
    }
    else if(parse_type_ != "ASCII"){
        RCLCPP_ERROR(this->get_logger(), "PARSE TYPE DOESN'T MATCH ANY FORMAT, USE 'ASCII' OR 'BINARY' INSTEAD");
        exit(-1);
    }
    NTRIP_client_start();
}

template <typename Type>
Type stringToNum(const string &str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <typename Type>
string numToString (const Type &num)
{
    stringstream ss;
    string s;

    ss << num;
    s = ss.str();
    return s;
}

void ClapB7Driver::serial_receive_callback(const char *data, unsigned int len)
{
//    RCLCPP_INFO_STREAM(this->get_logger(), "Boost Data Size: '" << numToString(len) << "'");

    if (parse_type_ == "ASCII") {
        ParseDataASCII(data);
    }
    else{
        ClapB7Parser(&clapB7Controller, reinterpret_cast<const uint8_t *>(data), len);
    }
}

void ClapB7Driver::timer_callback() {
}

void ClapB7Driver::ParseDataASCII(const char* serial_data) {
    string raw_serial_data = numToString(serial_data);

    std::string delimiter = ",";

    size_t pos = 0;
    std::string token;

    int header_pos = raw_serial_data.find(';');
    header_ = raw_serial_data.substr(0, header_pos);
    header_ = header_.substr(0, header_.find(','));
    raw_serial_data.erase(0, header_pos + delimiter.length());

    while ((pos = raw_serial_data.find(delimiter)) != std::string::npos) {
        token = raw_serial_data.substr(0, pos);
        seperated_data_.push_back(token);
        raw_serial_data.erase(0, pos + delimiter.length());
    }
    seperated_data_.push_back(raw_serial_data);

    if (header_ == "#INTERESULTA") {
        int i = 0;
        clapB7Controller.clapData.ins_status= stringToNum<uint32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.pos_type= stringToNum<uint32_t>(seperated_data_.at(i++));

        clapB7Controller.clapData.latitude= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.longitude= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.height= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.undulation= stringToNum<float>(seperated_data_.at(i++));

        clapB7Controller.clapData.north_velocity= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.east_velocity= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.up_velocity= stringToNum<double>(seperated_data_.at(i++));

        clapB7Controller.clapData.roll= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.pitch= stringToNum<double>(seperated_data_.at(i++));
        clapB7Controller.clapData.azimuth= stringToNum<double>(seperated_data_.at(i++));

        clapB7Controller.clapData.std_dev_latitude= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_longitude= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_height= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_north_velocity= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_east_velocity= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_up_velocity= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_roll= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_pitch= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.std_dev_azimuth= stringToNum<float>(seperated_data_.at(i++));

        clapB7Controller.clapData.extended_solution_stat= stringToNum<uint32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.time_since_update= stringToNum<uint16_t>(seperated_data_.at(i++));

        clapB7Controller.clapData.imu_error= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.imu_type= atoi((seperated_data_.at(i++).c_str()));

        clapB7Controller.clapData.z_accel_output= stringToNum<int32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.y_accel_output= stringToNum<int32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.x_accel_output= stringToNum<int32_t>(seperated_data_.at(i++));

        clapB7Controller.clapData.z_gyro_output= stringToNum<int32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.y_gyro_output= stringToNum<int32_t>(seperated_data_.at(i++));
        clapB7Controller.clapData.x_gyro_output= stringToNum<int32_t>(seperated_data_.at(i++));

        clapB7Controller.clapData.gps_sat_num= atoi((seperated_data_.at(i++)).c_str());
        clapB7Controller.clapData.bd_sat_num= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.glo_sat_num= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.gal_sat_num= atoi(seperated_data_.at(i++).c_str());

        clapB7Controller.clapData.rtk_delay= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.gdop= stringToNum<float>(seperated_data_.at(i++));

        clapB7Controller.clapData.remain_float_1= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.remain_float_2= stringToNum<float>(seperated_data_.at(i++));
        clapB7Controller.clapData.remain_double= stringToNum<double>(seperated_data_.at(i++));

        clapB7Controller.clapData.remain_char_1= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.remain_char_2= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.remain_char_3= atoi((seperated_data_.at(i++).c_str()));
        clapB7Controller.clapData.remain_char_4= atoi((seperated_data_.at(i++).c_str()));

        pub_ClapB7Data();
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Received data doesn't match with any header");
    }
    seperated_data_.clear();
}

void ClapB7Driver::pub_ClapB7Data() {

    rbf_clap_b7_msgs::msg::ClapData msg_clap_data;
    msg_clap_data.set__ins_status(static_cast<uint32_t>(clapB7Controller.clapData.ins_status));
    msg_clap_data.set__pos_type(static_cast<uint32_t>(clapB7Controller.clapData.pos_type));

    msg_clap_data.set__latitude(static_cast<double>(clapB7Controller.clapData.latitude));
    msg_clap_data.set__longitude(static_cast<double>(clapB7Controller.clapData.longitude));

    msg_clap_data.set__height(static_cast<double>(clapB7Controller.clapData.height));

    msg_clap_data.set__undulation(static_cast<float>(clapB7Controller.clapData.undulation));

    msg_clap_data.set__north_velocity(static_cast<double>(clapB7Controller.clapData.north_velocity));
    msg_clap_data.set__east_velocity(static_cast<double>(clapB7Controller.clapData.east_velocity));
    msg_clap_data.set__up_velocity(static_cast<double>(clapB7Controller.clapData.up_velocity));

    msg_clap_data.set__roll(static_cast<double>(clapB7Controller.clapData.roll));
    msg_clap_data.set__pitch(static_cast<double>(clapB7Controller.clapData.pitch));
    msg_clap_data.set__azimuth(static_cast<double>(clapB7Controller.clapData.azimuth));

    msg_clap_data.set__std_dev_latitude(static_cast<float>(clapB7Controller.clapData.std_dev_latitude));
    msg_clap_data.set__std_dev_longitude(static_cast<float>(clapB7Controller.clapData.std_dev_longitude));

    msg_clap_data.set__std_dev_height(static_cast<float>(clapB7Controller.clapData.std_dev_height));

    msg_clap_data.set__std_dev_north_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_north_velocity));
    msg_clap_data.set__std_dev_east_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_east_velocity));
    msg_clap_data.set__std_dev_up_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_up_velocity));

    msg_clap_data.set__std_dev_roll(static_cast<float>(clapB7Controller.clapData.std_dev_roll));
    msg_clap_data.set__std_dev_pitch(static_cast<float>(clapB7Controller.clapData.std_dev_pitch));
    msg_clap_data.set__std_dev_azimuth(static_cast<float>(clapB7Controller.clapData.std_dev_azimuth));

    msg_clap_data.set__extended_solution_stat(static_cast<uint32_t>(clapB7Controller.clapData.extended_solution_stat));
    msg_clap_data.set__time_since_update(static_cast<uint16_t>(clapB7Controller.clapData.time_since_update));

    msg_clap_data.set__imu_error(static_cast<uint8_t>(clapB7Controller.clapData.imu_error));
    msg_clap_data.set__imu_type(static_cast<uint8_t>(clapB7Controller.clapData.imu_type));

    msg_clap_data.set__z_accel_output(static_cast<int32_t>(clapB7Controller.clapData.z_accel_output * accel_scale_factor));
    msg_clap_data.set__y_accel_output(static_cast<int32_t>(clapB7Controller.clapData.y_accel_output * (- accel_scale_factor)));
    msg_clap_data.set__x_accel_output(static_cast<int32_t>(clapB7Controller.clapData.x_accel_output * accel_scale_factor));

    msg_clap_data.set__z_gyro_output(static_cast<int32_t>(clapB7Controller.clapData.x_gyro_output * gyro_scale_factor));
    msg_clap_data.set__y_gyro_output(static_cast<int32_t>(clapB7Controller.clapData.y_gyro_output * ( - gyro_scale_factor)));
    msg_clap_data.set__x_gyro_output(static_cast<int32_t>(clapB7Controller.clapData.z_gyro_output * gyro_scale_factor));

    msg_clap_data.set__gps_sat_num(static_cast<uint8_t>(clapB7Controller.clapData.gps_sat_num));
    msg_clap_data.set__bd_sat_num(static_cast<uint8_t>(clapB7Controller.clapData.bd_sat_num));
    msg_clap_data.set__glo_sat_num(static_cast<uint8_t>(clapB7Controller.clapData.glo_sat_num));
    msg_clap_data.set__gal_sat_num(static_cast<uint8_t>(clapB7Controller.clapData.gal_sat_num));

    msg_clap_data.set__rtk_delay(static_cast<float>(clapB7Controller.clapData.rtk_delay));
    msg_clap_data.set__gdop(static_cast<float>(clapB7Controller.clapData.gdop));

    msg_clap_data.set__remain_float_1(static_cast<float>(clapB7Controller.clapData.remain_float_1));
    msg_clap_data.set__remain_float_2(static_cast<float>(clapB7Controller.clapData.remain_float_2));

    msg_clap_data.set__remain_double(static_cast<double>(clapB7Controller.clapData.remain_double));

    msg_clap_data.set__remain_char_1(static_cast<uint8_t>(clapB7Controller.clapData.remain_char_1));
    msg_clap_data.set__remain_char_2(static_cast<uint8_t>(clapB7Controller.clapData.remain_char_2));
    msg_clap_data.set__remain_char_3(static_cast<uint8_t>(clapB7Controller.clapData.remain_char_3));
    msg_clap_data.set__remain_char_4(static_cast<uint8_t>(clapB7Controller.clapData.remain_char_4));

    pub_clap_data_->publish(msg_clap_data);
}
int ClapB7Driver::NTRIP_client_start()
{
  ntripClient.Init(ntrip_server_ip_, ntrip_port_, username_, password_, mount_point_);
  ntripClient.OnReceived([this](const char *buffer, int size)
                         {

                             serial_boost.write(buffer, size);

                             t_size += size;

                             std::cout << "NTRIP:" << t_size << std::endl; });

  ntripClient.set_location(41.018893949, 28.890924848);

  ntripClient.set_report_interval(0.001);
  ntripClient.Run();

  return 0;
}