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
    : Node("rbf_um482_driver"),

      gnss_status_topic_{this->declare_parameter(
                        "gnss_status_topic",
                        ParameterValue{"/gnss_status"},
                        ParameterDescriptor{})
                                    .get<std::string>()},

      nav_data_topic_{this->declare_parameter(
                      "nav_data_topic",
                      ParameterValue{"/nav_data"},
                      ParameterDescriptor{})
                                 .get<std::string>()},

      std_deviation_data_topic_{this->declare_parameter(
                                "std_deviation_data_topic",
                                ParameterValue{"/std_deviation_data"},
                                ParameterDescriptor{})
                                 .get<std::string>()},

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
              ParameterValue{230400},
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

      pub_gnss_status_{create_publisher<rbf_clap_b7_msgs::msg::GNSSStatus>(
              gnss_status_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_nav_data_{create_publisher<rbf_clap_b7_msgs::msg::NavigationData>(
              nav_data_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_std_deviation_data_{create_publisher<rbf_clap_b7_msgs::msg::StdDeviationData>(
              std_deviation_data_topic_, rclcpp::QoS{10}, PubAllocT{})},

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

#define PI 3.141592653589793
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
    std::cout << "frekans = " << clapB7Controller.freq << "\n";
    clapB7Controller.freq = 0;
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

    if((header_ == "#AGRICA") && (seperated_data_.size() >= 65) )
    {
        int i = 1;
        AgricMsg_p->command_length = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->year = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->month = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->day = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->hour = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->minute = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->second = atoi(seperated_data_.at(i++).c_str());

        AgricMsg_p->RtkStatus = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->HeadingStatus = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->numGPSsat = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->numBDSsat = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->numGLOsat = atoi(seperated_data_.at(i++).c_str());

        AgricMsg_p->Baseline_N = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Baseline_E = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Baseline_U = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Baseline_NStd = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Baseline_EStd = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Baseline_UStd = stringToNum<float>(seperated_data_.at(i++));

        AgricMsg_p->Heading = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Pitch = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Roll = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Speed = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Velocity_N = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Velocity_E = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Velocity_U = stringToNum<float>(seperated_data_.at(i++));

        AgricMsg_p->Xigema_Vx = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_Vy = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_Vz = stringToNum<float>(seperated_data_.at(i++));

        AgricMsg_p->lat = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->lon = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->Het = stringToNum<double>(seperated_data_.at(i++));

        AgricMsg_p->ecef_x = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->ecef_y = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->ecef_z = stringToNum<double>(seperated_data_.at(i++));

        AgricMsg_p->Xigema_lat = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_lon = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_alt = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_ecef_x = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_ecef_y = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->Xigema_ecef_z = stringToNum<float>(seperated_data_.at(i++));

        AgricMsg_p->base_lat = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->base_lon = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->base_alt = stringToNum<double>(seperated_data_.at(i++));

        AgricMsg_p->sec_lat = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->sec_lon = stringToNum<double>(seperated_data_.at(i++));
        AgricMsg_p->sec_alt = stringToNum<double>(seperated_data_.at(i++));

        AgricMsg_p->gps_week_second = stringToNum<int>(seperated_data_.at(i++));

        AgricMsg_p->diffage = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->speed_heading = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->undulation = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->remain_float3 = stringToNum<float>(seperated_data_.at(i++));
        AgricMsg_p->remain_float4 = stringToNum<float>(seperated_data_.at(i++));

        AgricMsg_p->numGALsat = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->remain_char2 = atoi(seperated_data_.at(i++).c_str());
        AgricMsg_p->remain_char3 = atoi(seperated_data_.at(i++).c_str());

        pub_GnssData();

    }
    else if (header_ == "#INTERESULTA") {
        int i = 0;
        freq++;
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

void ClapB7Driver::pub_GnssData() {
    rbf_clap_b7_msgs::msg::GNSSStatus msg_gnssStatus;
    rbf_clap_b7_msgs::msg::NavigationData msg_navData;
    rbf_clap_b7_msgs::msg::StdDeviationData msg_StdDevData;

    msg_gnssStatus.set__command_length(static_cast<uint8_t>(AgricMsg_p->command_length));
    msg_gnssStatus.set__year(static_cast<uint8_t>(AgricMsg_p->year));
    msg_gnssStatus.set__month(static_cast<uint8_t>(AgricMsg_p->month));
    msg_gnssStatus.set__day(static_cast<uint8_t>(AgricMsg_p->day));
    msg_gnssStatus.set__hour(static_cast<uint8_t>(AgricMsg_p->hour));
    msg_gnssStatus.set__minute(static_cast<uint8_t>(AgricMsg_p->minute));
    msg_gnssStatus.set__second(static_cast<uint8_t>(AgricMsg_p->second));

    msg_gnssStatus.set__rtk_status(static_cast<uint8_t>(AgricMsg_p->RtkStatus));
    msg_gnssStatus.set__heading_status(static_cast<uint8_t>(AgricMsg_p->HeadingStatus));
    msg_gnssStatus.set__num_gps_sat(static_cast<uint8_t>(AgricMsg_p->numGPSsat));
    msg_gnssStatus.set__num_bds_sat(static_cast<uint8_t>(AgricMsg_p->numBDSsat));
    msg_gnssStatus.set__num_glo_sat(static_cast<uint8_t>(AgricMsg_p->numGLOsat));
    msg_gnssStatus.set__num_gal_sat(static_cast<uint8_t>(AgricMsg_p->numGALsat));

    msg_gnssStatus.set__gps_week_second(static_cast<int32_t>(AgricMsg_p->gps_week_second));
    msg_gnssStatus.set__diffage(static_cast<float>(AgricMsg_p->diffage));
    msg_gnssStatus.set__speed_heading(static_cast<float>(AgricMsg_p->speed_heading));
    msg_gnssStatus.set__undulation(static_cast<float>(AgricMsg_p->undulation));

    msg_navData.baseline.set__north(static_cast<float>(AgricMsg_p->Baseline_N));
    msg_navData.baseline.set__east(static_cast<float>(AgricMsg_p->Baseline_E));
    msg_navData.baseline.set__up(static_cast<float>(AgricMsg_p->Baseline_U));
    msg_navData.set__heading(static_cast<float>(AgricMsg_p->Heading));
    msg_navData.set__pitch(static_cast<float>(AgricMsg_p->Pitch));
    msg_navData.set__roll(static_cast<float>(AgricMsg_p->Roll));
    msg_navData.set__speed(static_cast<float>(AgricMsg_p->Speed));
    msg_navData.velocity.set__north(static_cast<float>(AgricMsg_p->Velocity_N));
    msg_navData.velocity.set__east(static_cast<float>(AgricMsg_p->Velocity_E));
    msg_navData.velocity.set__up(static_cast<float>(AgricMsg_p->Velocity_U));
    msg_navData.set__lat(static_cast<double>(AgricMsg_p->lat));
    msg_navData.set__lon(static_cast<double>(AgricMsg_p->lon));
    msg_navData.set__het(static_cast<double>(AgricMsg_p->Het));
    msg_navData.set__ecef_x(static_cast<double>(AgricMsg_p->ecef_x));
    msg_navData.set__ecef_y(static_cast<double>(AgricMsg_p->ecef_y));
    msg_navData.set__ecef_z(static_cast<double>(AgricMsg_p->ecef_z));
    msg_navData.set__base_lat(static_cast<double>(AgricMsg_p->base_lat));
    msg_navData.set__base_lon(static_cast<double>(AgricMsg_p->base_lon));
    msg_navData.set__base_alt(static_cast<double>(AgricMsg_p->base_alt));
    msg_navData.set__sec_lat(static_cast<double>(AgricMsg_p->sec_lat));
    msg_navData.set__sec_lon(static_cast<double>(AgricMsg_p->sec_lon));
    msg_navData.set__sec_alt(static_cast<double>(AgricMsg_p->sec_alt));

    msg_StdDevData.baseline_std.set__north(static_cast<float>(AgricMsg_p->Baseline_NStd));
    msg_StdDevData.baseline_std.set__east(static_cast<float>(AgricMsg_p->Baseline_EStd));
    msg_StdDevData.baseline_std.set__up(static_cast<float>(AgricMsg_p->Baseline_UStd));
    msg_StdDevData.set__sigma_v_x(static_cast<float>(AgricMsg_p->Xigema_Vx));
    msg_StdDevData.set__sigma_v_y(static_cast<float>(AgricMsg_p->Xigema_Vy));
    msg_StdDevData.set__sigma_v_z(static_cast<float>(AgricMsg_p->Xigema_Vz));
    msg_StdDevData.set__sigma_lat(static_cast<float>(AgricMsg_p->Xigema_lat));
    msg_StdDevData.set__sigma_lon(static_cast<float>(AgricMsg_p->Xigema_lon));
    msg_StdDevData.set__sigma_alt(static_cast<float>(AgricMsg_p->Xigema_alt));
    msg_StdDevData.set__sigma_ecef_x(static_cast<float>(AgricMsg_p->Xigema_ecef_x));
    msg_StdDevData.set__sigma_ecef_y(static_cast<float>(AgricMsg_p->Xigema_ecef_y));
    msg_StdDevData.set__sigma_ecef_z(static_cast<float>(AgricMsg_p->Xigema_ecef_z));

    pub_gnss_status_->publish(msg_gnssStatus);
    pub_nav_data_->publish(msg_navData);
    pub_std_deviation_data_->publish(msg_StdDevData);
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