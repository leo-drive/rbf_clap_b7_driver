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

int freq_rawimu = 0;
int freq_inspvax = 0;
int freq_agric = 0;

ClapB7Driver::ClapB7Driver()
    : Node("rbf_clap_b7_driver"),

      // Clap Specific Data Topic
      clap_imu_topic_{this->declare_parameter(
                               "clap_imu_data",
                               ParameterValue{"clap_imu"},
                               ParameterDescriptor{})
                           .get<std::string>()},
      
      clap_ins_topic_{this->declare_parameter(
                               "clap_ins_data",
                               ParameterValue{"clap_ins"},
                               ParameterDescriptor{})
                           .get<std::string>()},
      // Std. imu topic
      imu_topic_{this->declare_parameter(
                         "imu_topic",
                         ParameterValue{"gnss/imu"},
                         ParameterDescriptor{})
                     .get<std::string>()},
      // std. msgs
      nav_sat_fix_topic_{this->declare_parameter(
                                 "nav_sat_fix_topic",
                                 ParameterValue{"gnss/gps"},
                                 ParameterDescriptor{})
                             .get<std::string>()},
      
      twist_topic_{this->declare_parameter(
                                 "twist_topic",
                                 ParameterValue{"gnss/twist"},
                                 ParameterDescriptor{})
                             .get<std::string>()},

      odom_topic_{this->declare_parameter(
                                 "odom_topic",
                                 ParameterValue{"gnss/odom"},
                                 ParameterDescriptor{})
                             .get<std::string>()},


      autoware_orientation_topic_{this->declare_parameter("autoware_orientation_topic",
                            ParameterValue("gnss/autoware_orientation"),
                            ParameterDescriptor{})
        .get<std::string>()},

      // Serial port config
      serial_name_{this->declare_parameter(
                           "serial_name",
                           ParameterValue{"/dev/ttyUSB0"},
                           ParameterDescriptor{})
                       .get<std::string>()},

      baud_rate_{this->declare_parameter(
                         "baud_rate",
                         ParameterValue{460800},
                         ParameterDescriptor{})
                     .get<long>()},

      ntrip_server_ip_{this->declare_parameter("ntrip_ip",
                                                     ParameterValue("212.156.70.42"),
                                                     ParameterDescriptor{})
                                     .get<std::string>()},

      username_{this->declare_parameter("ntrip_user_name",
                                              ParameterValue("K0734151301"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      password_{this->declare_parameter("ntrip_password",
                                              ParameterValue("GzMSQg"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      debug_{this->declare_parameter("debug",
                                        ParameterValue("false"),
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
      activate_ntrip_{this->declare_parameter("activate_ntrip",
                                              ParameterValue("true"),
                                              ParameterDescriptor{})
                               .get<std::string>()},

      enu_ned_transform_{this->declare_parameter("enu_ned_transform",
                                              ParameterValue("true"),
                                              ParameterDescriptor{})
                                .get<std::string>()},

      time_system_{static_cast<int>(this->declare_parameter(
                           "time_system_selection",
                           ParameterValue{0},
                           ParameterDescriptor{})
                       .get<int>())},

      gnss_frame_{this->declare_parameter("gnss_frame",
                                          ParameterValue("gnss"),
                                          ParameterDescriptor{})
                      .get<std::string>()},

      imu_frame_{this->declare_parameter("imu_frame",
                                         ParameterValue("gnss"),
                                         ParameterDescriptor{})
                     .get<std::string>()},

      autoware_orientation_frame_{this->declare_parameter("autoware_orientation_frame",
                                                        ParameterValue("gnss"),
                                                        ParameterDescriptor{})
                                  .get<std::string>()},

      twist_frame_{this->declare_parameter("twist_frame",
                                         ParameterValue("gnss"),
                                         ParameterDescriptor{})
                     .get<std::string>()},

      coordinate_system_{static_cast<int>(this->declare_parameter(
                         "time_system_selection",
                         ParameterValue{4},
                         ParameterDescriptor{})
                     .get<int>())},

      local_origin_latitude{static_cast<double>(this->declare_parameter("local_origin_latitude",
                                                       ParameterValue(40.81187906),
                                                       ParameterDescriptor{})
                                      .get<double>())},

      local_origin_longitude{static_cast<double>(this->declare_parameter("local_origin_longitude",
                                                                    ParameterValue(29.35810110),
                                                                    ParameterDescriptor{})
                                              .get<double>())},
      local_origin_altitude{static_cast<double>(this->declare_parameter("local_origin_altitude",
                                                                     ParameterValue(47.62),
                                                                     ParameterDescriptor{})
                                               .get<double>())},
      // Publisher
      pub_clap_imu_{create_publisher<rbf_clap_b7_msgs::msg::ImuData>(
          clap_imu_topic_, rclcpp::QoS{10}, PubAllocT{})},
      
      pub_clap_ins_{create_publisher<rbf_clap_b7_msgs::msg::InsData>(
          clap_ins_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_imu_{create_publisher<sensor_msgs::msg::Imu>(
          imu_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_nav_sat_fix_{create_publisher<sensor_msgs::msg::NavSatFix>(
          nav_sat_fix_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_twist_{create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          twist_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_gnss_orientation_{create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
          autoware_orientation_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_odom_{create_publisher<nav_msgs::msg::Odometry>(
          odom_topic_, rclcpp::QoS{10}, PubAllocT{})},

      // Timer
      timer_{this->create_wall_timer(
          1000ms, std::bind(&ClapB7Driver::timer_callback, this))},

      tf_broadcaster_odom_{nullptr}

{
  using namespace std::placeholders;

  read_parameters();
  // Set serial callback
  tf_broadcaster_odom_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


  try{
    serial_boost.open(serial_name_, baud_rate_);
  }catch(boost::system::system_error& e){

    RCLCPP_ERROR(this->get_logger(), "\033[1;31m ClapB7 Serial port could not be opened: \033[0m  %s", e.what());
        
  }

  if( serial_boost.isOpen() == false){
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"), 
        "\033[1;31m ClapB7 Serial port %s could not be opened, plug and relaunch the package\033[0m\n",serial_name_.c_str());
  }
  else if(serial_boost.isOpen() == true){
    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"), 
        "\033[1;32m Serial port %s opened!!!\033[0m\n",serial_name_.c_str());
    // setting the serial receive callback function
    serial_boost.setCallback(bind(&ClapB7Driver::serial_receive_callback, this, _1, _2));

  }
  // Init ClapB7
  ClapB7Init(&clapB7Controller, bind(&ClapB7Driver::pub_imu_data, this), bind(&ClapB7Driver::pub_ins_data, this));

  if (activate_ntrip_ == "true") {
    
    if(NTRIP_client_start()){
      RCLCPP_INFO(this->get_logger(), "\033[1;32m NTRIP client connected \033[0m");
    }
    else{
      RCLCPP_INFO(this->get_logger(), "\033[1;31m NTRIP client cannot connected \033[0m");

    }
  }

  RCLCPP_INFO(this->get_logger(), "ClabB7 Driver Initiliazed");
}

void ClapB7Driver::read_parameters(){
  RCLCPP_INFO(this->get_logger(), "Parameters");
  RCLCPP_INFO(this->get_logger(), "NTRIP Serial Name: %s",serial_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Server ID: %s",ntrip_server_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP UserName: %s",username_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Password: %s",password_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP MountPoint: %s",mount_point_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Port: %d",ntrip_port_);
  RCLCPP_INFO(this->get_logger(), "Activate NTRIP: %s",activate_ntrip_.c_str());
  RCLCPP_INFO(this->get_logger(), "Clap Data Topic: %s",clap_imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Clap INS Data Topic: %s",clap_ins_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU Topic: %s",imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "NavSatFix Topic: %s",nav_sat_fix_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Autoware Orientation Topic: %s",autoware_orientation_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Twist Topic: %s",twist_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Time System Selection: %d",time_system_);
  RCLCPP_INFO(this->get_logger(), "ENU -> NED Transform: %s",enu_ned_transform_.c_str());
  RCLCPP_INFO(this->get_logger(), "NavSatFix Frame: %s",gnss_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU Frame: %s",imu_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Autoware Orientation Frame: %s",autoware_orientation_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Twist Frame: %s",twist_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug: %s",debug_.c_str());
  RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------");
}

void ClapB7Driver::serial_receive_callback(const char *data, unsigned int len)
{
  ClapB7Parser(&clapB7Controller, reinterpret_cast<const uint8_t *>(data), len);
}

void ClapB7Driver::timer_callback()
{
  if(debug_ == "true"){
    RCLCPP_WARN(this->get_logger(), "freq_rawimu_hz = %d\n", freq_rawimu);
    RCLCPP_WARN(this->get_logger(),"freq_inspvax_hz = %d\n", freq_inspvax);
    RCLCPP_WARN(this->get_logger(),"freq_agric_hz = %d\n", freq_agric);

    //For clap_bestgnss debugging
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %d\n",clapB7Controller.clap_BestGnssData.sol_status);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Velocity Type: %d\n",clapB7Controller.clap_BestGnssData.vel_type);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %f\n",clapB7Controller.clap_BestGnssData.latency);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %f\n",clapB7Controller.clap_BestGnssData.age);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %lf\n",clapB7Controller.clap_BestGnssData.horizontal_speed);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %lf\n",clapB7Controller.clap_BestGnssData.track_angle);
    RCLCPP_INFO(this->get_logger(),"ClapB7 BestGNSS Sol_Status: %lf\n",clapB7Controller.clap_BestGnssData.vertical_speed);
  }
  freq_rawimu = 0;
  freq_inspvax = 0;
  freq_agric = 0;
}
double ClapB7Driver::deg2rad(double degree){
  return (degree * (M_PI / 180));

}
void ClapB7Driver::pub_imu_data()
{

  rbf_clap_b7_msgs::msg::ImuData msg_imu_data;

  msg_imu_data.set__imu_status(static_cast<uint8_t>(clapB7Controller.clap_RawimuMsgs.imu_status));


  msg_imu_data.set__z_accel_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu_data.set__y_accel_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_accel_output * (-ACCEL_SCALE_FACTOR)* HZ_TO_SECOND));
  msg_imu_data.set__x_accel_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_accel_output * ACCEL_SCALE_FACTOR* HZ_TO_SECOND));
  msg_imu_data.set__z_gyro_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_gyro_output * GYRO_SCALE_FACTOR* HZ_TO_SECOND));
  msg_imu_data.set__y_gyro_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_gyro_output * (-GYRO_SCALE_FACTOR)* HZ_TO_SECOND));
  msg_imu_data.set__x_gyro_output(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_gyro_output * GYRO_SCALE_FACTOR* HZ_TO_SECOND));

  pub_clap_imu_->publish(msg_imu_data);
 
}

void ClapB7Driver::pub_ins_data() {

  rbf_clap_b7_msgs::msg::InsData msg_ins_data;

  msg_ins_data.set__ins_status(static_cast<uint32_t>(clapB7Controller.clapData.ins_status));
  msg_ins_data.set__pos_type(static_cast<uint32_t>(clapB7Controller.clapData.pos_type));

  msg_ins_data.set__latitude(static_cast<double>(clapB7Controller.clapData.latitude));
  msg_ins_data.set__longitude(static_cast<double>(clapB7Controller.clapData.longitude));

  msg_ins_data.set__height(static_cast<double>(clapB7Controller.clapData.height));

  msg_ins_data.set__undulation(static_cast<float>(clapB7Controller.clapData.undulation));

  msg_ins_data.set__north_velocity(static_cast<double>(clapB7Controller.clapData.north_velocity));
  msg_ins_data.set__east_velocity(static_cast<double>(clapB7Controller.clapData.east_velocity));
  msg_ins_data.set__up_velocity(static_cast<double>(clapB7Controller.clapData.up_velocity));

  msg_ins_data.set__roll(static_cast<double>(clapB7Controller.clapData.roll));
  msg_ins_data.set__pitch(static_cast<double>(clapB7Controller.clapData.pitch));
  msg_ins_data.set__azimuth(static_cast<double>(clapB7Controller.clapData.azimuth));

  msg_ins_data.set__std_dev_latitude(static_cast<float>(clapB7Controller.clapData.std_dev_latitude));
  msg_ins_data.set__std_dev_longitude(static_cast<float>(clapB7Controller.clapData.std_dev_longitude));

  msg_ins_data.set__std_dev_height(static_cast<float>(clapB7Controller.clapData.std_dev_height));

  msg_ins_data.set__std_dev_north_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_north_velocity));
  msg_ins_data.set__std_dev_east_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_east_velocity));
  msg_ins_data.set__std_dev_up_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_up_velocity));

  msg_ins_data.set__std_dev_roll(static_cast<float>(clapB7Controller.clapData.std_dev_roll));
  msg_ins_data.set__std_dev_pitch(static_cast<float>(clapB7Controller.clapData.std_dev_pitch));
  msg_ins_data.set__std_dev_azimuth(static_cast<float>(clapB7Controller.clapData.std_dev_azimuth));

  msg_ins_data.set__extended_solution_stat(static_cast<uint32_t>(clapB7Controller.clapData.extended_solution_stat));
  msg_ins_data.set__time_since_update(static_cast<uint16_t>(clapB7Controller.clapData.time_since_update));
  

  
  if (time_system_ == 0)
  {
    time_sec = pow(10, -9) * ros_time_to_gps_time_nano();
    time_nanosec = ros_time_to_gps_time_nano();
  }
  else
  {
    time_sec = this->get_clock()->now().seconds();
    time_nanosec = this->get_clock()->now().nanoseconds();
  }

  if ((clapB7Controller.clapData.ins_status == INS_INACTIVE) || (clapB7Controller.clapData.ins_status == INS_ALIGNING))
  {
    ins_active_ = 0;
    publish_standart_msgs_agric();

  }
  else
  {
    ins_active_ = 1;

    //Prints to terminal once
    std::call_once(flag_ins_active, [](){RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INS Active!!!\n");});

    publish_nav_sat_fix();
  }
    
  publish_std_imu();
  publish_twist();
  publish_orientation();
  publish_odom();

  pub_clap_ins_->publish(msg_ins_data);

}

void ClapB7Driver::publish_nav_sat_fix()
{
  
  sensor_msgs::msg::NavSatFix msg_nav_sat_fix;

  //  GNSS NavSatFix Message
  msg_nav_sat_fix.header.set__frame_id(static_cast<std::string>(gnss_frame_));

  msg_nav_sat_fix.header.stamp.set__sec(time_sec);
  msg_nav_sat_fix.header.stamp.set__nanosec(time_nanosec);

  msg_nav_sat_fix.status.set__status(clapB7Controller.clapData.ins_status);
  msg_nav_sat_fix.status.set__service(1); // GPS Connection

  msg_nav_sat_fix.set__latitude(static_cast<double>(clapB7Controller.clapData.latitude));
  msg_nav_sat_fix.set__longitude(static_cast<double>(clapB7Controller.clapData.longitude));
  msg_nav_sat_fix.set__altitude(static_cast<double>(clapB7Controller.clapData.height));



  std::array<double, 9> pos_cov{0.001};
  pos_cov[0] = std::pow(clapB7Controller.clapData.std_dev_latitude,2);
  pos_cov[4] = std::pow(clapB7Controller.clapData.std_dev_longitude,2);
  pos_cov[8] = std::pow(clapB7Controller.clapData.std_dev_height,2);


  msg_nav_sat_fix.set__position_covariance(pos_cov);

  pub_nav_sat_fix_->publish(msg_nav_sat_fix);
}

void ClapB7Driver::publish_standart_msgs_agric()
{
  sensor_msgs::msg::NavSatFix msg_nav_sat_fix;

  // GNSS NavSatFix Message
  msg_nav_sat_fix.header.set__frame_id(static_cast<std::string>(gnss_frame_));

  msg_nav_sat_fix.header.stamp.set__sec(time_sec);
  msg_nav_sat_fix.header.stamp.set__nanosec(time_nanosec);

  msg_nav_sat_fix.status.set__status(clapB7Controller.clapData.ins_status);
  msg_nav_sat_fix.status.set__service(1); // GPS Connection

  msg_nav_sat_fix.set__latitude(static_cast<double>(clapB7Controller.clap_ArgicData.lat));
  msg_nav_sat_fix.set__longitude(static_cast<double>(clapB7Controller.clap_ArgicData.lon));
  msg_nav_sat_fix.set__altitude(static_cast<double>(clapB7Controller.clap_ArgicData.Het));

  std::array<double, 9> pos_cov{0};
  pos_cov[0] = std::pow(clapB7Controller.clap_ArgicData.Xigema_lat,2);
  pos_cov[4] = std::pow(clapB7Controller.clap_ArgicData.Xigema_lon,2);
  pos_cov[8] = std::pow(clapB7Controller.clap_ArgicData.Xigema_alt,2);

  msg_nav_sat_fix.set__position_covariance(pos_cov);

  pub_nav_sat_fix_->publish(msg_nav_sat_fix);
}

void ClapB7Driver::publish_std_imu(){

  sensor_msgs::msg::Imu msg_imu;
  msg_imu.header.set__frame_id(static_cast<std::string>(imu_frame_));

  msg_imu.header.stamp.set__sec(time_sec);
  msg_imu.header.stamp.set__nanosec(time_nanosec);

  tf2::Quaternion quart_orient, quart_orient_corrected;
  tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_;

  if(enu_ned_transform_=="true"){
    //ENU -> NED Transformation
    //ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
    //ins_rot_matrix.setRotation(quart_orient);
    //ins_rot_ = ENU2NED * ins_rot_matrix;
    //ins_rot_.getRotation(quart_orient_corrected);


    double t_roll,t_pitch,t_yaw;
    tf2::Matrix3x3 m(quart_orient);
    m.getRPY(t_roll,t_pitch,t_yaw);

    double t_roll_ = t_roll*180/M_PI;
    double t_pitch_ = t_pitch*180/M_PI;
    double t_yaw_ = t_yaw*180/M_PI;


    Eigen::AngleAxisd angle_axis_x(deg2rad(t_roll_+180), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_axis_y(deg2rad(t_pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_axis_z(deg2rad(t_yaw_-90), Eigen::Vector3d::UnitZ());


    Eigen::Quaterniond p(angle_axis_x * angle_axis_y * angle_axis_z );

    Eigen::Quaterniond q = p.inverse();
    msg_imu.orientation.set__w(q.w());
    msg_imu.orientation.set__x(q.x());
    msg_imu.orientation.set__y(q.y());
    msg_imu.orientation.set__z(q.z());

  }
  else{
    msg_imu.orientation.set__w(quart_orient.getW());
    msg_imu.orientation.set__x(quart_orient.getX());
    msg_imu.orientation.set__y(quart_orient.getY());
    msg_imu.orientation.set__z(quart_orient.getZ());

  }


  msg_imu.angular_velocity.set__x(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_gyro_output * (M_PI / 180) * GYRO_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.angular_velocity.set__y(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_gyro_output * (M_PI / 180) * -(GYRO_SCALE_FACTOR) * HZ_TO_SECOND));
  msg_imu.angular_velocity.set__z(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_gyro_output * (M_PI / 180) * GYRO_SCALE_FACTOR * HZ_TO_SECOND));

  msg_imu.linear_acceleration.set__x(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.linear_acceleration.set__y(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_accel_output * (-ACCEL_SCALE_FACTOR) * HZ_TO_SECOND));
  msg_imu.linear_acceleration.set__z(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));

  std::array<double, 9> orient_cov{0.001};
  orient_cov[0] = std::pow(clapB7Controller.clapData.std_dev_roll,2);
  orient_cov[4] = std::pow(clapB7Controller.clapData.std_dev_pitch,2);
  orient_cov[8] = std::pow(clapB7Controller.clapData.std_dev_azimuth,2);

  msg_imu.set__orientation_covariance(orient_cov);

  std::array<double, 9> angular_vel_cov{0.001};
  msg_imu.set__angular_velocity_covariance(angular_vel_cov);

  std::array<double, 9> linear_acc_cov{0.001};
  msg_imu.set__linear_acceleration_covariance(linear_acc_cov);

  pub_imu_->publish(msg_imu);

}

bool ClapB7Driver::NTRIP_client_start()
{

  ntripClient.Init(ntrip_server_ip_, ntrip_port_, username_, password_, mount_point_);
  ntripClient.OnReceived([this](const char *buffer, int size)
                         {

                             serial_boost.write(buffer, size);

                             if(debug_ == "true"){
                              RCLCPP_INFO(this->get_logger(), "NTRIP Data size: %d",size);
                              RCLCPP_INFO(this->get_logger(), "NTRIP Status: %d",ntripClient.service_is_running());
                             }

                          });

  ntripClient.set_location(41.018893949, 28.890924848);

  ntripClient.set_report_interval(0.001);
  return ntripClient.Run();
}

int64_t ClapB7Driver::ros_time_to_gps_time_nano()
{

  int64_t time_nano = (k_GPS_SEC_IN_WEEK * static_cast<std::int64_t>(clapB7Controller.header.refWeekNumber * k_TO_NANO) + (k_MILI_TO_NANO * clapB7Controller.header.weekMs) + k_UNIX_OFFSET);

  return time_nano;
}

void ClapB7Driver::publish_twist(){

  geometry_msgs::msg::TwistWithCovarianceStamped msg_twist;

  msg_twist.header.set__frame_id(static_cast<std::string>(twist_frame_));
  msg_twist.header.stamp.set__sec(time_sec);
  msg_twist.header.stamp.set__nanosec(time_nanosec);

  double total_speed_linear= 0;
  double t_angular_speed_z= 0;

  if(ins_active_ == 0){
    total_speed_linear = std::sqrt(std::pow(clapB7Controller.clap_ArgicData.Velocity_E,2)+std::pow(clapB7Controller.clap_ArgicData.Velocity_N,2));
  }
  else if(ins_active_ == 1){
    total_speed_linear = std::sqrt(std::pow(clapB7Controller.clapData.east_velocity,2)+std::pow(clapB7Controller.clapData.north_velocity,2));
  }
  msg_twist.twist.twist.linear.x = total_speed_linear;

  t_angular_speed_z = deg2rad(clapB7Controller.clap_RawimuMsgs.z_gyro_output * GYRO_SCALE_FACTOR * HZ_TO_SECOND);


  msg_twist.twist.twist.angular.z = t_angular_speed_z;

  msg_twist.twist.covariance[0]  = 0.04;
  msg_twist.twist.covariance[7]  = 10000.0;
  msg_twist.twist.covariance[14] = 10000.0;
  msg_twist.twist.covariance[21] = 10000.0;
  msg_twist.twist.covariance[28] = 10000.0;
  msg_twist.twist.covariance[35] = 0.02;

  pub_twist_->publish(msg_twist);
}

void ClapB7Driver::publish_orientation()
{

  autoware_sensing_msgs::msg::GnssInsOrientationStamped msg_gnss_orientation;
  tf2::Quaternion quart_orient, quart_orient_corrected;
  tf2::Matrix3x3 ENU2NED, ins_rot_matrix, ins_rot_,clap2ros,ins_corrected_rot_;

  msg_gnss_orientation.header.set__frame_id(static_cast<std::string>(autoware_orientation_frame_));
  msg_gnss_orientation.header.stamp.set__sec(static_cast<int32_t>(time_sec));
  msg_gnss_orientation.header.stamp.set__nanosec(static_cast<uint32_t>(time_nanosec));



  if(ins_active_ == 0){
    quart_orient.setRPY(deg2rad(clapB7Controller.clap_ArgicData.Roll), deg2rad(clapB7Controller.clap_ArgicData.Pitch), deg2rad(clapB7Controller.clap_ArgicData.Heading));
  }
  else if(ins_active_ == 1){
    quart_orient.setRPY(deg2rad(clapB7Controller.clapData.roll), deg2rad(clapB7Controller.clapData.pitch), deg2rad(clapB7Controller.clapData.azimuth));
  }

  if(enu_ned_transform_=="true"){
    //ENU -> NED Transformation
    // ENU2NED = tf2::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1);
    // //clap2ros = tf2::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);
    // ins_rot_matrix.setRotation(quart_orient);
    // ins_rot_ = ENU2NED * ins_rot_matrix;
    // //ins_corrected_rot_ = ins_rot_ * clap2ros;
    // ins_rot_.getRotation(quart_orient_corrected);
    double t_roll,t_pitch,t_yaw;
    tf2::Matrix3x3 m(quart_orient);
    m.getRPY(t_roll,t_pitch,t_yaw);

    double t_roll_ = t_roll*180/M_PI;
    double t_pitch_ = t_pitch*180/M_PI;
    double t_yaw_ = t_yaw*180/M_PI;


    Eigen::AngleAxisd angle_axis_x(deg2rad(t_roll_+180), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd angle_axis_y(deg2rad(t_pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd angle_axis_z(deg2rad(t_yaw_-90), Eigen::Vector3d::UnitZ());


    Eigen::Quaterniond p(angle_axis_x * angle_axis_y * angle_axis_z );

    Eigen::Quaterniond q = p.inverse();

    msg_gnss_orientation.orientation.orientation.set__w(q.w());
    msg_gnss_orientation.orientation.orientation.set__x(q.x());
    msg_gnss_orientation.orientation.orientation.set__y(q.y());
    msg_gnss_orientation.orientation.orientation.set__z(q.z());


  }
  else{
    msg_gnss_orientation.orientation.orientation.set__w(quart_orient.getW());
    msg_gnss_orientation.orientation.orientation.set__x(quart_orient.getX());
    msg_gnss_orientation.orientation.orientation.set__y(quart_orient.getY());
    msg_gnss_orientation.orientation.orientation.set__z(quart_orient.getZ());

  }

  msg_gnss_orientation.orientation.rmse_rotation_x = clapB7Controller.clapData.std_dev_roll;
  msg_gnss_orientation.orientation.rmse_rotation_y = clapB7Controller.clapData.std_dev_pitch;
  msg_gnss_orientation.orientation.rmse_rotation_z = clapB7Controller.clapData.std_dev_azimuth;

  pub_gnss_orientation_->publish(msg_gnss_orientation);

}

void ClapB7Driver::publish_odom(){

  nav_msgs::msg::Odometry msg_odom;
  double utm_northing, utm_easting;
  std::string utm_zone;
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::NavSatFix nav_sat_fix_msg;

  sensor_msgs::msg::NavSatFix nav_sat_fix_origin;

  GNSSStat gnss_stat;

  nav_sat_fix_origin.longitude =  local_origin_longitude;
  nav_sat_fix_origin.latitude = local_origin_latitude;
  nav_sat_fix_origin.altitude = local_origin_altitude;

  nav_sat_fix_msg.longitude =  clapB7Controller.clapData.longitude;
  nav_sat_fix_msg.latitude = clapB7Controller.clapData.latitude;
  nav_sat_fix_msg.altitude = clapB7Controller.clapData.height;

  gnss_stat = NavSatFix2LocalCartesianUTM(nav_sat_fix_msg,nav_sat_fix_origin);

  RCLCPP_INFO(this->get_logger(), "GNSS Status x: %f", gnss_stat.x);
  RCLCPP_INFO(this->get_logger(), "GNSS Status y: %f", gnss_stat.y);
  RCLCPP_INFO(this->get_logger(), "GNSS Status z: %f", gnss_stat.z);

  // Fill in the message.
  msg_odom.header.stamp.set__sec(time_sec);
  msg_odom.header.stamp.set__nanosec(time_nanosec);

  msg_odom.header.set__frame_id("odom");
  msg_odom.set__child_frame_id(static_cast<std::string>(gnss_frame_));

  msg_odom.pose.pose.position.x = gnss_stat.x;
  msg_odom.pose.pose.position.y = gnss_stat.y;
  msg_odom.pose.pose.position.z = gnss_stat.z;

  // msg_odom.pose.covariance[0*6 + 0] = std_x * std_x;
  // msg_odom.pose.covariance[1*6 + 1] = std_y * std_y;
  // msg_odom.pose.covariance[2*6 + 2] = std_z * std_z;
  msg_odom.pose.covariance[3*6 + 3] = clapB7Controller.clapData.std_dev_roll * clapB7Controller.clapData.std_dev_roll;
  msg_odom.pose.covariance[4*6 + 4] = clapB7Controller.clapData.std_dev_pitch * clapB7Controller.clapData.std_dev_pitch;
  msg_odom.pose.covariance[5*6 + 5] = clapB7Controller.clapData.std_dev_azimuth * clapB7Controller.clapData.std_dev_azimuth;

  //The twist message gives the linear and angular velocity relative to the frame defined in child_frame_id
  //Lİnear x-y-z hızlari yanlis olabilir
  msg_odom.twist.twist.linear.x      = clapB7Controller.clap_ArgicData.Velocity_E;
  msg_odom.twist.twist.linear.y      = clapB7Controller.clap_ArgicData.Velocity_N;
  msg_odom.twist.twist.linear.z      = clapB7Controller.clap_ArgicData.Velocity_U;
  msg_odom.twist.twist.angular.x     = clapB7Controller.clap_RawimuMsgs.x_gyro_output;
  msg_odom.twist.twist.angular.y     = clapB7Controller.clap_RawimuMsgs.y_gyro_output;
  msg_odom.twist.twist.angular.z     = clapB7Controller.clap_RawimuMsgs.z_gyro_output;
  msg_odom.twist.covariance[0*6 + 0] = clapB7Controller.clapData.std_dev_east_velocity * clapB7Controller.clapData.std_dev_east_velocity;
  msg_odom.twist.covariance[1*6 + 1] = clapB7Controller.clapData.std_dev_north_velocity * clapB7Controller.clapData.std_dev_north_velocity;
  msg_odom.twist.covariance[2*6 + 2] = clapB7Controller.clapData.std_dev_up_velocity * clapB7Controller.clapData.std_dev_up_velocity;
  msg_odom.twist.covariance[3*6 + 3] = 0;
  msg_odom.twist.covariance[4*6 + 4] = 0;
  msg_odom.twist.covariance[5*6 + 5] = 0;

  publish_transform(msg_odom.header.frame_id, "dummy", msg_odom.pose.pose,transform);
  pub_odom_->publish(msg_odom);

}

void ClapB7Driver::publish_transform(
  const std::string &ref_parent_frame_id,
  const std::string &ref_child_frame_id,
  const geometry_msgs::msg::Pose &ref_pose, 
  geometry_msgs::msg::TransformStamped &ref_transform){


  ref_transform.header.stamp.set__sec(time_sec);
  ref_transform.header.stamp.set__nanosec(time_nanosec);

  ref_transform.header.frame_id = ref_parent_frame_id;
  ref_transform.child_frame_id = ref_child_frame_id;

  ref_transform.transform.translation.set__x(ref_pose.position.x);
  ref_transform.transform.translation.set__y(ref_pose.position.y);
  ref_transform.transform.translation.set__z(ref_pose.position.z);

  ref_transform.transform.rotation.set__x(ref_pose.orientation.x);
  ref_transform.transform.rotation.set__y(ref_pose.orientation.y);
  ref_transform.transform.rotation.set__z(ref_pose.orientation.z);
  ref_transform.transform.rotation.set__w(ref_pose.orientation.w);

  tf_broadcaster_odom_->sendTransform(ref_transform);


}
ClapB7Driver::GNSSStat ClapB7Driver::NavSatFix2LocalCartesianUTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg,
  sensor_msgs::msg::NavSatFix nav_sat_fix_origin)
{
  GNSSStat utm_local;
  utm_local.coordinate_system = CoordinateSystem::UTM;

  // origin of the local coordinate system in global frame
  GNSSStat utm_origin;
  utm_origin.coordinate_system = CoordinateSystem::UTM;
  GeographicLib::UTMUPS::Forward(
    nav_sat_fix_origin.latitude, nav_sat_fix_origin.longitude, utm_origin.zone,
    utm_origin.east_north_up, utm_origin.x, utm_origin.y);
  utm_origin.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_origin);
  // individual coordinates of global coordinate system
  double global_x = 0.0;
  double global_y = 0.0;
  GeographicLib::UTMUPS::Forward(
    nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm_origin.zone,
    utm_origin.east_north_up, global_x, global_y);
  utm_local.latitude = nav_sat_fix_msg.latitude;
  utm_local.longitude = nav_sat_fix_msg.longitude;
  utm_local.altitude = nav_sat_fix_msg.altitude;
  // individual coordinates of local coordinate system
  utm_local.x = global_x - utm_origin.x;
  utm_local.y = global_y - utm_origin.y;
  utm_local.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg) - utm_origin.z;

  return utm_local;
}


double ClapB7Driver::EllipsoidHeight2OrthometricHeight(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  double OrthometricHeight{0.0};

  GeographicLib::Geoid egm2008("egm2008-1");
  OrthometricHeight = egm2008.ConvertHeight(
    nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
    GeographicLib::Geoid::ELLIPSOIDTOGEOID);

  return OrthometricHeight;
}