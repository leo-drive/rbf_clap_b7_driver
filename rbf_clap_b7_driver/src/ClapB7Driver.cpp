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
                               ParameterValue{"/clap_imu"},
                               ParameterDescriptor{})
                           .get<std::string>()},
      
      clap_ins_topic_{this->declare_parameter(
                               "clap_ins_data",
                               ParameterValue{"/clap_ins"},
                               ParameterDescriptor{})
                           .get<std::string>()},
      // Std. imu topic
      imu_topic_{this->declare_parameter(
                         "imu_topic",
                         ParameterValue{"/gnss/imu"},
                         ParameterDescriptor{})
                     .get<std::string>()},
      // std. msgs
      nav_sat_fix_topic_{this->declare_parameter(
                                 "nav_sat_fix_topic",
                                 ParameterValue{"/gnss/gps"},
                                 ParameterDescriptor{})
                             .get<std::string>()},
      
      twist_topic_{this->declare_parameter(
                                 "twist_topic",
                                 ParameterValue{"/gnss/twist"},
                                 ParameterDescriptor{})
                             .get<std::string>()},


      autoware_orientation_topic_{this->declare_parameter("autoware_orientation_topic",
                            ParameterValue("/gnss/orientation"),
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

      serial_boost(serial_name_, baud_rate_),


      ntrip_server_ip_{this->declare_parameter("ntrip_ip",
                                                     ParameterValue("212.156.70.42"),
                                                     ParameterDescriptor{})
                                     .get<std::string>()},

      username_{this->declare_parameter("ntrip_user_name",
                                              ParameterValue("K073432501"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      password_{this->declare_parameter("ntrip_password",
                                              ParameterValue("GR3g4"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      debug_{this->declare_parameter("debug",
                                        ParameterValue("true"),
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

      time_system_{this->declare_parameter(
                           "time_system_selection",
                           ParameterValue{0},
                           ParameterDescriptor{})
                       .get<int>()},

      gnss_frame_{this->declare_parameter("gnss_frame",
                                          ParameterValue("gnss"),
                                          ParameterDescriptor{})
                      .get<std::string>()},

      imu_frame_{this->declare_parameter("imu_frame",
                                         ParameterValue("gnss"),
                                         ParameterDescriptor{})
                     .get<std::string>()},

      twist_frame_{this->declare_parameter("twist_frame",
                                         ParameterValue("gnss"),
                                         ParameterDescriptor{})
                     .get<std::string>()},

      autoware_orientation_frame_{this->declare_parameter("autoware_orientation_frame",
                                                          ParameterValue("gnss"),
                                                          ParameterDescriptor{})
                                      .get<std::string>()},

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

      // Timer
      timer_{this->create_wall_timer(
          1000ms, std::bind(&ClapB7Driver::timer_callback, this))}

{
  using namespace std::placeholders;

  read_parameters();
  // Set serial callback
  serial_boost.setCallback(bind(&ClapB7Driver::serial_receive_callback, this, _1, _2));

  // Init ClapB7
  ClapB7Init(&clapB7Controller, bind(&ClapB7Driver::pub_imu_data, this), bind(&ClapB7Driver::pub_ins_data, this));

  if (activate_ntrip_ == "true") {
    NTRIP_client_start();
  }

  RCLCPP_INFO(this->get_logger(), "ClabB7 Driver Initiliazed");
}

void ClapB7Driver::read_parameters(){
  RCLCPP_INFO(this->get_logger(), "Parameters");
  RCLCPP_INFO(this->get_logger(), "NTRIP Serial Name: %s\n",serial_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Server ID: %s\n",ntrip_server_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP UserName: %s\n",username_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Password: %s\n",password_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP MountPoint: %s\n",mount_point_.c_str());
  RCLCPP_INFO(this->get_logger(), "NTRIP Port: %d\n",ntrip_port_);
  RCLCPP_INFO(this->get_logger(), "Activate NTRIP: %s\n",activate_ntrip_.c_str());
  RCLCPP_INFO(this->get_logger(), "Clap Data Topic: %s\n",clap_data_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Clap INS Data Topic: %s\n",clap_ins_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU Topic: %s\n",imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "NavSatFix Topic: %s\n",nav_sat_fix_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Autoware Orientation Topic: %s\n",autoware_orientation_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Twist Topic: %s\n",twist_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Time System Selection: %d\n",time_system_);
  RCLCPP_INFO(this->get_logger(), "ENU -> NED Transform: %s\n",enu_ned_transform_.c_str());
  RCLCPP_INFO(this->get_logger(), "NavSatFix Frame: %s\n",gnss_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU Frame: %s\n",imu_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Autoware Orientation Frame: %s\n",autoware_orientation_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Twist Frame: %s\n",twist_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug: %s\n",debug_.c_str());
  RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------",debug_.c_str());
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

  if(ins_active_ == 0){
    quart_orient.setRPY(deg2rad(clapB7Controller.clap_ArgicData.Roll), deg2rad(clapB7Controller.clap_ArgicData.Pitch), deg2rad(clapB7Controller.clap_ArgicData.Heading));
    

    if(debug_=="true"){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------RAW-----------\n");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: %f , Pitch: %f, Yaw: %f\n",clapB7Controller.clap_ArgicData.Roll
                                                                                 ,clapB7Controller.clap_ArgicData.Pitch
                                                                                 ,clapB7Controller.clap_ArgicData.Heading);
    }
  
  }
  else if(ins_active_ == 1){
    quart_orient.setRPY(deg2rad(clapB7Controller.clapData.roll), deg2rad(clapB7Controller.clapData.pitch), deg2rad(clapB7Controller.clapData.azimuth));
  
    if(debug_=="true"){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------RAW-----------\n");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: %f , Pitch: %f, Yaw: %f\n",clapB7Controller.clapData.roll
                                                                                 ,clapB7Controller.clapData.pitch
                                                                                 ,clapB7Controller.clapData.azimuth);
    }
  
  }
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

  if(debug_=="true"){
    tf2::Quaternion q(msg_imu.orientation.x,msg_imu.orientation.y,msg_imu.orientation.z,msg_imu.orientation.w);
    tf2::Matrix3x3 m(q);
    double t_roll, t_pitch, t_yaw;
    m.getRPY(t_roll,t_pitch,t_yaw);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-------------CONVERTED-----------\n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: %f , Pitch: %f, Yaw: %f\n",t_roll,t_pitch,t_yaw);
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

  /*
double t_roll, t_pitch, t_yaw;

  t_pitch = 180 * atan(clapB7Controller.clap_RawimuMsgs.x_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.y_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;

  t_roll = 180 * atan(clapB7Controller.clap_RawimuMsgs.y_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.x_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;

  t_yaw = 180 * atan(clapB7Controller.clap_RawimuMsgs.z_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.x_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;
  float old = 0;

  float delta_theta_z = (clapB7Controller.clap_RawimuMsgs.z_accel_output - old) * 0.01;

  old = clapB7Controller.clap_RawimuMsgs.z_accel_output;

  t_yaw += delta_theta_z * 180 / M_PI;
  */
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
  ntrip_status_ = ntripClient.Run();
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
