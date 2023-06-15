/*!
*	\file         ClapB7driver.hpp
*	\author       Robeff Technology - farukbaykara
*	\date         22/05/2022
*
*	\brief        Manage publishment of IMU and GNSS data as ROS and custom ros messages.
*/

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <mutex>

#include "TermiosSerial.h"
#include "AsyncSerial.h"

#include "rclcpp/rclcpp.hpp"

#include "nmea_msgs/msg/gpgga.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mavros_msgs/msg/rtcm.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <rbf_clap_b7_msgs/msg/clap_data.hpp>
#include <rbf_clap_b7_msgs/msg/imu_data.hpp>
#include <rbf_clap_b7_msgs/msg/ins_data.hpp>

#include <ntrip/ntrip_client.h>
#include "ClapB7BinaryParser.h"

#define ACCEL_SCALE_FACTOR (400 / (pow(2, 31)))
#define GYRO_SCALE_FACTOR (2160 / (pow(2, 31)))
#define HZ_TO_SECOND ( 100 )

// ROS Time to GPS time Parameters
#define k_TO_NANO (1e9)
#define k_GPS_SEC_IN_WEEK (60 * 60 * 24 * 7)
#define k_UNIX_OFFSET (315964782000000000)
#define k_MILI_TO_NANO (1e6)

#define INS_INACTIVE    0
#define INS_ALIGNING    1

extern int freq_rawimu;
extern int freq_inspvax;
extern int freq_agric;
extern int freq_bestgnss;


class ClapB7Driver : public rclcpp::Node
{
public:

    //---------------------------------------------------------------------//
    //- Constructor                                                       -//
    //---------------------------------------------------------------------//

    /*!
     * Default constructor.
     */
    ClapB7Driver();


    //Destructur
    ~ClapB7Driver() override {
        serial_boost.close();
    }

    /*!
     * @brief Callback function for serial port data reception.
     * @param data Received data.
     * @param len Received data length.
     * */
    void serial_receive_callback(const char *data, unsigned int len);

private:

    /*!
     * @brief Struct for UTM0 coordinates
     * */
    typedef struct _UTM0
    {
        double			easting;
        double			northing;
        double			altitude;
        int				zone;
    } UTM0;
    
    void timer_callback();

    /*!
     * @brief Publish custom ROS IMU and INS-GNSS data
     * */
    void pub_imu_data();
    void pub_ins_data();

    /*!
     * @brief Publish received EKF ROS Nav_sat_fix messages
     * */
    void publish_nav_sat_fix();

    /*!
     * @brief Publish received EKF ROS IMU messages
     * */
    void publish_std_imu();

    /*!
     * @brief Get current ROS time and convert it to GPS time in nano seconds.
     * */
    int64_t ros_time_to_gps_time_nano();

    /*!
     * @brief Publish received ROS Raw Nav_Sat_Fix message
     * */
    void publish_standart_msgs_agric();

    /*!
     * @brief Publish received EKF ROS Twist with Covairance Stamped message
     * */
    void publish_twist();

    /*!
     * @brief Publish received EKF Autoware Orientation message
     * */
    void publish_orientation();

    /*!
     * @brief Publish received EKF ROS Odometry message
     * */
    void publish_odom();

    /*!
     * @brief Publish tf2::TransformStamped message using parent frame to child frame
     * @param ref_parent_frame_id Parent frame id
     * @param ref_child_frame_id Child frame id
     * @param ref_pose Pose of child frame in parent frame
     * @param ref_transform TransformStamped message to be published
     * */
    void publish_transform(const std::string &ref_parent_frame_id,
                            const std::string &ref_child_frame_id,
                            const geometry_msgs::msg::Pose &ref_pose, 
                            geometry_msgs::msg::TransformStamped &ref_transform);

    /*!
     * @brief Publish received raw GPS ROS Nav_sat_fix message
     * */
    void publish_raw_nav_sat_fix();

    /*!
     * @brief Publish received raw GPS ROS IMU message
     * */
    void publish_raw_imu();

    /*!
     * @brief Return 'degree' param to radian
     * @param degree Degree value to be converted
     * */
    double deg2rad(double degree);

    /*!
     * @brief Write parameters read from YAML file to terminal in start of node
     * */
    void read_parameters();

    /*!
     * Lat-Lon to UTM conversion functions
     * */
    void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting);

    double computeMeridian(int zone_number);

    void initUTM(double Lat, double Long, double altitude);

    char UTMLetterDesignator(double Lat);


    /*!
     * @brief Convert ENU to NED
     * @param q_in Quaternion to be converted
     * */
    void transform_enu_to_ned(tf2::Quaternion &q_in);

    /*!
     * @brief Callback definition for RTK RTCM message received from NTRIP client topic
     * @param msg_rtcm RTCM message received from NTRIP client
     * */
    void rtcmCallback(const mavros_msgs::msg::RTCM::ConstSharedPtr msg_rtcm);

    //Topics
    std::string clap_data_topic_;
    std::string clap_imu_topic_;
    std::string clap_ins_topic_;
    std::string imu_topic_;
    std::string nav_sat_fix_topic_;
    std::string autoware_orientation_topic_;
    std::string twist_topic_;
    std::string odom_topic_;
    std::string rtcm_topic_;
    std::string raw_nav_sat_fix_topic_;
    std::string raw_imu_topic_;

    //NTRIP Parameters
    std::string activate_ntrip_;

    //Serial port params
    std::string serial_name_;
    long baud_rate_;
    std::string enu_ned_transform_;
    std::string debug_;

    //GPS time variables
    bool time_system_;
    int64_t time_sec;
    int64_t time_nanosec;

    //'True' if INS is active
    bool ins_active_;

    UTM0 m_utm0_;    


    //Frame Names
    std::string gnss_frame_;
    std::string imu_frame_;
    std::string autoware_orientation_frame_;
    std::string twist_frame_;

    CallbackAsyncSerial serial_boost;

    //Publishers
    rclcpp::Publisher<rbf_clap_b7_msgs::msg::ClapData>::SharedPtr pub_clap_data_;
    rclcpp::Publisher<rbf_clap_b7_msgs::msg::ImuData>::SharedPtr pub_clap_imu_;
    rclcpp::Publisher<rbf_clap_b7_msgs::msg::InsData>::SharedPtr pub_clap_ins_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;
    rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr pub_gnss_orientation_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_raw_nav_sat_fix_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw_imu_;

    //RTK RTCM message Subscriber
    rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr sub_rtcm_;

    //ClapB7Controller object to store received data
    ClapB7Controller clapB7Controller;

    std::once_flag flag_ins_active;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_;
};


