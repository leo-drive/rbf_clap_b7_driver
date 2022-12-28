//
// Created by mehce on 13.12.2022.
//

#pragma once

#include <cstdint>
#include <functional>

#define HEADER_LEN 	 	24U
#define CRC_LEN			 4U
#define MAX_DATA_LEN 	512U

#define FIRST_SYNCH		0xAAU
#define SECOND_SYNCH	0x44U
#define THIRD_SYNCH		0xB5U


typedef enum
{
    SYNCH1_CONTROL,
    SYNCH2_CONTROL,
    SYNCH3_CONTROL,
    CLAP_B7_HEADER_ADD,
    CLAP_B7_DATA_ADD,
}ClapB7ParseStatus;


#pragma pack(1)
typedef struct
{
    uint8_t SYNCH1;
    uint8_t SYNCH2;
    uint8_t SYNCH3;
    uint8_t cpuIDLE;
    uint16_t messageID;
    uint16_t msgLen;
    uint8_t timeRef;
    uint8_t timeStatus;
    uint16_t refWeekNumber;
    uint32_t weekMs;
    uint32_t reserved;
    uint8_t releaseVer;
    uint8_t leapSec;
    uint16_t outDelay;
}ClapB7Header;

struct __attribute__((packed)) ClapB7Data {
    uint32_t ins_status;
    uint32_t pos_type;
    double latitude;
    double longitude;
    double height;
    float undulation;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double roll;
    double pitch;
    double azimuth;
    float std_dev_latitude;
    float std_dev_longitude;
    float std_dev_height;
    float std_dev_north_velocity;
    float std_dev_east_velocity;
    float std_dev_up_velocity;
    float std_dev_roll;
    float std_dev_pitch;
    float std_dev_azimuth;
    uint32_t extended_solution_stat;
    uint16_t time_since_update;
    uint8_t imu_error;
    uint8_t imu_type;
    int32_t z_accel_output;
    int32_t y_accel_output;
    int32_t x_accel_output;
    int32_t z_gyro_output;
    int32_t y_gyro_output;
    int32_t x_gyro_output;
    uint8_t gps_sat_num;
    uint8_t bd_sat_num;
    uint8_t glo_sat_num;
    uint8_t gal_sat_num;
    float rtk_delay;
    float gdop;
    float remain_float_1;
    float remain_float_2;
    double remain_double;
    unsigned char remain_char_1;
    unsigned char remain_char_2;
    unsigned char remain_char_3;
    unsigned char remain_char_4;
    int crc;
};

typedef struct
{
    uint16_t dataIndex;
    ClapB7ParseStatus status;
    ClapB7Header header;
    uint8_t rawData[MAX_DATA_LEN];
    ClapB7Data clapData;
    std::function<void()> Parser;
} ClapB7Controller;

void ClapB7Init(ClapB7Controller* p_Controller, const std::function<void()> callBack);
void ClapB7Parser(ClapB7Controller* p_Controller, const uint8_t* p_Data, uint16_t len);