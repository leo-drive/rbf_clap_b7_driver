//
// Created by mehce on 13.12.2022.
//

#pragma once

#include <cstdint>
#include <functional>

#define HEADER_LEN 	 	28U
#define CRC_LEN			 4U
#define MAX_DATA_LEN 	512U

#define FIRST_SYNCH		0xAAU
#define SECOND_SYNCH	0x44U
#define THIRD_SYNCH		0x12U

#define RAWIMU_MSG_ID    268U
#define INSPVAX_MSG_ID   1465U


typedef enum
{
    SYNCH1_CONTROL,
    SYNCH2_CONTROL,
    SYNCH3_CONTROL,
    CLAP_B7_HEADER_LENGTH,
    CLAP_B7_HEADER_ADD,
    CLAP_B7_DATA_ADD,
}ClapB7ParseStatus;


#pragma pack(1)
typedef struct
{
    uint8_t SYNCH1;
    uint8_t SYNCH2;
    uint8_t SYNCH3;
    uint8_t headerLength;
    uint16_t messageID;
    uint8_t messageType;
    uint8_t portAddress;
    uint16_t messageLen;
    uint16_t sequence;
    uint8_t idleTime;
    uint8_t timeStatus;
    uint16_t refWeekNumber;
    uint32_t weekMs;
    uint32_t receiverStatus;
    uint16_t reserved;
    uint16_t receiverSwVer;
}ClapB7Header;

struct __attribute__((packed)) ClapB7_InspvaxMsgs_ {
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

};

struct __attribute__((packed)) ClapB7_RawimuMsgs_ {
    uint32_t gnssWeek;
    double  secondsIntoWeek;
    uint32_t imu_status;
    int32_t z_accel_output;
    int32_t y_accel_output;
    int32_t x_accel_output;
    int32_t z_gyro_output;
    int32_t y_gyro_output;
    int32_t x_gyro_output;
    int crc;
};

typedef struct
{
    uint16_t dataIndex;
    ClapB7ParseStatus status;
    ClapB7Header header;
    uint8_t rawData[MAX_DATA_LEN];
    ClapB7_InspvaxMsgs_ clapData;
    ClapB7_RawimuMsgs_  clap_RawimuMsgs;
    std::function<void()> Parser;
} ClapB7Controller;

void ClapB7Init(ClapB7Controller* p_Controller, const std::function<void()> callBack);
void ClapB7Parser(ClapB7Controller* p_Controller, const uint8_t* p_Data, uint16_t len);