//
// Created by coskun on 25.05.2022.
//

#pragma once

#include <cstring>
#include <stdint.h>


struct __attribute__((packed)) AgricMsg_ {
    uint8_t command_length;            // The digit length from GNSS to CRC is 232 bytes, Fixed value: 0XE8
    uint8_t year;                      // UTC time -year
    uint8_t month;                     // UTC time -month
    uint8_t day;                       // UTC time -day
    uint8_t hour;                      // UTC time -hour
    uint8_t minute;                    // UTC time -minute
    uint8_t second;                    // UTC time-second

    uint8_t RtkStatus;                 // Rover position status  0: Ineffective: 1: Single point: 2: Pseudo-range differential: 4: Fix solution: 5: Float solution:
    uint8_t HeadingStatus;             // Heading solution status of master and slave antennas  0: Ineffective: 4: Fix solution: 5: Float solution:
    uint8_t numGPSsat;                 // GPS satellite number
    uint8_t numBDSsat;                 // BDS satellite number
    uint8_t numGLOsat;                 // GLONASS satellite number

    float Baseline_N;               // From the base to rover baseline vector, Northing component
    float Baseline_E;               // From the base to rover baseline vector, Easting component
    float Baseline_U;               // From the base to rover baseline vector, zenith direction. component standard deviation
    float Baseline_NStd;            // From the base to rover baseline vector, Northing component standard deviation
    float Baseline_EStd;            // From the base to rover baseline vector, Easting component standard deviation
    float Baseline_UStd;            // From the base to rover baseline vector, zenith direction. component standard deviation

    float Heading;                  // Heading
    float Pitch;                    // Pitch
    float Roll;                     // Roll
    float Speed;                    // Speed
    float Velocity_N;               // Northing velocity
    float Velocity_E;               // Easting velocity
    float Velocity_U;               // velocity

    float Xigema_Vx;                // Northing velocity standard deviation
    float Xigema_Vy;                // Easting velocity standard deviation
    float Xigema_Vz;                // velocity standard deviation

    double lat;                     // Rover station latitude (-90 to 90 degrees)   /  a ‘-’ sign denotes south and a ‘+’ sign denotes north
    double lon;                     // Rover station longitude (-180 to 180 degrees)   /  a ‘-’ sign denotes west and a ‘+’ sign denotes east
    double Het;                     // Rover station height

    double ecef_x;                  // ECEF X value (m)
    double ecef_y;                  // ECEF Y value (m)
    double ecef_z;                  // ECEF Z value (m)

    float Xigema_lat;               // Latitude standard deviation
    float Xigema_lon;               // Longitude standard deviation
    float Xigema_alt;               // Height standard deviation
    float Xigema_ecef_x;            // ECEF_X standard deviation
    float Xigema_ecef_y;            // ECEF_Y standard deviation
    float Xigema_ecef_z;            // ECEF_Z standard deviation

    double base_lat;                // Base station latitude (-90 to 90 degrees)
    double base_lon;                // Base station longitude(-180 to 180 degrees)
    double base_alt;                // Base station height

    double sec_lat;                // Sub-antenna latitude (-90 to 90 degrees)
    double sec_lon;                // Sub-antenna longitude(-180 to 180 degrees)
    double sec_alt;                // Sub-antenna height

    int gps_week_second;           // Number of milliseconds into the GPS reference week
    float diffage;                 // Differential age
    float speed_heading;           // Direction of velocity
    float undulation;              // Height outlier
    float remain_float3;           // Reserved
    float remain_float4;           // Reserved

    uint8_t numGALsat;                // Galileo satellite number
    uint8_t remain_char2;             // Reserved
    uint8_t remain_char3;             // Reserved
//    char remain_char4;             // Reserved

//    int32_t crc_hex;               // 32 bits CRC (only Binary and ASCII)
};

struct ClapImuData {
    unsigned char error;
    unsigned char type;
    unsigned short week;

    double secToWeek;
    unsigned long status;

    long z_accel;
    long y_accel;
    long x_accel;

    long z_gyro;
    long y_gyro;
    long x_gyro;
    int crc;
};
