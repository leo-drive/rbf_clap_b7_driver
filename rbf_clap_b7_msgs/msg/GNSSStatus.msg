# GNSS Status

uint8 command_length             # The digit length from GNSS to CRC is 232 bytes, Fixed value: 0XE8
uint8 year                       # UTC time -year
uint8 month                      # UTC time -month
uint8 day                        # UTC time -day
uint8 hour                       # UTC time -hour
uint8 minute                     # UTC time -minute
uint8 second                     # UTC time-second

uint8 rtk_status                  # Rover position status  0: Ineffective: 1: Single point: 2: Pseudo-range differential: 4: Fix solution: 5: Float solution:
uint8 heading_status              # Heading solution status of master and slave antennas  0: Ineffective: 4: Fix solution: 5: Float solution:
uint8 num_gps_sat                  # GPS satellite number
uint8 num_bds_sat                  # BDS satellite number
uint8 num_glo_sat                  # GLONASS satellite number
uint8 num_gal_sat                  # Galileo satellite number
    
int32 gps_week_second            # Number of milliseconds into the GPS reference week
float32 diffage                  # Differential age
float32 speed_heading            # Direction of velocity
float32 undulation               # Height outlier
