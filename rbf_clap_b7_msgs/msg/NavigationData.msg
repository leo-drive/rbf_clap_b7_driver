# Nabigation Data

ENU baseline                      # From the base to rover baseline vectors

float32 heading                   # Heading
float32 pitch                     # Pitch
float32 roll                      # Roll
float32 speed                     # Speed
ENU velocity                      # velocitys

float64 lat                      # Rover station latitude (-90 to 90 degrees)   /  a ‘-’ sign denotes south and a ‘+’ sign denotes north
float64 lon                      # Rover station longitude (-180 to 180 degrees)   /  a ‘-’ sign denotes west and a ‘+’ sign denotes east
float64 het                      # Rover station height

float64 ecef_x                   # ECEF X value (m)
float64 ecef_y                   # ECEF Y value (m)
float64 ecef_z                   # ECEF Z value (m)

float64 base_lat                 # Base station latitude (-90 to 90 degrees)
float64 base_lon                 # Base station longitude(-180 to 180 degrees)
float64 base_alt                 # Base station height

float64 sec_lat                 # Sub-antenna latitude (-90 to 90 degrees)
float64 sec_lon                 # Sub-antenna longitude(-180 to 180 degrees)
float64 sec_alt                 # Sub-antenna height
