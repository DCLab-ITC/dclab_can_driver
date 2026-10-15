# Motors of a DCLab Smart Driver or Motor Controller on one receive ID
# (topic /dclab/motor_command; firmware Smart Driver 2.13, Motor Controller
# 2.22 and later). M1/M2 = the first/second motor on can_id (lower-numbered
# first). M1 only -> DLC 4; M2 enabled -> DLC 8 (M1's half disabled when
# m1_enable is false). A motor alone on its ID is always M1.
# A disabled motor keeps its command; it is not a stop (send voltage 0).
uint8 MODE_VOLTAGE=0         # goal in V (Motor Controller: voltage/current)
uint8 MODE_POSITION=1        # rad
uint8 MODE_SPEED=2           # rad/s
uint8 MODE_POSITION_RAMP=3   # goal = ramp rate, until the board restarts
uint8 MODE_SPEED_RAMP=4
uint8 MODE_VOLTAGE_RAMP=5

uint16 can_id
bool m1_enable
uint8 mode1
float32 goal1
bool m2_enable
uint8 mode2
float32 goal2
