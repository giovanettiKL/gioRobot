# Discrete motor command.
#
# One node (e.g. robot_action_node) publishes this on /motor_command;
# motor_controller_node subscribes and drives the motors accordingly,
# continuing until the next command or the watchdog timeout (no /cmd_vel
# message / no motor_command received within CMD_VEL_TIMEOUT stops the robot).

uint8 FORWARD=0
uint8 BACKWARD=1
uint8 LEFT=2
uint8 RIGHT=3
uint8 SET_SPEED=4
uint8 STOP=5

uint8 command

# Fraction of max speed, 0.0-1.0.
#   SET_SPEED              -> becomes the persisted default speed.
#   FORWARD/BACKWARD/LEFT/RIGHT, speed > 0 -> overrides (and persists as)
#     the default speed for this and subsequent commands.
#   FORWARD/BACKWARD/LEFT/RIGHT, speed == 0 -> use the current default speed.
#   STOP                    -> ignored.
float32 speed
