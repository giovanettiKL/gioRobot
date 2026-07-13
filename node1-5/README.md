# pi5_motor_control — ROS2 Humble

Differential drive motor controller for **Raspberry Pi 5** using `lgpio`.
Subscribes to `/cmd_vel` (`geometry_msgs/Twist`) and drives two DC motors via
a PWM H-bridge driver (L298N, DRV8833, TB6612FNG, etc.).

> **Why not RPi.GPIO?** The Pi 5 moved GPIO handling to a separate RP1 chip
> instead of memory-mapping it directly on the SoC. Classic `RPi.GPIO` (and
> `pigpio`) don't support this and fail on Pi 5. `lgpio` is the
> Raspberry-Pi-maintained library that talks to RP1 correctly.

---

## Package layout

```
pi5_motor_control/
├── config/
│   └── motor_params.yaml          # All tunable parameters
├── launch/
│   └── motor_controller.launch.py
├── pi5_motor_control/
│   ├── __init__.py
│   └── motor_controller_node.py   # Main node
├── package.xml
└── setup.py
```

---

## Hardware wiring (L298N example — BCM pin numbers)

| L298N pin | Pi5 GPIO (BCM) | Function           |
|-----------|----------------|--------------------|
| ENA       | 12             | Left motor PWM     |
| IN1       | 20             | Left motor dir A   |
| IN2       | 21             | Left motor dir B   |
| ENB       | 13             | Right motor PWM    |
| IN3       | 19             | Right motor dir A  |
| IN4       | 26             | Right motor dir B  |

> **Note:** PWM here is generated in software by `lgpio.tx_pwm()` (not the
> SoC's hardware PWM peripheral), so any GPIO can be used — there's no
> hardware-PWM-only pin restriction like on the Pi 4.

---

## Installation

### 1 — System dependencies
```bash
sudo apt update
sudo apt install python3-lgpio ros-humble-desktop
```

### 2 — Create / source your workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy this package folder here:
cp -r /path/to/pi5_motor_control .
```

### 3 — Build
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pi5_motor_control
source install/setup.bash
```

---

## Running

### Launch (recommended)
```bash
ros2 launch pi5_motor_control motor_controller.launch.py
```

### With a custom parameter file
```bash
ros2 launch pi5_motor_control motor_controller.launch.py \
  params_file:=/path/to/my_params.yaml
```

### Direct node run
```bash
ros2 run pi5_motor_control motor_controller \
  --ros-args --params-file config/motor_params.yaml
```

---

## Testing without hardware

The node detects a missing `lgpio` import automatically and enters
**SIMULATION mode** — all GPIO calls are replaced by debug log messages. You
can run and test on any Linux machine without a Pi.

```bash
# Send a drive-forward command from another terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10

# Spin in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}" --rate 10

# Monitor duty cycles
ros2 topic echo /motor_duty
```

---

## Key parameters (`config/motor_params.yaml`)

| Parameter          | Default | Description                              |
|--------------------|---------|-------------------------------------------|
| `wheel_base`       | 0.20 m  | Distance between wheel centres           |
| `max_linear_speed` | 0.5 m/s | Clamps `cmd_vel` linear.x                |
| `max_angular_speed`| 2.0 rad/s | Clamps `cmd_vel` angular.z              |
| `pwm_frequency`    | 1000 Hz | PWM carrier frequency                    |
| `gpiochip`         | 4       | RP1 gpiochip number (try 0 if 4 fails)    |
| `pin_ena/enb`      | 12/13   | BCM PWM enable pins                      |
| `pin_in1..in4`     | 20/21/19/26 | BCM direction pins                   |

---

## Watchdog

If no `/cmd_vel` message is received within **0.5 s**, both motors are stopped
automatically. Adjust `CMD_VEL_TIMEOUT` in the node source if needed.

---

## Troubleshooting

- **`gpiochip_open` fails / "GPIO busy"**: try `gpiochip: 0` instead of `4` in
  the params file — the RP1 chip number has moved between OS image versions.
- **Permission denied opening gpiochip**: add your user to the `gpio` group
  (`sudo usermod -aG gpio $USER`, then log out/in), or check `/dev/gpiochip*`
  permissions.

---

## Extending this package

- **Encoder odometry**: add a `JointState` or `Odometry` publisher reading
  GPIO edge events via `lgpio`'s alert/callback functions.
- **PID speed control**: replace the open-loop duty mapping with a PID loop
  tuned against encoder feedback.
- **Nav2 integration**: this node is drop-in compatible with Nav2's
  `DifferentialDrive` controller — just point `cmd_vel` correctly in your
  Nav2 params.
