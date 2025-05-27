# Serial Parser

This repository provides a ROS 2 node to interact with serial ports. It is designed to read data from a serial port, parse it, and publish structured messages in ROS 2. Additionally, it can listen to a specific ROS topic and write the data to the same serial port.

It contains a single ROS2 node: `serial_parser`. It takes the following ROS params:
- `port`: The serial port to read from (e.g., `/dev/ttyUSB0`).
- `baudrate`: The baud rate for the serial communication (default is `115200`).
- `listen_to_topic`: The ROS topic to listen to. When receiving a message on this topic, the node will write the data to the serial port.
- `publish_topic`: The ROS topic to publish the parsed data received from the serial port.
- `poll_rate`: The rate at which the node polls the serial port for new data (default is `10 Hz`).

## Dependencies

The driver has been tested with the following set up:
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
- Python 3.10.12
- [`pyserial`](https://github.com/pyserial/pyserial)

---

## Installation

Clone the repository into your ROS 2 workspace and build the packages:

```bash
cd ~/ros2_ws/src
git clone https://github.com/smarc-project/serial_parser.git
colcon build && source install/setup.bash
```
## Usage
### 1. Launch file
The easiest option to run the driver is using the launch file in `serial_parser` package:
```bash
ros2 launch serial_parser test.launch
```

### 2. Run the node manually
```bash
ros2 run serial_parser serial_parser --ros-args \
  -p port:="/dev/pts/4" \
  -p baudrate:=115200 \
  -p listen_to_topic:="serial_input" \
  -p publish_topic:="serial_output"
```

## Testing
The `serial_parser` node can be tested by simulating a virtual serial connection:
1. Start a virtual serial port using `socat`:
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
Note the output, which will show two virtual serial ports (e.g., `/dev/pts/3` and `/dev/pts/4`).

2. In another terminal, start the `serial_parser` node, listen to the second virtual port:
```bash
ros2 run serial_parser serial_parser --ros-args \
  -p port:="/dev/pts/4" \
  -p baudrate:=115200 \
  -p listen_to_topic:="serial_input" \
  -p publish_topic:="serial_output"
```

3. In another terminal, run `test/test_virtual_serial_connection.py` to send data to the first virtual port:
```bash
python3 test/test_virtual_serial_connection.py --port="/dev/pts/3" --baudrate=115200
```

4. You can send messages to the `listen_to_topic` by running:
```bash
ros2 topic pub /listen_to_topic std_msgs/msg/String "data: 'Hello, Serial!'"
```

5. You can check the messages published on the `publish_to_topic` by running:
```bash
ros2 topic echo /publish_to_topic
```