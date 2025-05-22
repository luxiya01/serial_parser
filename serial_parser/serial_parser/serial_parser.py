import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SerialParser(Node):
    """
    A ROS2 node that reads data from a serial port and publishes it to a topic.
    """

    def __init__(self):
        super().__init__("serial_parser")
        self.get_logger().info("Setting up serial parser")

        # Default parameters, can be overridden by command line arguments or launch files
        self.declare_parameter("port", "/dev/pts/4")#"/dev/ttyUSB1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("topic_name", "serial_data")
        self.declare_parameter("poll_rate", 10) # Hz
        self.declare_parameter("send_command", "stdo,19\r\n")

        # Retrieve parameters
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.send_command = self.get_parameter("send_command").get_parameter_value().string_value
        self.poll_rate = self.get_parameter("poll_rate").get_parameter_value().integer_value

        # Start serial reader, block until connected
        self.start_serial_reader()
        # Set up publisher and timer for reading and publishing data
        self.publisher = self.create_publisher(String, self.topic_name, 10)
        self.create_timer(1.0 / self.poll_rate, self.read_serial_and_publish)

    def start_serial_reader(self):
        """
        Start the serial reader in a separate thread.
        This method will keep trying to connect until successful or the node is shut down.
        """
        self.get_logger().info(f"Starting serial reader. Block until connected.")
        self.get_logger().info(f"Using port: {self.port}, baudrate: {self.baudrate}")
        self.get_logger().info(f"Publishing to topic: {self.topic_name}")
        self._connect_serial()

    def _connect_serial(self):
        """
        Connect to the serial port if not already connected.
        This method will keep trying to connect until successful or the node is shut down.
        Each attempt will double the sleep time until a maximum of 30 seconds.
        """
        self.connected = False
        sleep_time = .5
        while not self.connected and rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                self.connected = True
            except serial.SerialException as e:
                sleep_time = min(sleep_time * 2, 30)  # Exponential backoff but cap at 30 seconds
                self.get_logger().info(f"Error opening serial port: {e}. Trying again in {sleep_time} seconds.")
                time.sleep(sleep_time)

    def read_serial_and_publish(self):
        """
        Read a line from the serial port and publish it.
        Loop until the node is shut down or the serial port is closed.
        """
        if self.connected and self.ser.is_open:
            if self.send_command:
                self.ser.write(self.send_command.encode("utf-8"))
                self.get_logger().info(f"Sent command: {self.send_command}")

            line = self.ser.readline().decode("utf-8").strip()
            if len(line) > 0:
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
                self.get_logger().info(f"Publishing: {msg.data}")
        else:
            self.get_logger().warn("Serial port is not open or connected. Cannot read data.")


def main(args=None):
    rclpy.init(args=args)
    node = SerialParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down serial reader")
    finally:
        node.ser.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
