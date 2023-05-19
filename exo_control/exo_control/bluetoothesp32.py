import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bleak import BleakClient

DEVICE_ADDRESS1 = ""
DEVICE_ADDRESS2 = ""

class BLEClientNode(Node):
    def __init__(self, device_addresses):
        super().__init__('ble_client_node')

        # Connect to multiple BLE devices
        self.ble_clients = []
        for addr in device_addresses:
            client = BleakClient(addr)
            client.connect()
            self.ble_clients.append(client)

        # Setup ROS2 publishers and subscribers
        self.publishers = [self.create_publisher(String, f'sensor_position_{i}', 10) for i in range(len(device_addresses))]

        # Assuming you want to control each device separately
        self.subscriptions = [self.create_subscription(String, f'velocity_command_{i}', self.velocity_callback, 10) for i in range(len(device_addresses))]

    def on_sensor_data_received(self, data, device_index):
        # Publish sensor data received from BLE device to ROS2 topic
        ros_msg = String()
        ros_msg.data = data
        self.publishers[device_index].publish(ros_msg)

    def velocity_callback(self, msg):
        # Send velocity command received from ROS2 topic to BLE device
        # Assuming msg.data is a string that contains the device index and the command, separated by a comma
        device_index, command = map(int, msg.data.split(','))
        self.ble_clients[device_index].write_gatt_char('your-characteristic-uuid', command)

    def on_shutdown(self):
        for client in self.ble_clients:
            if client.is_connected:
                client.disconnect()

def main():
    device_addresses = ['device_address_1', 'device_address_2']  # replace with your devices' addresses
    rclpy.init()
    ble_client_node = BLEClientNode(device_addresses)
    rclpy.on_shutdown(ble_client_node.on_shutdown)
    rclpy.spin(ble_client_node)

    ble_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
