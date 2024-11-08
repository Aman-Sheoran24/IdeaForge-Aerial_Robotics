import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorCombinedSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_combined_subscriber')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  

        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.listener_callback,
            qos_profile)
        self.subscription  # Prevent unused variable warning
        self.count = 0

    def listener_callback(self, msg):
        thresh = 0.6
        timestamp_sec = msg.timestamp / 1_000_000.0  # Convert from microseconds to seconds
        gyro = {'x': msg.gyro_rad[0],'y': msg.gyro_rad[1],'z': msg.gyro_rad[2]}
        acc_z = msg.accelerometer_m_s2[2]
        if gyro['x'] > thresh and gyro['y'] < -thresh:
            new_msg = 1
        elif gyro['x'] < -thresh and gyro['y'] > thresh:
            new_msg = 2
        elif gyro['x'] < -thresh and gyro['y'] < -thresh:
            new_msg = 3
        elif gyro['x'] > thresh and gyro['y'] > thresh:
            new_msg = 4
        else:
            new_msg = 0
        if new_msg and self.count <3 and acc_z > -2:
            self.count+=1
            self.get_logger().info(f'Received sensor data:\n'
                                f' - Timestamp (s): {timestamp_sec}\n'
                                f' - gyro_rad: {msg.gyro_rad}\n'
                                f' - accelerometer_m_s2: {msg.accelerometer_m_s2}\n'
                                f' - gyro_integral_dt: {msg.gyro_integral_dt}\n'
                                f' - accelerometer_integral_dt: {msg.accelerometer_integral_dt}\n'
                                f' - fail_id: {new_msg}\n')
        
        
        

def main(args=None):
    rclpy.init(args=args)
    sensor_combined_subscriber = SensorCombinedSubscriber()
    rclpy.spin(sensor_combined_subscriber)
    sensor_combined_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
