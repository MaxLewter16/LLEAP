"""
    Created to get motion points for cubic spline interpolation on a hip motor.
"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/left_leg_controller/state',
            self.listener_callback,
            10)
        self.log_file = open('logjointstates.txt', 'w')

    def listener_callback(self, msg):
        try:
            index = msg.joint_names.index('left_hip_revolute_joint')
            time_sec = msg.header.stamp.sec
            time_nanosec = msg.header.stamp.nanosec
            desired_position = msg.desired.positions[index]
            desired_velocity = msg.desired.velocities[index]
            desired_acceleration = msg.diresed.accelerations[index]
            actual_position = msg.actual.positions[index]
            actual_velocity = msg.actual.velocities[index]
            actual_acceleration = msg.actual.accelerations[index]
            error_position = msg.error.positions[index]
            error_velocity = msg.error.velocities[index]
            error_acceleration = msg.error.accelerations[index]
            log_message = f'Time: {time_sec}.{time_nanosec}, Desired Position: {desired_position}, Desired Velocity: {desired_velocity}, Desired Acceleration: {desired_acceleration}, Actual Position: {actual_position}, Actual Velocity: {actual_velocity}, Actual Acceleration: {actual_acceleration}, Error Position: {error_position}, Error Velocity: {error_velocity}, Error Acceleration: {error_acceleration}\n'
            self.log_file.write(log_message)
            self.log_file.flush()
        except ValueError:
            pass

    def close_file(self):
        if self.log_file:
            self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    joint_state_logger = JointStateLogger()
    rclpy.spin(joint_state_logger)

    joint_state_logger.close_file()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
