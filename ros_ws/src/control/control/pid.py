#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        
        self.Kp = 1.0  #modify the value to get the desired result for kp, ki, kd
        self.Ki = 0.05  
        self.Kd = 0.01  
        self.setpoint = 0.0  
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.base_speed = 100.0  # Base speed for the motors

        self.sub_imu = self.create_subscription(Imu, "imu_data", self.imu_callback, 10)

        self.motor_left_pub = self.create_publisher(Float32, 'motor_left_speed_pid', 10)
        self.motor_right_pub = self.create_publisher(Float32, 'motor_right_speed_pid', 10)

        self.current_angle = 0.0

    def imu_callback(self, data):
        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z

        # Calculate the pitch angle using the accelerometer data
        pitch = self.calculate_pitch(ax, ay, az)

        # Update the current angle value
        self.current_angle = pitch     


    def calculate_pitch(self, ax, ay, az):

        # Calculate the pitch angle using the accelerometer data
        pitch = math.atan2(ay, az)
        return pitch * 180.0 / 3.14


    def compute_pid(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time

        # PID error calculations
        error = self.setpoint - self.current_angle  
        self.integral += error * dt
        if dt > 0:
            derivative = (error - self.last_error) / dt 
        else:
            derivative = 0.0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.last_error = error
        self.last_time = current_time

        return output

    def control_motors(self):
        pid_output = self.compute_pid()

        # Adjust motor speeds based on PID output
        left_motor_speed = self.base_speed - pid_output 
        right_motor_speed = self.base_speed + pid_output

        # Limit motor speeds to the range [-255, 255]
        left_motor_speed = max(min(left_motor_speed, 255), -255)
        right_motor_speed = max(min(right_motor_speed, 255), -255)

        # Publish the motor speeds
        self.motor_left_pub.publish(Float32(data=left_motor_speed))
        self.motor_right_pub.publish(Float32(data=right_motor_speed))

    def run(self):
        rate = self.create_rate(50)  # Set the loop rate to 50 Hz
        while rclpy.ok():
            self.control_motors()  # Control the motors
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    pid_controller.run()  # Start the control loop
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
