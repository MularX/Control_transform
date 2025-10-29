import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import RPi.GPIO as GPIO
import time
import threading
import math
from tf2_ros import TransformBroadcaster

# --- GLOBALNE ZMIENNE DLA ENKODERÓW ---
ENCODER_PINS = {
    "PRZOD LEWE": 5,    # PULSE_PIN dla Silnika FL
    "PRZOD PRAWE": 22,  # PULSE_PIN dla Silnika FR
    "TYL LEWE": 25,     # PULSE_PIN dla Silnika RL
    "TYL PRAWE": 20     # PULSE_PIN dla Silnika RR
}

ENCODER_PIN_TO_MOTOR_ID = {
    ENCODER_PINS["PRZOD LEWE"]: 3,
    ENCODER_PINS["PRZOD PRAWE"]: 1,
    ENCODER_PINS["TYL LEWE"]: 2,
    ENCODER_PINS["TYL PRAWE"]: 4
}

encoder_pulse_counts = {name: 0 for name in ENCODER_PINS.keys()}
average_right_wheels = 0
average_left_wheels = 0
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0
prev_pulses = {name: 0.0 for name in ENCODER_PINS.keys()}
WHEEL_RADIUS = 0.0405
WHEEL_BASE = 0.23
TICKS_PER_REVOLUTION = 270
DISTANCE_PER_TICK = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
SLIP_FACTOR = 0.9
motor_directions = {1: True, 2: True, 3: True, 4: True}
current_robot_mode = "STOPPED"
encoder_locks = {name: threading.Lock() for name in ENCODER_PINS.keys()}
DISPLAY_REFRESH_RATE = 0.0714  # ~14 Hz to match lidar
JOINT_STATE_PUBLISH_RATE = 0.1

# --- Funkcja obsługi przerwania dla Enkodera ---
def encoder_callback(channel):
    motor_id = ENCODER_PIN_TO_MOTOR_ID.get(channel)
    if motor_id is not None:
        motor_name = next(name for name, pin in ENCODER_PINS.items() if pin == channel)
        with encoder_locks[motor_name]:
            if current_robot_mode == "FORWARD":
                encoder_pulse_counts[motor_name] += 1
            elif current_robot_mode == "BACKWARD":
                encoder_pulse_counts[motor_name] -= 1
            elif current_robot_mode == "TURN_LEFT":
                if "PRAWE" in motor_name:
                    encoder_pulse_counts[motor_name] += 1
                elif "LEWE" in motor_name:
                    encoder_pulse_counts[motor_name] -= 1
            elif current_robot_mode == "TURN_RIGHT":
                if "LEWE" in motor_name:
                    encoder_pulse_counts[motor_name] += 1
                elif "PRAWE" in motor_name:
                    encoder_pulse_counts[motor_name] -= 1
            else:
                encoder_pulse_counts[motor_name] += 1 if motor_directions[motor_id] else -1

class ROSNode(Node):
    def __init__(self):
        super().__init__('ros_node')
        self.Vel_X = 0.0
        self.Vel_Y = 0.0
        self.Ang_Z = 0.0
        self.last_message_time = None
        self.subscriber_ = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.odom_publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timeout_timer = self.create_timer(0.5, self.check_timeout)  # Increased to 0.5s
        self.display_timer = self.create_timer(DISPLAY_REFRESH_RATE, self.display_encoder_counts)
        self.joint_state_timer = self.create_timer(JOINT_STATE_PUBLISH_RATE, self.publish_joint_states)
        self.get_logger().info("======================================================")
        self.get_logger().info("  PROGRAM DO STEROWANIA I ZCZYTYWANIA Z ENKODERÓW   ")
        self.get_logger().info("  (Z filtrem bouncetime=1ms)                          ")
        self.get_logger().info("======================================================")
        self.get_logger().info("  Naciśnij CTRL+C, aby zakończyć działanie programu. ")
        self.get_logger().info("------------------------------------------------------")
        self.MOTOR_PINS = {
            1: {'PWM': 18, 'DIR': 17},
            2: {'PWM': 13, 'DIR': 24},
            3: {'PWM': 12, 'DIR': 6},
            4: {'PWM': 21, 'DIR': 16},
        }
        GPIO.setmode(GPIO.BCM)
        self.pwm_instances = {}
        for motor_id, pins in self.MOTOR_PINS.items():
            GPIO.setup(pins['PWM'], GPIO.OUT)
            GPIO.setup(pins['DIR'], GPIO.OUT)
            pwm = GPIO.PWM(pins['PWM'], 10000)
            pwm.start(100)  # Start stopped (100% duty = stop)
            self.pwm_instances[motor_id] = pwm
            self.get_logger().info(f"Motor {motor_id} on PWM {pins['PWM']} and DIR {pins['DIR']} initialized.")
        for motor_name, pulse_pin in ENCODER_PINS.items():
            GPIO.setup(pulse_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(pulse_pin, GPIO.RISING, callback=encoder_callback, bouncetime=1)
            self.get_logger().info(f"Enkoder '{motor_name}' na pinie BCM {pulse_pin} skonfigurowany.")

    def callback(self, msg):
        self.Vel_X = msg.linear.x
        self.Vel_Y = msg.linear.y
        self.Ang_Z = msg.angular.z
        self.last_message_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Received cmd_vel: Vel_X={self.Vel_X:.2f}, Ang_Z={self.Ang_Z:.2f}")
        self.update_motors()

    def check_timeout(self):
        if self.last_message_time is None:
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_message_time > 0.5:  # Trigger stop after 0.5s of no commands
            self.stop_all()
            self.get_logger().info("Timeout: No message received, stopping motors.")
            self.last_message_time = None

    def set_motor(self, motor_id, direction, speed_percent):
        global motor_directions
        motor_directions[motor_id] = direction
        GPIO.output(self.MOTOR_PINS[motor_id]['DIR'], GPIO.HIGH if direction else GPIO.LOW)
        # Reversed logic: 0% = max speed (100% duty), 100% = stop (0% duty)
        duty_cycle = 100 - max(0, min(100, speed_percent))  # Clamp and invert
        self.pwm_instances[motor_id].ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f"Setting motor {motor_id} to duty cycle {duty_cycle}%")

    def stop_all(self):
        global current_robot_mode
        current_robot_mode = "STOPPED"
        for motor_id in self.MOTOR_PINS:
            self.set_motor(motor_id, motor_directions[motor_id], 100)  # 100% = stop
        self.get_logger().info("All motors stopped.")

    def move_forward(self, speed):
        global current_robot_mode
        current_robot_mode = "FORWARD"
        self.set_motor(1, True, speed)
        self.set_motor(2, False, speed)
        self.set_motor(3, False, speed)
        self.set_motor(4, True, speed)

    def move_backward(self, speed):
        global current_robot_mode
        current_robot_mode = "BACKWARD"
        self.set_motor(1, False, speed)
        self.set_motor(2, True, speed)
        self.set_motor(3, True, speed)
        self.set_motor(4, False, speed)

    def turn_left(self, speed):
        global current_robot_mode
        current_robot_mode = "TURN_LEFT"
        self.set_motor(1, True, speed)
        self.set_motor(2, True, speed)
        self.set_motor(3, True, speed)
        self.set_motor(4, True, speed)

    def turn_right(self, speed):
        global current_robot_mode
        current_robot_mode = "TURN_RIGHT"
        self.set_motor(1, False, speed)
        self.set_motor(2, False, speed)
        self.set_motor(3, False, speed)
        self.set_motor(4, False, speed)

    def update_motors(self):
        self.get_logger().info(
            f"Vel_X: {self.Vel_X:.2f}, Vel_Y: {self.Vel_Y:.2f}, Ang_Z: {self.Ang_Z:.2f}"
        )
        speed = -100 * self.Vel_X + 100  # Reversed logic adjustment
        if abs(self.Vel_X) > 0.01 or abs(self.Ang_Z) > 0.01:  # prog
            if self.Vel_X > 0 and abs(self.Ang_Z) < 0.01:
                self.move_forward(speed)
            elif self.Vel_X < 0 and abs(self.Ang_Z) < 0.01:
                self.move_backward(speed)
            elif self.Ang_Z > 0 and abs(self.Vel_X) < 0.01:
                self.turn_left(speed)
            elif self.Ang_Z < 0 and abs(self.Vel_X) < 0.01:
                self.turn_right(speed)
        else:
            self.stop_all()

    def display_encoder_counts(self):
        global average_right_wheels, average_left_wheels, pose_x, pose_y, pose_theta
        global prev_pulses
        PULSE_THRESHOLD = 50

        with encoder_locks["PRZOD PRAWE"], encoder_locks["TYL PRAWE"], encoder_locks["PRZOD LEWE"], encoder_locks["TYL LEWE"]:
            diff_fl = encoder_pulse_counts["PRZOD LEWE"] - prev_pulses["PRZOD LEWE"]
            diff_fr = encoder_pulse_counts["PRZOD PRAWE"] - prev_pulses["PRZOD PRAWE"]
            diff_rl = encoder_pulse_counts["TYL LEWE"] - prev_pulses["TYL LEWE"]
            diff_rr = encoder_pulse_counts["TYL PRAWE"] - prev_pulses["TYL PRAWE"]

            right_diff = min(abs(diff_fr), abs(diff_rr))
            left_diff = min(abs(diff_fl), abs(diff_rl))
            
            right_pulses = diff_fr if abs(diff_fr) == right_diff else diff_rr
            left_pulses = diff_fl if abs(diff_fl) == left_diff else diff_rl

            delta_right = right_pulses * DISTANCE_PER_TICK
            delta_left = left_pulses * DISTANCE_PER_TICK

            if current_robot_mode == "TURN_LEFT":
                delta_left = -abs(left_pulses) * DISTANCE_PER_TICK
                delta_right = abs(right_pulses) * DISTANCE_PER_TICK
            elif current_robot_mode == "TURN_RIGHT":
                delta_left = abs(left_pulses) * DISTANCE_PER_TICK
                delta_right = -abs(right_pulses) * DISTANCE_PER_TICK

            right_slip = abs(diff_fr - diff_rr)
            left_slip = abs(diff_fl - diff_rl)
            if right_slip > PULSE_THRESHOLD or left_slip > PULSE_THRESHOLD:
                self.get_logger().warn(f"Slip detected: Right diff={right_slip}, Left diff={left_slip}")

            if current_robot_mode in ["TURN_LEFT", "TURN_RIGHT"]:
                d_center = 0.0
            else:
                d_center = (delta_right + delta_left) / 2

            delta_theta = (delta_right - delta_left) / WHEEL_BASE * SLIP_FACTOR

            pose_x += d_center * math.cos(pose_theta)
            pose_y += d_center * math.sin(pose_theta)
            pose_theta += delta_theta

            linear_vel = d_center / DISPLAY_REFRESH_RATE
            angular_vel = delta_theta / DISPLAY_REFRESH_RATE

            prev_pulses["PRZOD LEWE"] = encoder_pulse_counts["PRZOD LEWE"]
            prev_pulses["PRZOD PRAWE"] = encoder_pulse_counts["PRZOD PRAWE"]
            prev_pulses["TYL LEWE"] = encoder_pulse_counts["TYL LEWE"]
            prev_pulses["TYL PRAWE"] = encoder_pulse_counts["TYL PRAWE"]
            average_right_wheels = right_pulses
            average_left_wheels = left_pulses

            self.get_logger().info(
                f"Encoders: FL={encoder_pulse_counts['PRZOD LEWE']}, FR={encoder_pulse_counts['PRZOD PRAWE']}, "
                f"RL={encoder_pulse_counts['TYL LEWE']}, RR={encoder_pulse_counts['TYL PRAWE']}, "
                f"Delta Right={delta_right:.4f}, Delta Left={delta_left:.4f}, "
                f"D Center={d_center:.4f}, Delta Theta={delta_theta:.4f}, Pose Theta={pose_theta:.4f}"
            )

            qw = math.cos(pose_theta / 2)
            qz = math.sin(pose_theta / 2)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = pose_x
            t.transform.translation.y = pose_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = pose_x
            odom_msg.pose.pose.position.y = pose_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = angular_vel
            self.odom_publisher_.publish(odom_msg)

    def publish_joint_states(self):
        with encoder_locks["PRZOD PRAWE"], encoder_locks["TYL PRAWE"], encoder_locks["PRZOD LEWE"], encoder_locks["TYL LEWE"]:
            diff_fl = encoder_pulse_counts["PRZOD LEWE"] - prev_pulses["PRZOD LEWE"]
            diff_fr = encoder_pulse_counts["PRZOD PRAWE"] - prev_pulses["PRZOD PRAWE"]
            diff_rl = encoder_pulse_counts["TYL LEWE"] - prev_pulses["TYL LEWE"]
            diff_rr = encoder_pulse_counts["TYL PRAWE"] - prev_pulses["TYL PRAWE"]

            pulses_fl = encoder_pulse_counts["PRZOD LEWE"]
            pulses_fr = encoder_pulse_counts["PRZOD PRAWE"]
            pulses_rl = encoder_pulse_counts["TYL LEWE"]
            pulses_rr = encoder_pulse_counts["TYL PRAWE"]

            angle_per_tick = 2 * math.pi / TICKS_PER_REVOLUTION
            angle_fl = pulses_fl * angle_per_tick
            angle_fr = pulses_fr * angle_per_tick
            angle_rl = pulses_rl * angle_per_tick
            angle_rr = pulses_rr * angle_per_tick

            vel_fl = (diff_fl * angle_per_tick) / JOINT_STATE_PUBLISH_RATE
            vel_fr = (diff_fr * angle_per_tick) / JOINT_STATE_PUBLISH_RATE
            vel_rl = (diff_rl * angle_per_tick) / JOINT_STATE_PUBLISH_RATE
            vel_rr = (diff_rr * angle_per_tick) / JOINT_STATE_PUBLISH_RATE

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']
            joint_state.position = [angle_fr, angle_rl, angle_fl, angle_rr]
            joint_state.velocity = [vel_fr, vel_rl, vel_fl, vel_rr]
            joint_state.effort = [0.0, 0.0, 0.0, 0.0]
            
            self.joint_state_publisher.publish(joint_state)

def main():
    rclpy.init()
    ros_node = ROSNode()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        ros_node.get_logger().info("Shutting down node due to KeyboardInterrupt.")
        ros_node.stop_all()
    except Exception as e:
        ros_node.get_logger().error(f"Unexpected error during execution: {e}")
        ros_node.stop_all()
    finally:
        for pwm in ros_node.pwm_instances.values():
            try:
                pwm.stop()
            except Exception as e:
                ros_node.get_logger().error(f"Failed to stop PWM: {e}")
        GPIO.cleanup()
        rclpy.shutdown()
        print("\nPiny GPIO wyczyszczone. Program zakończony.")
        print("======================================================")

if __name__ == '__main__':
    main()