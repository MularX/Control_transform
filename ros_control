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
    "PRZOD LEWE": 5,    # PULSE_PIN dla Silnika FL (Przedni Lewy)
    "PRZOD PRAWE": 22,  # PULSE_PIN dla Silnika FR (Przedni Prawy)
    "TYL LEWE": 25,     # PULSE_PIN dla Silnika RL (Tylny Lewy)
    "TYL PRAWE": 20     # PULSE_PIN dla Silnika RR (Tylny Prawy)
}

ENCODER_PIN_TO_MOTOR_ID = {
    ENCODER_PINS["PRZOD LEWE"]: 1,
    ENCODER_PINS["PRZOD PRAWE"]: 2,
    ENCODER_PINS["TYL LEWE"]: 3,
    ENCODER_PINS["TYL PRAWE"]: 4
}

encoder_pulse_counts = {name: 0 for name in ENCODER_PINS.keys()}

# Nowe zmienne do przechowywania średnich impulsów
average_right_wheels = 0
average_left_wheels = 0

# Zmienne do odometrii
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0
prev_right_pulses = 0.0
prev_left_pulses = 0.0

# Stałe do odometrii
WHEEL_RADIUS = 0.0405  # m
WHEEL_BASE = 0.23      # m
TICKS_PER_REVOLUTION = 270
DISTANCE_PER_TICK = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION

motor_directions = {
    1: True, # PRZOD LEWE
    2: True, # PRZOD PRAWE
    3: True, # TYL LEWE
    4: True  # TYL PRAWE
}

current_robot_mode = "STOPPED"

encoder_locks = {name: threading.Lock() for name in ENCODER_PINS.keys()}

# --- USTAWIENIA PROGRAMU ---
DISPLAY_REFRESH_RATE = 0.05 
JOINT_STATE_PUBLISH_RATE = 0.1  # Publish joint states at 10 Hz

# --- Funkcja obsługi przerwania dla Enkodera ---
def encoder_callback(channel):
    motor_id = ENCODER_PIN_TO_MOTOR_ID.get(channel)
    if motor_id is not None:
        motor_name = next(name for name, pin in ENCODER_PINS.items() if pin == channel)
        
        with encoder_locks[motor_name]: 
            is_left_wheel = "LEWE" in motor_name
            is_right_wheel = "PRAWE" in motor_name

            if current_robot_mode == "FORWARD":
                encoder_pulse_counts[motor_name] += 1
            elif current_robot_mode == "BACKWARD":
                encoder_pulse_counts[motor_name] -= 1
            elif current_robot_mode == "TURN_LEFT":
                if is_right_wheel:
                    encoder_pulse_counts[motor_name] += 1
                elif is_left_wheel:
                    encoder_pulse_counts[motor_name] -= 1
            elif current_robot_mode == "TURN_RIGHT":
                if is_left_wheel:
                    encoder_pulse_counts[motor_name] += 1
                elif is_right_wheel:
                    encoder_pulse_counts[motor_name] -= 1
            else:
                if motor_directions[motor_id]:
                    encoder_pulse_counts[motor_name] += 1
                else:
                    encoder_pulse_counts[motor_name] -= 1

class ROSNode(Node):
    def __init__(self):
        super().__init__('ros_node')
        self.Vel_X = 0.0
        self.Vel_Y = 0.0
        self.Ang_Z = 0.0
        self.last_message_time = None  

        self.subscriber_ = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10
        )

        self.odom_publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
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
            2: {'PWM': 23, 'DIR': 24}, 
            3: {'PWM': 12, 'DIR': 6},  
            4: {'PWM': 21, 'DIR': 16}, 
        }
        GPIO.setmode(GPIO.BCM)
        self.pwm_instances = {}
        for motor_id, pins in self.MOTOR_PINS.items():
            GPIO.setup(pins['PWM'], GPIO.OUT)
            GPIO.setup(pins['DIR'], GPIO.OUT)
            pwm = GPIO.PWM(pins['PWM'], 10000)  
            pwm.start(0) 
            self.pwm_instances[motor_id] = pwm

        for motor_name, pulse_pin in ENCODER_PINS.items():
            GPIO.setup(pulse_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
            GPIO.add_event_detect(pulse_pin, GPIO.RISING, callback=encoder_callback, bouncetime=1) 
            self.get_logger().info(f"Enkoder '{motor_name}' na pinie BCM {pulse_pin} skonfigurowany.")

    def clamp_velocity(self, value, min_val=0.0, max_val=1.0):
        return max(min_val, min(max_val, value))

    def normalize_velocities(self):
        if self.Vel_X > 1:
            self.Vel_X = 1
        if self.Vel_X < -1:
            self.Vel_X = -1
        if self.Ang_Z > 1:
            self.Ang_Z = 1
        if self.Ang_Z < -1:
            self.Ang_Z = -1

    def callback(self, msg):
        self.Vel_X = msg.linear.x
        self.Vel_Y = msg.linear.y
        self.Ang_Z = msg.angular.z
        self.normalize_velocities()  
        self.last_message_time = self.get_clock().now().nanoseconds / 1e9  
        self.update_motors()  

    def check_timeout(self):
        if self.last_message_time is None:
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_message_time > 1.0:
            self.stop_all()
            self.get_logger().info("No message received for 1 second, stopping motors.")
            self.last_message_time = None 

    def set_motor(self, motor_id, direction: bool, speed_percent: float):
        global motor_directions
        motor_directions[motor_id] = direction
        GPIO.output(self.MOTOR_PINS[motor_id]['DIR'], GPIO.HIGH if direction else GPIO.LOW)
        self.pwm_instances[motor_id].ChangeDutyCycle(speed_percent)

    def stop_all(self):
        global current_robot_mode
        current_robot_mode = "STOPPED"
        for motor_id in self.MOTOR_PINS:
            self.pwm_instances[motor_id].ChangeDutyCycle(100)

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
        speed = speed + (100-speed)/2
        self.get_logger().info(f"speed '{speed}'")
        self.set_motor(1, True, speed)
        self.set_motor(2, True, speed)
        self.set_motor(3, True, speed)
        self.set_motor(4, True, speed)

    def turn_right(self, speed):
        global current_robot_mode
        current_robot_mode = "TURN_RIGHT"
        speed = speed + (100-speed)/2
        self.get_logger().info(f"speed '{speed}'")
        self.set_motor(1, False, speed)
        self.set_motor(2, False, speed)
        self.set_motor(3, False, speed)
        self.set_motor(4, False, speed)

    def arc_NE(self, speed):
        global current_robot_mode
        current_robot_mode = "ARC"
        self.set_motor(1, True, 0)
        self.set_motor(2, False, speed)
        self.set_motor(3, False, speed)
        self.set_motor(4, True, 0)

    def arc_NW(self, speed):
        global current_robot_mode
        current_robot_mode = "ARC"
        self.set_motor(1, True, speed)
        self.set_motor(2, False, 0)
        self.set_motor(3, False, 0)
        self.set_motor(4, True, speed)

    def arc_SW(self, speed):
        global current_robot_mode
        current_robot_mode = "ARC"
        self.set_motor(1, False, speed)
        self.set_motor(2, True, 0)
        self.set_motor(3, True, 0)
        self.set_motor(4, False, speed)

    def arc_SE(self, speed):
        global current_robot_mode
        current_robot_mode = "ARC"
        self.set_motor(1, False, 0)
        self.set_motor(2, True, speed)
        self.set_motor(3, True, speed)
        self.set_motor(4, False, 0)

    def update_motors(self):
        self.get_logger().info(
            f"Vel_X: {self.Vel_X:.2f}, Vel_Y: {self.Vel_Y:.2f}, Ang_Z: {self.Ang_Z:.2f}"
        )
        speed = -100 * self.Vel_X + 100  
        angular_speed = 100 

        if self.Vel_X > 0:
            speed = -100 * self.Vel_X + 100
            self.move_forward(speed)
        elif self.Vel_X < 0:
            speed = 100 * self.Vel_X + 100
            self.move_backward(speed)
        elif self.Ang_Z > 0:
            angular_speed = -100 * self.Ang_Z + 100 
            self.turn_left(angular_speed)
        elif self.Ang_Z < 0:    
            angular_speed = 100 * self.Ang_Z + 100 
            self.turn_right(angular_speed)
        else:
            self.stop_all()

    def navigate_to_goal(self):
        pass

    def display_encoder_counts(self):
        global average_right_wheels, average_left_wheels, pose_x, pose_y, pose_theta
        global prev_right_pulses, prev_left_pulses

        # Obliczanie średnich dla prawych i lewych kół
        with encoder_locks["PRZOD PRAWE"], encoder_locks["TYL PRAWE"], encoder_locks["PRZOD LEWE"], encoder_locks["TYL LEWE"]:
            average_right_wheels = (encoder_pulse_counts["PRZOD PRAWE"] + encoder_pulse_counts["TYL PRAWE"]) / 2
            average_left_wheels = (encoder_pulse_counts["PRZOD LEWE"] + encoder_pulse_counts["TYL LEWE"]) / 2

            # Obliczanie odometrii
            right_distance = average_right_wheels * DISTANCE_PER_TICK
            left_distance = average_left_wheels * DISTANCE_PER_TICK
            delta_right = (average_right_wheels - prev_right_pulses) * DISTANCE_PER_TICK
            delta_left = (average_left_wheels - prev_left_pulses) * DISTANCE_PER_TICK

            # Kinematyka różnicowa
            d_center = (delta_right + delta_left) / 2
            delta_theta = (delta_right - delta_left) / WHEEL_BASE

            # Aktualizacja pozycji
            pose_x += d_center * math.cos(pose_theta)
            pose_y += d_center * math.sin(pose_theta)
            pose_theta += delta_theta

            # Obliczanie prędkości
            linear_vel = d_center / DISPLAY_REFRESH_RATE
            angular_vel = delta_theta / DISPLAY_REFRESH_RATE

            # Aktualizacja poprzednich wartości enkoderów
            prev_right_pulses = average_right_wheels
            prev_left_pulses = average_left_wheels

        # Obliczanie kwaternionu dla orientacji
        qw = math.cos(pose_theta / 2)
        qz = math.sin(pose_theta / 2)

        # Publikowanie transformacji TF
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

        # Tworzenie wiadomości Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Wypełnianie pozycji
        odom_msg.pose.pose.position.x = pose_x
        odom_msg.pose.pose.position.y = pose_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Wypełnianie orientacji (kwaternion)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Wypełnianie prędkości
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel

        # Publikowanie wiadomości
        self.odom_publisher_.publish(odom_msg)

    def publish_joint_states(self):
        with encoder_locks["PRZOD PRAWE"], encoder_locks["TYL PRAWE"], encoder_locks["PRZOD LEWE"], encoder_locks["TYL LEWE"]:
            # Calculate pulse counts for each wheel
            pulses_fl = encoder_pulse_counts["PRZOD LEWE"]
            pulses_fr = encoder_pulse_counts["PRZOD PRAWE"]
            pulses_rl = encoder_pulse_counts["TYL LEWE"]
            pulses_rr = encoder_pulse_counts["TYL PRAWE"]
        
        # Convert pulse counts to angular positions in radians
        angle_per_tick = 2 * math.pi / TICKS_PER_REVOLUTION
        angle_fl = pulses_fl * angle_per_tick  # wheel_1 (PRZOD LEWE)
        angle_fr = pulses_fr * angle_per_tick  # wheel_2 (PRZOD PRAWE)
        angle_rl = pulses_rl * angle_per_tick  # wheel_3 (TYL LEWE)
        angle_rr = pulses_rr * angle_per_tick  # wheel_4 (TYL PRAWE)

        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']
        joint_state.position = [angle_fl, angle_fr, angle_rl, angle_rr]
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0, 0.0]
        
        self.joint_state_publisher.publish(joint_state)

def main():
    rclpy.init()
    ros_node = ROSNode()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        ros_node.get_logger().info("Shutting down node.")
    finally:
        ros_node.stop_all() 
        for pwm in ros_node.pwm_instances.values():
            pwm.stop() 
        GPIO.cleanup() 
        rclpy.shutdown() 
        print("\nPiny GPIO wyczyszczone. Program zakończony.")
        print("======================================================")

if __name__ == '__main__':
    main()
