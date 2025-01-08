import rclpy
from rclpy.node import Node
import math
import numpy as np
import asyncio
from functools import partial
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleGlobalPosition
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Bool


class SlaveDroneControl(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        #!!!!!!!!!!!!! check and set num of drones!!!!!!!!!!!!!
        self.num_of_drones = 3
        self.target_distance = 7.0
        self.min_distance = 2.9
        self.max_distance = 3.1

        self.slave_status_subscribers = {}
        self.slave_global_position_subscribers = {}
        self.slave_trajectory_publisher = {}

        self.master_curr_lat = 0.0
        self.master_curr_lon = 0.0
        self.master_curr_alt = 0.0

        self.slave_nav_state = {}
        self.slave_arm_state = {}
        self.slave_failsafe = {}
        self.slave_flightCheck = {}
        self.slave_global_position = {}
        self.slave_relative_position = {
            2: {"x_offset": -1, "y_offset": -1},  # Slave 1 position
            3: {"x_offset": 1, "y_offset": 1},  # Slave 2 position
        }

        for i in range(2, self.num_of_drones + 1):
            self.slave_nav_state[i] = VehicleStatus.NAVIGATION_STATE_MAX
            self.slave_arm_state[i] = VehicleStatus.ARMING_STATE_DISARMED
            self.slave_failsafe[i] = False
            self.slave_flightCheck[i] = False
            self.slave_global_position[i] = {
                "curr_lat": 0.0,
                "curr_lon": 0.0,
                "curr_alt": 0.0,
            }

        # create master drone subscriptions
        self.master_global_position = self.create_subscription(
            VehicleGlobalPosition,
            "/px4_1/fmu/out/vehicle_global_position",
            self.master_vehicle_global_position_callback,
            qos_profile,
        )
        self.master_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/px4_1/fmu/out/vehicle_attitude',
            self.master_attitude_callback,
            qos_profile
        )

        # create slave drones subscriptions
        for i in range(2, self.num_of_drones + 1):
            slave_topic_status = f"/px4_{i}/fmu/out/vehicle_status"
            slave_topic_global_position = f"/px4_{i}/fmu/out/vehicle_global_position"

            self.slave_status_subscribers[i] = self.create_subscription(
                VehicleStatus,
                slave_topic_status,
                partial(self.slave_vehicle_status_callback, slave_id=i),
                qos_profile,
            )

            self.slave_global_position_subscribers[i] = self.create_subscription(
                VehicleGlobalPosition,
                slave_topic_global_position,
                partial(self.slave_vehicle_global_position_callback, slave_id=i),
                qos_profile,
            )

        # create slave drones publishers
        for i in range(2, self.num_of_drones + 1):
            slave_topic_tarajectory_publish = f"/px4_{i}/fmu/in/trajectory_setpoint"

            self.slave_trajectory_publisher[i] = self.create_publisher(
                TrajectorySetpoint, slave_topic_tarajectory_publish, qos_profile
            )

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def master_vehicle_global_position_callback(self, msg):
        self.master_curr_lat = msg.lat
        self.master_curr_lon = msg.lon
        self.master_curr_alt = msg.alt
    
    def master_attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.master_true_yaw = -(np.arctan2(2.0*(orientation_q[0]*orientation_q[3] + orientation_q[1]*orientation_q[2]), 
                            1.0 - 2.0*(orientation_q[2]*orientation_q[2] + orientation_q[3]*orientation_q[3])))
    

    def slave_vehicle_status_callback(self, msg, slave_id: int):
        self.slave_nav_state[slave_id] = msg.nav_state
        self.slave_arm_state[slave_id] = msg.arming_state
        self.slave_failsafe[slave_id] = msg.failsafe
        self.slave_flightCheck[slave_id] = msg.pre_flight_checks_pass

        # if (msg.nav_state != self.nav_state):
        #     self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        # if (msg.arming_state != self.arm_state):
        #     self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        # if (msg.failsafe != self.failsafe):
        #     self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        # if (msg.pre_flight_checks_pass != self.flightCheck):
        #     self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

    def slave_vehicle_global_position_callback(self, msg, slave_id: int):
        # Update slave drone global position
        if slave_id in self.slave_global_position:
            self.slave_global_position[slave_id] = {
                "curr_lat": msg.lat,
                "curr_lon": msg.lon,
                "curr_alt": msg.alt,
            }

    # slave drone control
    def slave_drone_control(self, slave_id):
        master_lat = self.master_curr_lat
        master_lon = self.master_curr_lon
        master_alt = self.master_curr_alt
        master_yaw = self.master_true_yaw 
        
        slave_lat = self.slave_global_position[slave_id]["curr_lat"]
        slave_lon = self.slave_global_position[slave_id]["curr_lon"]
        slave_alt = self.slave_global_position[slave_id]["curr_alt"]
        
        slave_relative_x = (master_lat - slave_lat) * 111000
        slave_relative_y = (master_lon - slave_lon) * 111000 * math.cos(math.radians(slave_lat))

        slave_target_relative_x = self.target_distance * math.cos(master_yaw) * self.slave_relative_position[slave_id]["x_offset"]
        slave_target_relative_y = self.target_distance * math.sin(master_yaw) * self.slave_relative_position[slave_id]["y_offset"]
        self.get_logger().info(f"target_x: {slave_target_relative_x}")
        self.get_logger().info(f"target_y: {slave_target_relative_y}")
        
        slave_x_error = slave_target_relative_x - slave_relative_x
        slave_y_error = slave_target_relative_y - slave_relative_y
        slave_error_distance = (slave_x_error**2 + slave_y_error**2)**(1/2)
        # self.get_logger().info(f"error_distance: {slave_error_distance}")
        alt_error = master_alt - slave_alt

        if abs(slave_x_error) <= 0.1:
            velocity_world_x = 0
        elif slave_x_error > 0.1:
            velocity_world_x = -0.8
        else:
            velocity_world_x = 0.8
            
        if abs(slave_y_error) <= 0.1:
            velocity_world_y = 0
        elif slave_y_error > 0.1:
            velocity_world_y = -0.8
        else:
            velocity_world_y = 0.8

        if abs(alt_error) <= 0.05:
            velocity_world_z = 0
        elif alt_error > 0.05:
            velocity_world_z = -0.5
        else:
            velocity_world_z = 0.5

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = velocity_world_x
        trajectory_msg.velocity[1] = velocity_world_y
        trajectory_msg.velocity[2] = velocity_world_z
        trajectory_msg.position[0] = float("nan")
        trajectory_msg.position[1] = float("nan")
        trajectory_msg.position[2] = float("nan")
        trajectory_msg.acceleration[0] = float("nan")
        trajectory_msg.acceleration[1] = float("nan")
        trajectory_msg.acceleration[2] = float("nan")
        trajectory_msg.yaw = float("nan")

        self.slave_trajectory_publisher[slave_id].publish(trajectory_msg)
    

    def cmdloop_callback(self):
        for i in range(2, self.num_of_drones + 1):
            if self.slave_arm_state[i] == VehicleStatus.ARMING_STATE_ARMED:
                self.slave_drone_control(i)

def main(args=None):
    rclpy.init(args=args)

    slave_drone_control = SlaveDroneControl()

    rclpy.spin(slave_drone_control)

    slave_drone_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
