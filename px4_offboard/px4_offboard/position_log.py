import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import Twist, Vector3, Point
from math import pi
from std_msgs.msg import Bool

class PositionLog(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.curr_alt = 0.0
        self.master_true_yaw = 0.0

        #create gps subscription        
        self.global_position = self.create_subscription(
            VehicleGlobalPosition,
            '/px4_1/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            qos_profile
        )
        
        self.master_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/px4_1/fmu/out/vehicle_attitude',
            self.master_attitude_callback,
            qos_profile
        )

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)


    def vehicle_global_position_callback(self, msg):
        self.curr_lat = msg.lat
        self.curr_lon = msg.lon
        self.curr_alt = msg.alt
    
    #receives current trajectory values from drone and grabs the yaw value of the orientation
    # def master_attitude_callback(self, msg):
    #     orientation_q = msg.q

    #     #trueYaw is the drones current yaw value
    #     self.true_yaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
    #                           1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
    def master_attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.master_true_yaw = -(np.arctan2(2.0*(orientation_q[0]*orientation_q[3] + orientation_q[1]*orientation_q[2]), 
                            1.0 - 2.0*(orientation_q[2]*orientation_q[2] + orientation_q[3]*orientation_q[3])))
    
    


    def cmdloop_callback(self):
        self.get_logger().info(f"Latitude: {self.curr_lat}")
        self.get_logger().info(f"Longitude: {self.curr_lon}")
        self.get_logger().info(f"Altitude: {self.curr_alt}")
        self.get_logger().info(f"Yaw: {self.master_true_yaw}")

def main(args=None):
    rclpy.init(args=args)
    gps_position_log = PositionLog()

    rclpy.spin(gps_position_log)

    gps_position_log.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()