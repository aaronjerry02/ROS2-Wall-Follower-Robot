#!/usr/bin/env python

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wall_follower_interfaces.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup




class WallFinder(Node):
    def __init__(self):
        super().__init__('Wall_Finder_Node')
        
        self.reentrant_group_1 = ReentrantCallbackGroup()

        name_service = '/wall_finder_service'
        self.srv = self.create_service(FindWall, name_service, self.main_callback,callback_group=self.reentrant_group_1)

        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ranges_logged = False # flag for checking initial values
        self.closest_sector = None
        self.closest_distance = None
        self.facing_wall = False 
        self.at_wall = False
        self.align = False
        self.at_wall_and_ready = False
        self.find_wall_active = False


    def laserscan_callback(self, msg):
        if self.find_wall_active:
            self.get_logger().info("Service activated, finding wall...")
            self.right_distance = msg.ranges[541]
            self.front_distance = msg.ranges[0]
            self.front_right = msg.ranges[630]
            self.right_rear = msg.ranges[450]
            self.front_left = msg.ranges[90]
            self.left = msg.ranges[180]
            self.left_rear = msg.ranges[270]
            self.right_right_rear = msg.ranges[450:541]

            if not self.ranges_logged:
                self.get_logger().info(
                    f"  Right-Rear : {self.right_rear:.2f} m\n"
                    f"  Right      : {self.right_distance:.2f} m\n"
                    f"  Front-Right: {self.front_right:.2f}m\n"
                    f"  Front      : {self.front_distance:.2f} m\n"
                    f"  Front-Left : {self.front_left:.2f} m\n"
                    f"  Left       : {self.left:.2f} m\n"
                    f"  Left-Rear  : {self.left_rear:.2f} m"
                )
                self.ranges_logged = True

            sectors = {
                "Right_Rear": (450, 541),
                "Right": (542, 630),
                "Front_Right": (631, 719),
                "Front_Range_Left": (0, 60), #front range divided into two parts
                "Front_Range_Right": (659,719),
                "Front_Left": (0, 90),
                "Left": (91, 180),
                "Left_Rear": (181, 270),
            }


            min_distances = {key: float('inf') for key in sectors.keys()}
            for sector, (start_idx, end_idx) in sectors.items():
                if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                    sector_ranges = msg.ranges[start_idx:end_idx + 1]
                    if sector_ranges:
                        min_distances[sector] = min(sector_ranges) 

            self.closest_sector = min(min_distances, key=min_distances.get)
            self.closest_distance = min_distances[self.closest_sector]
            self.min_distances = min_distances

            # if self.at_wall and not self.align: # all three sectors of right side should be inside max limit
            #     self.get_logger().info("FLAGGGGG ⭕⭕🔁🔁⭕😂😂.")
            #     max_distances = {key: float('inf') for key in sectors.keys()}
            #     for sector, (start_idx, end_idx) in sectors.items():
            #         if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
            #             sector_ranges = msg.ranges[start_idx:end_idx + 1]
            #             if sector_ranges:
            #                 max_distances[sector] = max(sector_ranges)

            #     self.furthest_right_distance = max_distances["Right"]
            #     wall_threshold = 0.20
            #     self.detections = {sector: max_distance < wall_threshold for sector, max_distance in max_distances.items()} 

            if not self.facing_wall:
                self.face_wall()
            elif not self.at_wall:
                self.go_to_wall()
            elif not self.align:
                self.align_to_right()

        else:
            return

    def main_callback(self,request,response):
        self.find_wall_active = True
        self.at_wall_and_ready = False
        self.facing_wall = False 

        self.get_logger().info("Find wall sequence started")


        while not self.at_wall_and_ready:
            time.sleep(0.1)

        self.get_logger().info("✔️ Wall finding complete. Returning success to client.")
        
        self.find_wall_active = False
        
        stop_action = Twist()
        stop_action.linear.x = 0.0
        stop_action.angular.z = 0.0
        self.publisher.publish(stop_action)

        response.wallfound = True
        return response

    def face_wall(self):
        action = Twist()

        if abs(self.closest_distance - self.front_distance) < 0.01:
            action.linear.x = 0.0
            action.angular.z = 0.0
            self.facing_wall = True
            self.get_logger().info("Facing The Wall ✔️")

        elif self.closest_sector in ["Front_Left", "Front_Range_Left", "Left", "Left_Rear", "Back"]:
            action.linear.x = 0.0
            action.angular.z = 0.2
            self.get_logger().info("Turning Left...")

        elif self.closest_sector in ["Front_Range_Right", "Front_Right", "Right", "Right_Rear"]:
            action.linear.x = 0.0
            action.angular.z = -0.2
            self.get_logger().info("Turning Right...")
        self.publisher.publish(action)


    def go_to_wall(self):
        action = Twist()
        if self.front_distance < 0.25:
            action.linear.x = 0.0
            action.angular.z = 0.0
            self.get_logger().info("Reached the wall ✔️")
            self.at_wall = True
        else:
            action.linear.x = 0.1
            action.angular.z = 0.0

        self.publisher.publish(action)

    def align_to_right(self):
        self.closest_distance = self.min_distances["Right"]
        action = Twist()
        if abs(self.closest_distance - self.right_distance) < 0.01:

            action.linear.x = 0.0
            action.angular.z = 0.0
            self.at_wall_right = True
            self.get_logger().info("✔️ Wall to the right ✔️")
            self.get_logger().info(f"{self.closest_distance},{self.right_distance}")
            self.align = True
            self.at_wall_and_ready = True

        else: 
            action.linear.x = 0.0
            action.angular.z = 0.2
            self.get_logger().info("🔁 Rotating to align wall on the right 🔁")
            self.get_logger().info(f"{self.closest_distance},{self.right_distance}")
        self.publisher.publish(action)

def main(args = None):
    rclpy.init(args=args)
    wall_find_node = WallFinder()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(wall_find_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        wall_find_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
