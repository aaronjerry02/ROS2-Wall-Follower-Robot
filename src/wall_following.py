#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
from wall_follower_interfaces.srv import FindWall
from rclpy.action import ActionClient
from wall_follower_interfaces.action import OdomRecord


class AutoWallFollowerNode(Node):
    def __init__(self):
        super().__init__('Wall_Follower_Node')

        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_client = ActionClient(self, OdomRecord, '/record_odom')

        name_service = '/wall_finder_service'
        self.node = self.create_client(FindWall, name_service)

        while not self.node.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service '+name_service+' not available, waiting again...')

        self.req = FindWall.Request() #empty request
        self.future = self.node.call_async(self.req)

        self.wall_found = False
        self.ranges_logged = False # flag for checking initial values
        self.goal_sent = False
        self.get_logger().info("Autonomous Follower Node Ready and Waiting...")

    def send_goal(self):
        goal_msg = OdomRecord.Goal()
        self.get_logger().info('Sending goal to start odometry recording...')

        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        # Attach callback for when the result is received
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        recorded_path = result.list_of_odoms
        self.get_logger().info(f'Action completed, array of positions:')
        for point in recorded_path:
            self.get_logger().info(f'   x: {point.x:.2f}, y: {point.y:.2f}')

        self.get_logger().info('Stopping the robot...')
        self.stop_bot()



    def feedback_callback(self, feedback_msg):
        total_distance = feedback_msg.feedback.current_total
        self.get_logger().info(f'Distance traveled: {total_distance:.2f} meters')


    def laserscan_callback(self, msg):
        if self.wall_found and not self.goal_sent:
            self.send_goal()
            self.goal_sent = True
        if self.wall_found:
            self.get_logger().info("✅Wall Following Initiating✅")
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


            if self.wall_found:
                min_distances = {key: float('inf') for key in sectors.keys()}
                for sector, (start_idx, end_idx) in sectors.items():
                    if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                        sector_ranges = msg.ranges[start_idx:end_idx + 1]
                        if sector_ranges:
                            min_distances[sector] = min(sector_ranges)  

                wall_threshold = 0.3
                self.detections = {sector: min_distance < wall_threshold for sector, min_distance in min_distances.items()}
            
            
            if self.detections["Front_Range_Left"] or self.detections["Front_Range_Right"]:
                self.get_logger().info("⭕Front Distance less than 0.5⭕")
                self.turn_bot()

            elif self.wall_found:
                self.get_logger().info("🟦Starting Alignment...🟦")
                self.wall_aligner()
            
        else:
            self.get_logger().info("⭕Waiting for service response...⭕")
    
    def turn_bot(self):
        action = Twist()
        action.linear.x = 0.0
        action.angular.z = 0.2
        self.get_logger().info("Turning in Turn_Bot function...")
        self.publisher.publish(action)

    def wall_aligner(self):
        valid_ranges = [i for i in self.right_right_rear if not math.isinf(i) and not math.isnan(i)]
        min_dist = min(valid_ranges) if valid_ranges else float('inf')
        action = Twist()
        if (min_dist > 0.255):
            self.get_logger().info("Too far from wall, Turning inwards...")
            action.angular.z = -0.2
            action.linear.x = 0.1
        elif (min_dist < 0.20):
            self.get_logger().info("Too close to the wall, Turning outwards...")
            action.angular.z = 0.2
            action.linear.x = 0.1
        elif (min_dist < 0.255 and min_dist > 0.20):
            self.get_logger().info("⬆️ Moving Straight ⬆️")
            action.angular.z = 0.0
            action.linear.x = 0.1
        else:
            self.get_logger().info("Error. Stopping Bot.")
            action.angular.z = 0.0
            action.linear.x = 0.0
        self.publisher.publish(action)

    def stop_bot(self):
        action = Twist()
        action.linear.x = 0.0
        action.angular.z = 0.0
        self.get_logger().info("⭕Stop bot⭕")
        self.publisher.publish(action)
        self.wall_found = False
        
        self.destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = AutoWallFollowerNode()

    # Keep spinning until the service response is received
    while rclpy.ok() and not node.future.done():
        rclpy.spin_once(node)

    # Check the service response
    try:
        response = node.future.result()
        if response.wallfound:
            node.get_logger().info("✅ Wall found. Proceeding to wall-following stage.")
            node.wall_found = True
        else:
            node.get_logger().info("❌ Wall NOT found. Aborting.")
            node.destroy_node()
            rclpy.shutdown()
            return
    except Exception as e:
        node.get_logger().info(f"Service call failed: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Now spin normally for the wall following loop
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
