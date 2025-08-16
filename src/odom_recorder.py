#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from rclpy.action import ActionServer
from wall_follower_interfaces.action import OdomRecord
import math
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odometer_recorder_node')

        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.reentrant_group_1)

        self.action_server = ActionServer(self, OdomRecord, '/record_odom', self.execute_action, callback_group=self.reentrant_group_1)
        self.last_x = None
        self.last_y = None
        self.last_odom = None
        self.total_distance = 0.0

    def odom_callback(self,msg):
        self.last_odom = msg.pose.pose.position

    async def execute_action(self, goal_handle):
        while getattr(self, 'last_odom', None) is None:
            self.get_logger().info('Waiting for the first odometry message...')
            time.sleep(0.5)

        self.get_logger().info('Executing goal...')
        self.first_odom = self.last_odom #initial value to get a starting point
        self.odom_record = []
        self.total_distance=0.0
        self.last_x = self.first_odom.x
        self.last_y = self.first_odom.y

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                break

            self.odom_record.append(self.last_odom)
            feedback_msg = OdomRecord.Feedback()

            incremental_distance = self.calc_distance(self.last_odom.x, self.last_x, self.last_odom.y, self.last_y)
            self.total_distance += incremental_distance
            self.last_x = self.last_odom.x
            self.last_y = self.last_odom.y #update the position
            feedback_msg.current_total = self.total_distance
            self.get_logger().info(f'Publishing feedback: {feedback_msg.current_total}')
            goal_handle.publish_feedback(feedback_msg)

            distance_from_start = self.calc_distance(self.last_odom.x, self.first_odom.x, self.last_odom.y, self.first_odom.y)

            if(distance_from_start <= 0.75 and self.total_distance > 1.0):
                self.get_logger().info('Lap completed!')
                break

            time.sleep(1)

        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        return result

    def calc_distance(self,x1,x2,y1,y2):
        return math.sqrt((x1-x2)**2+(y1-y2)**2)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



