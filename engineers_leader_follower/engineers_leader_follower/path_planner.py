import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self.robot2_offset = 0.4
        self.heading_error_gain = 1.0

        self.get_logger().info(f"{self.get_name()} has been started")

        self.robot1_publisher = self.create_publisher(Twist, "/robot1/cmd_vel", 10)
        self.robot2_publisher = self.create_publisher(Twist, "/robot2/cmd_vel", 10)

        self.robot1_odom_subscriber = self.create_subscription(Odometry, "/robot1/odometry/filtered", self.robot1_odom_callback, 10)
        self.robot2_odom_subscriber = self.create_subscription(Odometry, "/robot2/odometry/filtered", self.robot2_odom_callback, 10)

        self.goal_points = [
            (2.0, 0.0),
            (3.0, 1.0)
        ]

        self._goals_initialized = False

        self.robot2_end_goal = (self.goal_points[-1][0], self.goal_points[-1][1] - self.robot2_offset)

        self.current_goal_point = self.goal_points[0]

        self.robot1_goal = self.goal_points[0]
        self.robot2_goal = None

        self.robot1_points = []
        self.robot1_all_points = []
        self.robot2_all_points = []

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot2_x = 0.0
        self.robot2_y = -self.robot2_offset
        self.robot2_theta = 0.0

        self.distance_threshold = self.robot2_offset
        self.velocity = 0.5

        self.timer = self.create_timer(0.05, self.timer_callback)

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot1_theta = math.atan2(siny_cosp, cosy_cosp)

        if not self._goals_initialized:
            self.goal_points = [(p[0] + self.robot1_x, p[1] + self.robot1_y) for p in self.goal_points]
            self.current_goal_point = self.goal_points.pop(0)
            self.robot1_goal = self.current_goal_point
            self.robot2_end_goal = (self.goal_points[-1][0], 
                                    self.goal_points[-1][1] - self.robot2_offset)
            self._goals_initialized = True
            self.get_logger().info(f"Goals initialized. First: {self.current_goal_point}, Remaining: {self.goal_points}")

    def robot2_odom_callback(self, msg):
        self.robot2_x = msg.pose.pose.position.x
        self.robot2_y = msg.pose.pose.position.y - self.robot2_offset
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot2_theta = math.atan2(siny_cosp, cosy_cosp)

    def calculate_path_distance(self):
        total = 0.0
        for i in range(1, len(self.robot1_points)):
            total += np.linalg.norm(np.array(self.robot1_points[i]) - np.array(self.robot1_points[i-1]))
        return total

    def timer_callback(self):
        if not self._goals_initialized:
            return
        
        robot1_msg = Twist()
        robot2_msg = Twist()

        self.robot1_points.append((self.robot1_x, self.robot1_y))
        self.robot1_all_points.append((self.robot1_x, self.robot1_y))
        self.robot2_all_points.append((self.robot2_x, self.robot2_y))

        # If robot 1 has reached its current goal
        if np.linalg.norm(np.array(self.current_goal_point) - np.array((self.robot1_x, self.robot1_y))) < 0.2:
            self.get_logger().info(f"Reached goal point: {self.current_goal_point}")

            if len(self.goal_points) > 0:
                self.current_goal_point = self.goal_points.pop(0)
                self.get_logger().info(f"New goal point: {self.current_goal_point}")
            else:
                self.robot1_goal = None

                if np.linalg.norm(np.array(self.robot2_end_goal) - np.array((self.robot2_x, self.robot2_y))) > 0.2:
                    self.get_logger().info(f"Robot 1 reached all goal points. Robot 2 moving to end goal: {self.robot2_end_goal}")
                    self.robot2_goal = self.robot2_end_goal
                else:
                    self.get_logger().info("All goal points reached. Stopping robots.")
                    self.get_logger().info(f"robot 1 points: {self.robot1_all_points}")
                    self.get_logger().info(f"robot 2 points: {self.robot2_all_points}")
                    self.shutdown()
                    rclpy.shutdown()

        if self.robot1_goal is not None:
            dx = self.current_goal_point[0] - self.robot1_x
            dy = self.current_goal_point[1] - self.robot1_y

            dist = math.sqrt(dx*dx + dy*dy)
            ux = dx / dist
            uy = dy / dist
            cross_track = -ux * self.robot1_y + uy * self.robot1_x
            
            robot1_msg.linear.x = self.velocity * ux - self.heading_error_gain * cross_track * (-uy)
            robot1_msg.linear.y = self.velocity * uy - self.heading_error_gain * cross_track * ux


            if self.calculate_path_distance() > self.distance_threshold:
                self.robot2_goal = self.robot1_points.pop(0)

        if self.robot2_goal is not None:
            dx = self.robot2_goal[0] - self.robot2_x
            dy = self.robot2_goal[1] - self.robot2_y
            
            dist = math.sqrt(dx*dx + dy*dy)
            ux = dx / dist
            uy = dy / dist
            cross_track = -ux * self.robot2_y + uy * self.robot2_x
            
            robot2_msg.linear.x = self.velocity * ux - self.heading_error_gain * cross_track * (-uy)
            robot2_msg.linear.y = self.velocity * uy - self.heading_error_gain * cross_track * ux

        self.robot1_publisher.publish(robot1_msg)
        self.robot2_publisher.publish(robot2_msg)

    def shutdown(self):
        self.get_logger().info(f"{self.get_name()} is shutting down")
        self.robot1_publisher.publish(Twist())
        self.robot2_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()

    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        path_planner.shutdown()
        path_planner.get_logger().info('Interrupt received, shutting down')

    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()