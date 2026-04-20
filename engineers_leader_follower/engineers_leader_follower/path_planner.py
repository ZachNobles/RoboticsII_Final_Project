from platform import node

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.get_logger().info(f"{self.get_name()} has been started")

        self.robot1_publisher = self.create_publisher(Twist, "/robot1/cmd_vel", 10)
        self.robot2_publisher = self.create_publisher(Twist, "/robot2/cmd_vel", 10)

        self.robot2_offset = 0.4

        self.goal_points = [
            (2.0, 0.0),
            (3.0, 1.0)
        ]
        self.robot2_end_goal = (self.goal_points[-1][0], self.goal_points[-1][1] - self.robot2_offset)

        self.current_goal_point = self.goal_points[0]

        self.robot1_goal = None
        self.robot2_goal = None

        self.robot1_points = []

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot2_x = 0.0
        self.robot2_y = -self.robot2_offset
        self.robot2_theta = 0.0

        self.distance_threshold = self.robot2_offset

        self.velocity = 0.5
        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    
    def calculate_path_distance(self):
        sum = 0.0
        for i in range(1, len(self.robot1_points)):
            sum += np.linalg.norm(np.array(self.robot1_points[i]) - np.array(self.robot1_points[i-1]))
        return sum

    
    def timer_callback(self):
        robot1_msg = Twist()
        robot2_msg = Twist()

        self.robot1_points.append((self.robot1_x, self.robot1_y))

        if np.linalg.norm(np.array(self.current_goal_point) - np.array((self.robot1_x, self.robot1_y))) < 0.2:
            self.get_logger().info(f"Reached goal point: {self.current_goal_point}")

            if len(self.goal_points) > 0:
                self.current_goal_point = self.goal_points.pop(0)
                self.get_logger().info(f"New goal point: {self.current_goal_point}")
            else:
                robot1_msg.linear.x = 0.0
                robot1_msg.linear.y = 0.0
                robot2_msg.linear.x = 0.0
                robot2_msg.linear.y = 0.0

                if np.linalg.norm(np.array(self.robot2_end_goal) - np.array((self.robot2_x, self.robot2_y))) > 0.2:
                    self.get_logger().info(f"Robot 1 reached all goal points. Robot 2 moving to end goal: {self.robot2_end_goal}")
                    theta2 = np.arctan2(self.robot2_end_goal[1] - self.robot2_y, self.robot2_end_goal[0] - self.robot2_x)
                    robot2_msg.linear.x = self.velocity * np.cos(theta2)
                    robot2_msg.linear.y = self.velocity * np.sin(theta2)
                    self.robot2_x += robot2_msg.linear.x * 0.05
                    self.robot2_y += robot2_msg.linear.y * 0.05
                
                else:
                    robot2_msg.linear.x = 0.0
                    robot2_msg.linear.y = 0.0
                    self.robot1_publisher.publish(robot1_msg)
                    self.robot2_publisher.publish(robot2_msg)
                    self.get_logger().info("All goal points reached. Stopping robots.")
                    return

                self.robot1_publisher.publish(robot1_msg)
                self.robot2_publisher.publish(robot2_msg)
                

        theta1 = np.arctan2(self.current_goal_point[1] - self.robot1_y, self.current_goal_point[0] - self.robot1_x)
        robot1_msg.linear.x = self.velocity * np.cos(theta1)
        robot1_msg.linear.y = self.velocity * np.sin(theta1)

        path_distance = self.calculate_path_distance()

        if path_distance > self.distance_threshold:
            self.robot2_goal = self.robot1_points.pop(0)
            theta2 = np.arctan2(self.robot2_goal[1] - self.robot2_y, self.robot2_goal[0] - self.robot2_x)
            robot2_msg.linear.x = self.velocity * np.cos(theta2)
            robot2_msg.linear.y = self.velocity * np.sin(theta2)
        
        self.robot1_publisher.publish(robot1_msg)
        self.robot2_publisher.publish(robot2_msg)
        self.get_logger().info(f"Published velocities - Robot1: ({robot1_msg.linear.x:.2f}, {robot1_msg.linear.y:.2f}), Robot2: ({robot2_msg.linear.x:.2f}, {robot2_msg.linear.y:.2f})")

        self.robot1_x += robot1_msg.linear.x * 0.05
        self.robot1_y += robot1_msg.linear.y * 0.05
        self.robot2_x += robot2_msg.linear.x * 0.05
        self.robot2_y += robot2_msg.linear.y * 0.05
    
    def shutdown(self):
        self.get_logger().info(f"{self.get_name()} is shutting down")
        robot1_msg = Twist()
        robot2_msg = Twist()

        robot1_msg.linear.x = 0.0
        robot1_msg.linear.y = 0.0
        robot2_msg.linear.x = 0.0
        robot2_msg.linear.y = 0.0
        self.robot1_publisher.publish(robot1_msg)
        self.robot2_publisher.publish(robot2_msg)





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