import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path planner")
        self.robot1_publisher = self.create_publisher(Twist, "robot1/raw_vel", 10)
        self.robot2_publisher = self.create_publisher(Twist, "robot2/raw_vel", 10)

        self.goal_points = [
            (1.0, 5.0),
            (1.0, 8.0),

        ]

        self.current_goal_point = self.goal_points[0]

        self.robot1_goal = None
        self.robot2_goal = None

        self.robot1_points = []

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0

        self.velocity = 1.0
        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.T_2_1 = np.array([0.0, -0.3, 0.0]).T # robot 2 to robot 1 transform
    
    def timer_callback(self):
        robot1_msg = Twist()
        robot2_msg = Twist()

        self.robot1_points.append((self.robot1_x, self.robot1_y))

        theta1 = np.arctan2(self.current_goal_point[1] - self.robot1_y, self.current_goal_point[0] - self.robot1_x)
        robot1_msg.linear.x = self.velocity * np.cos(theta1)
        robot1_msg.linear.y = self.velocity * np.sin(theta1)

        if len(self.robot1_points) > 5:
            robot2_goal = self.robot1_points.pop(0)
            theta2 = np.arctan2(robot2_goal[1] - self.robot2_y, robot2_goal[0] - self.robot2_x)
            robot2_msg.linear.x = self.velocity * np.cos(theta2)
            robot2_msg.linear.y = self.velocity * np.sin(theta2)
        
        self.robot1_publisher.publish(robot1_msg)
        self.robot2_publisher.publish(robot2_msg)










def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()

    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()