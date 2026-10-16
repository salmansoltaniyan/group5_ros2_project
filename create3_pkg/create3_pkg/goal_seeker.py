import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class GoalSeeker(Node):
    def __init__(self):
        super().__init__('goal_seeker_node')
        
        # Goal coordinates (Relative distance to travel in meters)
        self.goal_dx = 1.0  
        self.goal_dy = .5  
        
        self.STATE_INIT = 0
        self.STATE_MOVING = 1
        self.STATE_DONE = 2
        self.state = self.STATE_INIT
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Only Odometry and Cmd_vel needed!
        self.odom_sub = self.create_subscription(Odometry, '/robot_9/odom', self.odom_callback, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, '/robot_9/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"System started. Goal set to: dX:{self.goal_dx}, dY:{self.goal_dy}")

    def odom_callback(self, msg):
        if self.state == self.STATE_INIT:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # WHY DO WE NEED YAW? 
        # Because we need to know which way the robot is currently facing!
        # If we don't know where it's facing, we don't know if it should turn left or right to reach the goal.
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.dx = self.current_x - self.initial_x
        self.dy = self.current_y - self.initial_y

        if self.state == self.STATE_INIT:
            self.state = self.STATE_MOVING 

    def control_loop(self):
        if self.state == self.STATE_INIT or self.state == self.STATE_DONE:
            return
        
        # 1. How far are we from the goal?
        remaining_x = self.goal_dx - self.dx
        remaining_y = self.goal_dy - self.dy
        distance_to_goal = math.hypot(remaining_x, remaining_y)

        # 2. What angle is the goal relative to our starting point?
        target_angle = math.atan2(remaining_y, remaining_x)
        
        # 3. What is the difference between where we are facing (Yaw) and the target angle?
        angle_diff = target_angle - self.current_yaw
        
        # Normalize the angle so it's between -pi and pi
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()
        
        # 4. Movement Logic
        if distance_to_goal > 0.05: # If we are further than 5 centimeters away
            
            if abs(angle_diff) > 0.1: # If we are facing the wrong way (more than ~5 degrees off)
                # Rotate towards the goal, don't move forward yet
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                cmd.linear.x = 0.0 
            else:
                # We are facing the goal, drive straight!
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0 
                
        else:
            # We reached the goal!
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("🎉 Arrived at the goal!")
            self.state = self.STATE_DONE

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoalSeeker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
