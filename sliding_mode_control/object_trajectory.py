import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time

class ObjectRoute(Node):
    def __init__(self):
        super().__init__('object_route')
        self.publisher_ = self.create_publisher(Twist, '/ambulance_moving/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.start_time = time.time()

        # Route parameters
        self.radius = 40
        self.omega = 0.2
        self.v_forward = 1.0
        self.vz = 0.2

    def timer_callback(self):
        t = time.time() - self.start_time
        msg = Twist()

        # Phase 1: Helix (0–20s)
        if t <= 30.0 and t >= 23:
            vx = -self.radius * self.omega * np.sin(self.omega * t) 
            vy =  self.radius * self.omega * np.cos(self.omega * t)
            vz = self.vz
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.linear.z = float(0)

        # Phase 2: Straight escape (20–30s)
        elif t <= 1.0:
            msg.linear.x = 3.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
        elif t <= 12.0:
            msg.linear.x = 10.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
        elif t <= 17.0:
            msg.linear.x = 6.0
            msg.linear.y = 6.0
            msg.linear.z = 0.0
        elif t <= 23.0:
            msg.linear.x = 6.0
            msg.linear.y = 10.0
            msg.linear.z = 0.0
        elif t>=30.0 and t<=45:
            msg.linear.x = -4.0
            msg.linear.y = 5.0
            msg.linear.z = 0.0
        elif t>=45.0 and t<=55:
            msg.linear.x = -8.0
            msg.linear.y = 5.0
            msg.linear.z = 0.
        elif t>=55.0 and t<=65:
            msg.linear.x = -10.0
            msg.linear.y = 8.0
            msg.linear.z = 0.0
        elif t>=65.0 and t<=75:
            msg.linear.x = -10.0
            msg.linear.y = 15.0
            msg.linear.z = 0.0
        elif t>=75.0 and t<=85:
            msg.linear.x = -5.0
            msg.linear.y = 15.0
            msg.linear.z = 0.0
        elif t>=85.0 and t<=100:
            msg.linear.x = 10.0
            msg.linear.y = 15.0
            msg.linear.z = 0.0
        elif t>=100.0 and t<=120:
            msg.linear.x = 16.0
            msg.linear.y = 7.0
            msg.linear.z = 0.0
        elif t>=120.0 and t<=140:
            msg.linear.x = 16.0
            msg.linear.y = -1.0
            msg.linear.z = 0.0
        elif t>=140.0 and t<=160:
            msg.linear.x = 8.0
            msg.linear.y = -7.0
            msg.linear.z = 0.0
        elif t>=160.0 and t<=180:
            msg.linear.x = 1.0
            msg.linear.y = -13.0
            msg.linear.z = 0.0
        elif t>=180.0 and t<=200:
            msg.linear.x = -5.0
            msg.linear.y = -16.0
            msg.linear.z = 0.0
        elif t>=200.0 and t<=220:
            msg.linear.x = -5.0
            msg.linear.y = -18.0
            msg.linear.z = 0.0
        elif t>=220.0 and t<=240:
            msg.linear.x = -1.0
            msg.linear.y = -10.0
            msg.linear.z = 0.0
        elif t>=240.0 and t<=300:
            msg.linear.x = -5.0
            msg.linear.y = -5.0
            msg.linear.z = 0.0
        # Stop afterwards
        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

        # Publish
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRoute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
