
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Twist

class DroneVision(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.bridge = CvBridge()
        self.yolo = YOLO("runs/detect/yolo_retrain_single_class1/weights/best.pt")  # Το μοντέλο YOLO
        self.Z=0
        self.Y=0
        self.X=0
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.rgb_cb, 10)
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_cb, 10)
        self.publisher_ = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        #/mavros/setpoint_velocity/cmd_vel_unstamped
        self.latest_rgb = None
        self.latest_depth = None

    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.process()

    def process(self):

        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        x1=0
        x2=0
        self.Y=0
        self.X=0
        label="tools"
        results = self.yolo(self.latest_rgb)[0]
        for det in results.boxes.data.cpu().numpy():
            x1, y1, x2, y2, conf, cls = det
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2) / 2)
            print(x1+x2)
            print(y1+y2)
            distance = self.latest_depth[y_c, x_c]
            X = (x_c )
            Y = (y_c ) 
            Z = distance
            self.Z=Z
            self.X=X
            self.Y=Y
            label = self.yolo.names[int(cls)]
            #print(Z)
            #print(X)
            #print(Y)

        self.latest_rgb = None
        self.latest_depth = None
        msg1 = Twist()
        kz=0.1
        ky=0.01
        kx=0.01
        k_z=0.004

        if label=="car" or label=="truck" or label=="bus":
            msg1.linear.x = kz*(self.Z-10)
            msg1.linear.y = -kx*(self.X-160)
            msg1.linear.z = -ky*(self.Y-20)
           # msg1.angular.z= k_z*np.arctan(msg1.linear.x/msg1.linear.y)
            self.publisher_.publish(msg1)
        else:
            msg1.linear.y =  0.0
            msg1.linear.x =  0.0
            msg1.linear.z =  0.0
            self.publisher_.publish(msg1)
            
        # if self.Z>8:
        #      msg1.linear.y =  1.0
        #      self.publisher_.publish(msg1)
        #      self.get_logger().info('Publishing: Linear x = 0.5, Angular z = 0.0')       
        # if  self.X<320 and self.X>=180:
        #     msg1.linear.x =  0.8
        #     self.publisher_.publish(msg1)
        #     self.get_logger().info('Publishing: Linear x = 0.0, Angular z = -1.0')
        # elif self.X<140 and self.X>0:
        #     msg1.linear.x =  -0.8
        #     self.publisher_.publish(msg1)
        # else : 
        #     msg1.linear.x =  0.0
        #     msg1.angular.z = 0.0
        #     self.publisher_.publish(msg1)
        #     self.get_logger().info('Publishing: Linear x = 0.0, Angular z = 0.0')

        # if  self.Y<120 and self.Y>=80:
        #     msg1.linear.z =  0.8
        #     self.publisher_.publish(msg1)
        #     self.get_logger().info('Publishing: Linear x = 0.0, Angular z = -1.0')
        # elif self.Y<40 and self.Y>0:
        #     msg1.linear.z =  -0.8
        #     self.publisher_.publish(msg1)
        # else : 
        #     msg1.linear.z =  0.0
        #     msg1.angular.z = 0.0
        #     self.publisher_.publish(msg1)
        #     self.get_logger().info('Publishing: Linear x = 0.0, Angular z = 0.0')





def main(args=None):
    rclpy.init(args=args)
    node = DroneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
