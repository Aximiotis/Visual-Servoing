import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):  
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',  # άλλαξε το αν χρειάζεται
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.counter = 132
        self.save_path = '/home/roboticslab/gazebo_images'  # άλλαξε path

        os.makedirs(self.save_path, exist_ok=True)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = os.path.join(self.save_path, f'image_{self.counter:04d}.jpg')
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved {filename}')



def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
