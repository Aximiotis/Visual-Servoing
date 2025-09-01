import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
import pandas as pd
import numpy as np
from ultralytics import YOLO
import joblib
class ImageSaver(Node):
    def __init__(self):  
        super().__init__('image_saver')
        self.yolo = YOLO("runs/detect/yolo_retrain_single_class2/weights/best.pt")
        self.rgb_sub = self.create_subscription(
            Image,
            '/depth_camera/image_raw',  # άλλαξε το αν χρειάζεται
            self.rgb_cb,
            10)
        self.bridge = CvBridge()
        self.counter = 200
        self.save_path = '/home/roboticslab/gazebo_images'  # άλλαξε path
        self.flag=0
        os.makedirs(self.save_path, exist_ok=True)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
             Odometry,                  # τύπος μηνύματος
             '/mavros/global_position/local', 
             self.listener_callback1,  # συνάρτηση callback
             qos                       # queue size
         )
        self.subscription = self.create_subscription(
             Odometry,                  # τύπος μηνύματος
             '/ambulance_moving/odom', 
             self.listener_callback2,  # συνάρτηση callback
             qos                       # queue size
         )
        self.pos=[0,0,0]
        self.bus=[0,0,0]
        self.latest_rgb = None
        self.model = joblib.load('mlp_model.pkl')
        self.publisher=self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',1)
        self.create_timer(0.01,self.position_publisher)
        self.pos_final=np.array([0,0,0])

    def listener_callback1(self,msg:Odometry):
        pos=msg.pose.pose.position
        self.pos[0]=pos.x
        self.pos[1]=pos.y
        self.pos[2]=pos.z
    def listener_callback2(self,msg:Odometry):
        pos=msg.pose.pose.position
        self.bus[0]=pos.x+10
        self.bus[1]=pos.y+10
        self.bus[2]=pos.z+3
    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        time.sleep(0.01)
        self.process()
    # def listener_callback(self, msg):
    #     time.sleep(2)
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     filename = os.path.join(self.save_path, f'image_{self.counter:04d}.jpg')
    #     cv2.imwrite(filename, cv_image)
    #     self.get_logger().info(f'Saved {filename}')
    #     self.counter=self.counter+1
    def process(self):

        if self.latest_rgb is None:
            return
        
        x1=0
        x2=0
        x_c=0
        y_c=0
        u0=160.5
        fx=277.2
        v0=120
        fy=277.2
        label="tools"
        results = self.yolo(self.latest_rgb)[0]
        for det in results.boxes.data.cpu().numpy():
            x1, y1, x2, y2, conf, cls = det
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2) / 2)
            self.x = (x_c -u0)/fx
            self.y = (y_c -v0)/fy 
            label = self.yolo.names[int(cls)]

        self.latest_rgb = None
        Rdd1=np.array([
                [np.cos(np.pi/6),0,np.sin(np.pi/6)],
                [0,1,0],
                [-np.sin(np.pi/6),0,np.cos(np.pi/6)]
            ])
        Rd1c=np.array([
                [0,0,1],
                [-1,0,0],
                [0,-1,0],
            ])
        if label=="car" or label=="truck" or label=="bus":
            area=(x2-x1)*(y2-y1)
            ratio=(x2-x1)/(y2-y1)
            w=x2-x1
            h=y2-y1
            distance=np.sqrt((self.pos[0]-self.bus[0])**2+(self.pos[1]-self.bus[1])**2+(self.pos[2]-self.bus[2])**2)
            data = {
                'area':area,
                'ratio':ratio,
                'width':w,
                'height':h,
                'x_center':x_c,
                'y_center':y_c,
                'output': distance
            }
            self.flag=1
            data1=np.array([area,ratio,w,h,x_c,y_c]).reshape(1,-1)
            dis=self.model.predict(data1)
            print(dis[0])
            print(distance)

            X=self.x*dis[0]
            Y=self.y*dis[0]
            v=np.array([X,Y,dis[0]])
            pos1=Rdd1 @ Rd1c @ v.T
            self.pos_final=pos1+np.array([self.pos[0],self.pos[1],self.pos[2]]).T
            # df = pd.DataFrame([data])
            # print(data)
            # df.to_csv('GFG.csv', mode='a', index=False, header=False)
    def position_publisher(self):
            if self.flag==1:
                msg3=PoseStamped()
                msg3.pose.position.x=self.pos_final[0]
                msg3.pose.position.y=self.pos_final[1]
                msg3.pose.position.z=15.0
                self.publisher.publish(msg3)

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
