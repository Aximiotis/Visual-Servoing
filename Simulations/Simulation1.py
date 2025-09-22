
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry

class DroneVision(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.bridge = CvBridge()
        self.yolo = YOLO("runs/detect/yolo_retrain_single_class2/weights/best.pt")  # Το μοντέλο YOLO
        self.Z=0
        self.Y=0
        self.X=0
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
             Odometry,                  # τύπος μηνύματος
             '/mavros/global_position/local', 
             self.listener_callback,  # συνάρτηση callback
             qos                       # queue size
         )
        self.subscription  # Χρειάζεται για να μην σβηστεί από τον garbage collector
        self.pos=[0,0,0]
        self.ori=[0,0,0,0]
        self.Rot=np.zeros((3,3))

        self.state_ex=0
        self.state_ey=0

        self.previous_x=0
        self.previous_y=0
        self.annotated_image_pub = self.create_publisher(Image,'/drone/yolo/image_annotated',10)

        self.rgb_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.rgb_cb, 10)
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_cb, 10)
        self.publisher_ = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.latest_rgb = None
        self.latest_depth = None

        self.integralx=0.0
        self.integraly=0.0
        self.vx=0
        self.vy=0
        self.errorx=np.zeros((10,1))
        self.errory=np.zeros((10,1))
        self.errorz=np.zeros((10,1))
        self.ex=0
        self.ey=0
        self.ez=0
        self.counter=0

    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def listener_callback(self,msg):


        pos=msg.pose.pose.position
        self.pos=pos
        ori=msg.pose.pose.orientation
        self.ori=ori
        Rot=R.from_quat([ori.x,ori.y,ori.z,ori.w])
        self.Rot=Rot.as_matrix()
        print(self.ori)

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.process()

    def process(self):

        if self.latest_rgb is None :
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
            X = (x_c -160)/277.2
            Y = (y_c -60)/277.2 
            Z = (60-(x2-x1))/30
            self.Z=Z
            self.X=X
            self.Y=Y
            label = self.yolo.names[int(cls)]

        annotated = results.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.annotated_image_pub.publish(annotated_msg)

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

        R_total=self.Rot @ Rdd1 @ Rd1c 


        errorx=0.0
        errory=0.0
        errorz=0.0
        e=np.array([self.X,self.Y,self.Z]).T

        # error smoothing

        if self.counter<10:

            self.errorx[self.counter]=e[0]
            self.errory[self.counter]=e[1]
            self.errorz[self.counter]=e[2]

            if self.counter==0:
                errorx=self.errorx[0]
                errory=self.errory[0]
                errorz=self.errorz[0]
                self.ex=self.errorx[0]
                self.ey=self.errory[0]
                self.ez=self.errorz[0]
            else :
                errorx=0.8*self.ex+0.2*self.errorx[self.counter]
                errory=0.8*self.ey+0.2*self.errory[self.counter]
                errorz=0.8*self.ez+0.2*self.errorz[self.counter]
                self.ex=errorx
                self.ey=errory
                self.ez=errorz

        elif self.counter>=10:
            self.errorx[0:9]=self.errorx[1:10]
            self.errory[0:9]=self.errory[1:10]
            self.errorz[0:9]=self.errorz[1:10]
            self.errorx[9]=e[0]
            self.errory[9]=e[1]
            self.errorz[9]=e[2]
            errorx=0.55*np.mean(self.errorx[0:9])+0.45*e[0]
            errory=0.55*np.mean(self.errory[0:9])+0.45*e[1]
            errorz=0.55*np.mean(self.errorz[0:9])+0.45*e[2]

        self.counter+=1            
        e_normal=R_total @ np.array([float(errorx),float(errory),float(errorz)])
        


        print(e_normal)


        # gain calibration
        def sigma(x):
            return  1/(1+np.exp(-x))
        self.latest_rgb = None
        self.latest_depth = None
        msg1 = Twist()
        kz=0.04

        # adaptive gains
        ky=2*np.round(abs(e_normal[1]/0.075))+5*np.round(abs((e_normal[1]-self.state_ey)/0.075))+5
        kx=2.5*np.round(abs(e_normal[0]/0.075))+2.5*np.round(abs((e_normal[0]-self.state_ex)/0.075))+5
        k_z=2*np.round(abs(self.X/0.15))+2*np.round(abs(self.X-self.previous_x))+1
        kx=kx*sigma((kx-0.5)/5)
        ky=ky*sigma((ky-0.5)/5)

        self.integralx+=0.0001*e_normal[0]
        self.integraly+=0.0001*e_normal[1]

        # s1=(e_normal[1]-self.state_ey)/0.01 + e_normal[1]
        # s0=(e_normal[0]-self.state_ex)/0.01 + e_normal[0]

        if label=="car" or label=="truck" or label=="bus":
            if np.sqrt(e_normal[0]**2+e_normal[1]**2)>0.15:
                # pid control
                msg1.linear.x =np.clip(kx*e_normal[0] + 2*kx*(e_normal[0]-self.state_ex) + 1*self.integralx,-35,35) 
                msg1.linear.y =np.clip(ky*e_normal[1] + 5*ky*(e_normal[1]-self.state_ey)+ 1*self.integraly,-35,35) 

                # continious calculations
                msg1.linear.x=msg1.linear.x+0.1*(msg1.linear.x-self.vx)+0.001*e_normal[0]
                msg1.linear.y=msg1.linear.y+0.1*(msg1.linear.y-self.vy)+0.001*e_normal[1]

                msg1.angular.z = -k_z*self.X-k_z*10*(self.X-self.previous_x)
                msg1.angular.y=0.0
                self.publisher_.publish(msg1)
            
                self.state_ex = e_normal[0]
                self.state_ey = e_normal[1]
                self.previous_x=self.X
                self.vx=msg1.linear.x
                self.vy=msg1.linear.y
            else :
                msg1.linear.x = np.clip(5.8*e_normal[0] + 5*(e_normal[0]-self.state_ex),-15,15) 
                msg1.linear.y = np.clip(5.8*e_normal[1] + 5*(e_normal[1]-self.state_ey),-15,15) 

                msg1.linear.x=msg1.linear.x+0.1*(msg1.linear.x-self.vx)+0.001*e_normal[0]
                msg1.linear.y=msg1.linear.y+0.1*(msg1.linear.y-self.vy)+0.001*e_normal[1]

                msg1.angular.z = -0.01*self.X-0.001*10*(self.X-self.previous_x)
                msg1.angular.y=0.0
                self.publisher_.publish(msg1)
    
                self.state_ex = e_normal[0]
                self.state_ey = e_normal[1]
                self.previous_x=self.X

        else:
                msg1.linear.y =  0.0
                msg1.linear.x =  0.0
                msg1.linear.z =  0.0
                self.publisher_.publish(msg1)


def main(args=None):
    rclpy.init(args=args)
    node = DroneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    node = DroneVision()