
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import joblib
class DroneVision(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.bridge = CvBridge()
        self.yolo = YOLO("runs/detect/yolo_retrain_single_class2/weights/best.pt")  # Το μοντέλο YOLO
        self.Z=10
        self.y=0
        self.x=0

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
        self.counter=0
        self.model = joblib.load('mlp_model.pkl')
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.rgb_cb, 10)
        self.publisher_ = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.latest_rgb = None
        self.busx=0
        self.buxy=0
        self.busz=0

    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def listener_callback(self,msg):

        pos=msg.pose.pose.position
        self.pos[0]=pos.x
        self.pos[1]=pos.y
        self.pos[2]=pos.z
        ori=msg.pose.pose.orientation
        self.ori=ori
        Rot=R.from_quat([ori.x,ori.y,ori.z,ori.w])
        self.Rot=Rot.as_matrix()


    def process(self):
        if self.latest_rgb is None :
            return
        
        u0=160.5
        fx=277.2
        v0=120
        fy=277.2

        x1=0
        x2=0
        self.y=0
        self.x=0
        label="tools"
        results = self.yolo(self.latest_rgb)[0]
        for det in results.boxes.data.cpu().numpy():
            x1, y1, x2, y2, conf, cls = det
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2) / 2)
            self.x = (x_c -u0)/fx
            self.y = (y_c -v0)/fy 
            area=(x2-x1)*(y2-y1)
            ratio=(x2-x1)/(y2-y1)
            w=x2-x1
            h=y2-y1
            data1=np.array([area,ratio,w,h,x_c,y_c]).reshape(1,-1)
            dis=self.model.predict(data1)
            #self.Z = (np.sqrt(320*240)-np.sqrt((x2-x1)**2+(y2-y1)**2))
        
            self.Z= dis[0]
            label = self.yolo.names[int(cls)]

        self.latest_rgb = None
        msg1 = Twist()

        L=np.zeros((3,6))
        L[0][0]=-1/self.Z
        L[0][1]=0
        L[0][2]=self.x/self.Z
        L[0][3]=self.x*self.y
        L[0][4]=-1-self.x*self.x
        L[0][5]=self.y

        L[1][1]=-1/self.Z
        L[1][0]=0
        L[1][2]=self.y/self.Z
        L[1][4]=-self.x*self.y
        L[1][3]=1+self.y*self.y
        L[1][5]=-self.x

        L[2][0]=0
        L[2][1]=0
        L[2][2]=-1
        L[2][3]=0
        L[2][4]=0
        L[2][5]=0

        if label=="car" or label=="truck" or label=="bus":
            L_star=np.linalg.inv((L.T @ L)+0.01*np.eye(6)) @ L.T
            e=np.array([x_c-160,y_c-40,0.05*(np.sqrt(self.Z**2-self.pos[2]**2)-30.0)])
            V=-2*(L_star) @ e


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

            
            V0linear=R_total @ V[:3]
            V0angular=R_total @ V[3:6]
            msg1.linear.x=V0linear[0]
            msg1.linear.y=V0linear[1]
            msg1.linear.z=0.0

            msg1.angular.x=0.0
            msg1.angular.y=0.0
            msg1.angular.z=0.0005*V0angular[2]
            print(x_c)
            print(y_c)
            print(self.Z)
            self.publisher_.publish(msg1)
        else:
            V=np.array([1.0,1.0,0.0])
            V1=self.Rot @ V
            msg1.linear.y =  0.0
            msg1.linear.x =  0.0
            msg1.linear.z =  0.0

            msg1.angular.y =  0.0
            msg1.angular.x =  0.0
            msg1.angular.z =  0.0

            #print(self.Rot)
            self.publisher_.publish(msg1)

def main(args=None):
    rclpy.init(args=args)
    node = DroneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
