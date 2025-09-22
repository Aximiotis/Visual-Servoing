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
        self.V1=np.zeros((6,1))
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
        self.annotated_image_pub = self.create_publisher(Image,'/drone/yolo/image_annotated',10)
        self.latest_rgb = None
        self.busx=0
        self.buxy=0
        self.busz=0
        self.e0 = 0
        self.e1 = 0
        self.e2 = 0
        self.Xn=np.zeros((3,8))
        self.Yn=np.zeros((3,1))
        self.A=np.array([0.01,0.02,0.02,0.02,0.03,0.05,0.1,0.7]).T

    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def listener_callback(self,msg):
        
        #orientation and position messages
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
        

        #camera parameters
        u0=160.5
        fx=277.2
        v0=120
        fy=277.2

        x1=0
        x2=0
        self.y=0
        self.x=0
        label="tools"

        #yolo detection bounding box
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


            #data1=np.array([area,ratio,w,h,x_c,y_c]).reshape(1,-1)
            #dis=self.model.predict(data1)
            #self.Z= dis[0]
            self.Z=3*277.2/(h)
            label = self.yolo.names[int(cls)]
            break
        annotated = results.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.annotated_image_pub.publish(annotated_msg)
        self.latest_rgb = None
        msg1 = Twist()

        #jacobian interaction matrix
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

        #controller implementation
        if label=="car" or label=="truck" or label=="bus":
            #Least Squares approach
            L_star= L.T @ np.linalg.inv((L @ L.T)+0.001*np.eye(3))
            e=np.array([self.x,self.y,(self.Z-27)/30])

            #error integration
            self.e0 += 0.0001*e[0]
            self.e1 += 0.0001*e[1]
            self.e2 += 0.0001*e[2]

            #sliding with LS control approach
            saturated=np.array([np.clip(e[0]/1000,-1,1),np.clip(e[1]/1000,-1,1),np.clip(e[2]/1000,-1,1)]).T
            V=-30*(L_star) @ e.T - 60*(L_star) @ np.array([self.e0,self.e1,self.e2]).T  +2000*L_star@ saturated


            #Rotation matrix application
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

            #publishing velocity commands
            msg1.linear.x=np.clip(V0linear[0],-25,25)
            msg1.linear.y=np.clip(V0linear[1],-25,25)
            msg1.linear.z=np.clip(float(V0linear[2]),-0.05,0.05)

            msg1.angular.x=0.0
            msg1.angular.y=0.0
            msg1.angular.z=0.4*np.clip(V0angular[2]/10,-1,1)
            print(x_c)
            print(y_c)
            print(self.Z)

            #filtering 
            self.Xn[:,0:7]=self.Xn[:,1:8]
            self.Xn[:,7]=np.array([V0linear[0],V0linear[1],msg1.angular.z]).T
         
            self.counter+=1
            
            #linear filter application to velocity commands for smoother results
            if self.counter>=9:

                msg1.linear.x=np.clip(float(self.Xn[0,:] @ self.A + 0.05*self.Yn[0]),-25,25)
                self.Yn[0]=msg1.linear.x

                msg1.linear.y=np.clip(float(self.Xn[1,:] @ self.A + 0.05*self.Yn[1]),-25,25)
                self.Yn[1]=msg1.linear.y

                msg1.angular.z=float(self.Xn[2,:] @ self.A + 0.1*self.Yn[2])
                self.Yn[2]=msg1.angular.z

                self.counter=9
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
