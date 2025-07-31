import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from mavros_msgs.msg import PositionTarget

from nav_msgs.msg import Odometry

import numpy as np

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher1')
        #self.publisher_ = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        #timer_period = 0.5                                    /mavros/setpoint_position/local
        #self.timer = self.create_timer(timer_period, self.publish_twist)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.publisher=self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',1)
        self.create_timer(1,self.position_publisher)


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
    def listener_callback(self,msg):


        pos=msg.pose.pose.position
        self.pos=pos
        ori=msg.pose.pose.orientation
        self.ori=ori
        Rot=R.from_quat([ori.x,ori.y,ori.z,ori.w])
        self.Rot=Rot.as_matrix()
        print(self.ori)

    def position_publisher(self):
        msg1=PoseStamped()
        v=np.array([-30.0,25.0,8.0])
        position=v.reshape(-1,1)
        # target=self.Rot @ position 

        # print(target)

        

        msg1.pose.position.x=v[0]
        msg1.pose.position.y=v[1]
        msg1.pose.position.z=v[2]

        msg1.pose.orientation.x=float(0)
        msg1.pose.orientation.y=float(0)
        msg1.pose.orientation.z=float(0)
        msg1.pose.orientation.w=float(1)
        self.publisher.publish(msg1)

def main(args=None):
    rclpy.init(args=args)
    subscriber = TwistPublisher()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
