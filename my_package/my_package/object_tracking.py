import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from matplotlib import pyplot as plt
from sklearn.metrics import r2_score


class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher2')
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
        self.subscription
        self.pos=[0,0]
        self.vel=[0,0]
        self.busp=[0,0]
        self.busv=[0,0]
        self.subscriber_ = self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped',self.velocity_listener,10)
        self.subscriber_

        self.posx_list=[]
        self.velx_list=[]
        self.buspx_list=[]
        self.busvx_list=[]
        self.posy_list=[]
        self.vely_list=[]
        self.buspy_list=[]
        self.busvy_list=[]
        self.counter=0
        self.velsubscriber_ambulance = self.create_subscription(Twist, '/ambulance_moving/cmd_vel',self.velocity_ambulance,10)
        self.velsubscriber_ambulance

        self.possubscriber_ambulance = self.create_subscription(Odometry, '/ambulance_moving/odom',self.position_ambulance,10)
        self.possubscriber_ambulance     

    def listener_callback(self,msg):
        self.pos[0]=msg.pose.pose.position.x
        self.pos[1]=msg.pose.pose.position.y
        self.proccess()

    def velocity_listener(self,msg):
        self.vel[0]=msg.linear.x
        self.vel[1]=msg.linear.y

    def position_ambulance(self,msg):
        self.busp[0]=msg.pose.pose.position.x
        self.busp[1]=msg.pose.pose.position.y
        
    def velocity_ambulance(self,msg):
        self.busv[0]=msg.linear.x
        self.busv[1]=msg.linear.y

    def proccess(self):
        self.counter+=1
        self.posx_list.append(self.pos[0])
        self.posy_list.append(self.pos[1])
        self.buspx_list.append(self.busp[0])
        self.buspy_list.append(self.busp[1])

        self.velx_list.append(self.vel[0])
        self.vely_list.append(self.vel[1])
        self.busvx_list.append(self.busv[0])
        self.busvy_list.append(self.busv[1])
        if self.counter==1300:
            plt.figure()
            x=np.arange(0,self.counter,1)
            plt.plot(x,self.posx_list,x,self.buspx_list)
            plt.legend(['Drone Position','Object Position'])
            plt.grid(True)
            plt.title('X axis')
            plt.show()

            plt.figure()
            plt.plot(x,self.posy_list,x,self.buspy_list)
            plt.legend(['Drone Position','Object Position'])
            plt.grid(True)
            plt.title('Y axis')
            plt.show()

            plt.figure()
            plt.plot(x,self.velx_list,x,self.busvx_list)
            plt.legend(['Drone Velocity','Object Velocity'])
            plt.grid(True)
            plt.title('X axis')
            plt.show()

            plt.figure()
            plt.plot(x,self.vely_list,x,self.busvy_list)
            plt.legend(['Drone Velocity','Object Velocity'])
            plt.grid(True)
            plt.title('Y axis')
            plt.show()
            print('ok')
            print(r2_score(self.posx_list,self.buspx_list))
            print(r2_score(self.posy_list,self.buspy_list))
            print(r2_score(self.velx_list,self.busvx_list))
            print(r2_score(self.vely_list,self.busvy_list))

def main(args=None):
    rclpy.init(args=args)
    subscriber = TwistPublisher()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()