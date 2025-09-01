import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from pyproj import Proj, Transformer

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
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.publisher=self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',1)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos
        )
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.rgb_cb, 10)
        self.latest_rgb = None

    def gps_cb(self, msg: NavSatFix):
        """Callback for drone GPS updates"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
    def rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def process(self):

        if self.latest_rgb is None:
            return
        
        x1=0
        x2=0
        self.Y=0
        self.X=0
        x_c=0
        y_c=0
        label="tools"
        results = self.yolo(self.latest_rgb)[0]
        for det in results.boxes.data.cpu().numpy():
            x1, y1, x2, y2, conf, cls = det
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2) / 2)
            X = (x_c )
            Y = (y_c ) 
            Z = 1
            self.Z=Z
            self.X=X
            self.Y=Y
            label = self.yolo.names[int(cls)]

        self.latest_rgb = None

        if label=="car" or label=="truck" or label=="bus":
            print('ok')


        [lat,lon]=self.pixel_to_coordinates(self.current_lat,self.current_lon,20,0,31.5,0,x_c,y_c,320,240,1.05,1.05)
        [X,Y]=self.latlon_to_xy(lat,lon,self.current_lat,self.current_lon)
    
        print([X,Y])
    def lla_to_ecef(self,lat, lon, alt):
        # Convert degrees to radians
        lat = math.radians(lat)
        lon = math.radians(lon)

        # WGS84 constants
        a = 6378137.0          # semi-major axis
        e2 = 6.69437999014e-3  # first eccentricity squared

        N = a / math.sqrt(1 - e2 * (math.sin(lat) ** 2))

        x = (N + alt) * math.cos(lat) * math.cos(lon)
        y = (N + alt) * math.cos(lat) * math.sin(lon)
        z = (N * (1 - e2) + alt) * math.sin(lat)

        return x, y, z
    
    def latlon_to_xy(self,lat, lon, lat0, lon0):

        proj_utm = Proj(proj='utm', zone=int((lon0 + 180) / 6) + 1, ellps='WGS84')
        x0, y0 = proj_utm(lon0, lat0)
        x, y = proj_utm(lon, lat)
        return x - x0, y - y0


    def pixel_to_coordinates(
    # Drone position and orientation
    self,drone_lat, drone_lon, drone_altitude,
    drone_yaw, gimbal_pitch, gimbal_roll,
    # Image and pixel information
    pixel_x, pixel_y, image_width, image_height,
    # Camera properties
    horizontal_fov, vertical_fov
    ):

        # Convert degrees to radians
        drone_lat_rad = math.radians(drone_lat)
        drone_lon_rad = math.radians(drone_lon)
        yaw_rad = math.radians(drone_yaw)
        pitch_rad = math.radians(gimbal_pitch)
        roll_rad = math.radians(gimbal_roll)
    
        # Calculate angular offsets from center of image
        center_x = image_width / 2
        center_y = image_height / 2
    
        # Calculate angular offsets in radians
        # Pixel offset from center normalized to [-0.5, 0.5] range
        x_offset_normalized = (pixel_x - center_x) / image_width
        y_offset_normalized = (pixel_y - center_y) / image_height
    
        # Convert to angular offsets
        x_angle_offset = x_offset_normalized * horizontal_fov
        y_angle_offset = y_offset_normalized * vertical_fov
    
        # Apply offsets to pitch and yaw, accounting for roll
        if abs(roll_rad) < 0.001:  # If roll is close to zero
            effective_yaw_rad = yaw_rad + x_angle_offset
            effective_pitch_rad = pitch_rad + y_angle_offset
        else:
            # With roll, we need to rotate the offset vector
            cos_roll = math.cos(roll_rad)
            sin_roll = math.sin(roll_rad)
        
            # Apply rotation to the angle offsets
            x_rotated = x_angle_offset * cos_roll - y_angle_offset * sin_roll
            y_rotated = x_angle_offset * sin_roll + y_angle_offset * cos_roll
        
            # Apply rotated offsets
            effective_yaw_rad = yaw_rad + x_rotated
            effective_pitch_rad = pitch_rad + y_rotated
    
        # Check if the effective pitch is pointing at or above horizon
        if effective_pitch_rad >= 0:
            return 0, 0
    
        # Calculate the distance to the point on the ground
        ground_distance = drone_altitude / math.tan(abs(effective_pitch_rad))
    
        # Calculate the direction vector components
        dx = ground_distance * math.sin(effective_yaw_rad)
        dy = ground_distance * math.cos(effective_yaw_rad)
    
        # Calculate target point using equirectangular approximation
        earth_radius = 6371000  # Earth radius in meters
    
        # Calculate the change in latitude and longitude
        delta_lat = dy / earth_radius
        delta_lon = dx / (earth_radius * math.cos(drone_lat_rad))
    
        # Convert back to degrees
        target_lat = math.degrees(drone_lat_rad + delta_lat)
        target_lon = math.degrees(drone_lon_rad + delta_lon)
    
        return target_lat, target_lon   


def main(args=None):
    rclpy.init(args=args)
    node = DroneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
