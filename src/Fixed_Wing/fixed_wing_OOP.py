#!/usr/bin/python3.8
#
import rospy
import math
import threading
import random
from time import sleep
from mavros_msgs.msg import State, Waypoint, CommandCode, WaypointReached, OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool, WaypointPush, SetModeRequest, CommandBoolRequest, WaypointPushRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
# or from geometry_msgs.msg import PoseStamped

class fixed_wing():
    WP_OFFSET = 0
    
    def wait(func):
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)
            self.rate.sleep()
            return result
        return wrapper
    
    def haversine(self,lat1, lon1, lat2, lon2):
        # Radius of the Earth in kilometers
        R = 6371.0

        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Difference in coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        return distance
    
    
    def __init__(self,landing_lat,landing_lon, landing_alt=8, landing_bearing=0, base_lat=0, base_lon=0):
        print("Fixed Wing UAV object created")
        self.wp_ptr = 0
        self.drop_ptr = 0
        self.waypoint_reached = False
        self.current_state = State()
        self.pose = PoseStamped()
        self.current_altitude = 0
        self.current_latitude = 0
        self.current_longitude = 0
        self.local_pos_pub = None
        self.offboard_setpoint_thread_running = False
        self.took_off = False
        self.rate = None 
        self.waypoints_sent = False
        #starting gps, we will get this from pose later.
        self.base_lat = 0 # grab from pose
        self.base_lon = 0 # grab from pose
        self.takeoff_altitude = 25  # Desired altitude for takeoff in meters
        self.cruise_altitude = 25   # Desired cruising altitude in meters
        self.landing_pos = (landing_lat, landing_lon, landing_alt)  # Desired landing position (latitude, longitude, altitude
        self.landing_bearing = landing_bearing
        self.RC_Control = OverrideRCIn()
        self.waypoints = [] # List to hold waypoints
        self.drops = []
        self.last_wp = -1
    
    @wait
    def waypoint_reached_cb(self,msg):
        self.wp_ptr = msg.wp_seq
        self.waypoint_reached = True
    
    def state_cb(self, msg):
        self.current_state = msg
        
    def offboard_setpoint_publishing_thread(self):
        while not rospy.is_shutdown() and self.offboard_setpoint_thread_running:
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
            
    def gps_cb(self,msg):
    # Assuming altitude is directly available
        self.current_pose = msg
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
    
    
    @wait
    def send_pwm(self, pin, value):
        self.RC_Control.channels[pin] = value
        self.rc_override.publish(self.RC_Control)
     
    def create_waypoint(self,latitude, longitude, altitude,param1=0, cmd=CommandCode.NAV_WAYPOINT,param2=5,param3=0,param4=0):
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = cmd
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = 0  # hold time
        wp.param2 = 0  # acceptance radius
        wp.param3 = 0  # pass through waypoint
        wp.param4 = 0  # yaw angle
        wp.x_lat = latitude
        wp.y_long = longitude
        wp.z_alt = altitude
        return wp
    
    def drop_bottle(self):
        # Implement bottle dropping logic here
        rospy.loginfo("Dropping bottle")
        self.drop_package()
        
    def initialize_connect_ros(self):
        rospy.init_node('waypoint_mission_node', anonymous=True)
        #Need to initialize node before rate
        self.rate = rospy.Rate(20.0)
        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.waypoint_reached_cb)

        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection")
            self.rate.sleep()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/mission/push')

        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.wp_push = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
        
        self.rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.mission_set_mode = SetModeRequest()
        self.mission_set_mode.custom_mode = 'AUTO.MISSION'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        # Start the setpoint publishing thread
        self.offboard_setpoint_thread_running = True
        self.setpoint_thread = threading.Thread(target=self.offboard_setpoint_publishing_thread)
        self.setpoint_thread.start()
        
        while(self.current_latitude == 0):
            rospy.loginfo("Waiting for GPS fix")
            self.rate.sleep()
            
    
    #based on landing bearing, puts a waypoint 200m behind the landing point. loiter to 10m 

    def calculate_destination_point(self, lat, lon, bearing, distance):
        R = 6371000  # radius of the Earth in meters
        bearing_rad = math.radians(bearing)

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                                math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
        new_lon_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                           math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat_rad))

        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)

        return new_lat, new_lon

    def get_landing_approach_waypoint(self,lat, lon, bearing_to_waypoint, distance_behind=200, loiter_altitude=10):
        # Reverse the bearing by adding 180 degrees (or π radians)
        reverse_bearing = (bearing_to_waypoint + 180) % 360
        distance_behind = 200  # distance in meters

        lat, lon = self.calculate_destination_point(lat, lon, reverse_bearing, distance_behind)
        return self.create_waypoint(lat, lon, loiter_altitude, cmd=CommandCode.NAV_LOITER_TO_ALT)
        
    
    def disarm(self):
        if self.current_state.armed:
            self.arm_cmd.value = False
            if self.arming_client.call(self.arm_cmd).success:
                rospy.loginfo("Vehicle disarmed")
        
    def parse_launch_file(self, file_path):
        #parse the launch file and return the waypoints
        wp_param_list = []
        drop = []
        file = open(file_path, 'r')
        content = file.readlines()
        file.close()
        for line in content:
            line = line.rstrip()
            vals = line.split(', ')
            if len(line) > 0:
                vals = line.split(', ')
                Lat = float(vals[0])
                Long = float(vals[1])
                Alt = int(vals[2])
                Operation = int(vals[3])
                Drop = int(vals[4])
                wp = self.create_waypoint(Lat, Long, Alt)
                wp_param_list.append(wp)
                drop.append(Drop)
        return wp_param_list, drop
    
    def prepare_wp_mission(self, file_path):
        #read the text file and parse it
        wp_params, self.drop = self.parse_launch_file(file_path)
        # Create a takeoff waypoint
        if self.base_lat <=0 or self.base_lon <= 0:
            self.base_lat = self.current_latitude
            self.base_lon = self.current_longitude
        self.takeoff_wp = self.create_waypoint(self.base_lat, self.base_lon, self.takeoff_altitude, cmd=CommandCode.NAV_TAKEOFF)
        self.waypoints.append(self.takeoff_wp)
        
        # Append the rest of the waypoints
        for i in range(len(wp_params)):
            self.waypoints.append(wp_params[i])

        landing_wp = self.create_waypoint(self.landing_pos[0], self.landing_pos[1], self.landing_pos[2])
        landing_wp.command = CommandCode.NAV_LAND
        land_approach = self.get_landing_approach_waypoint(landing_wp.x_lat, landing_wp.y_long, self.landing_bearing)
        self.waypoints.append(land_approach)
        self.waypoints.append(landing_wp)
   
    def arm(self):
        if not self.current_state.armed:
            self.arm_cmd.value = True
            if self.arming_client.call(self.arm_cmd).success:
                rospy.loginfo("Vehicle armed")      
    
    def start_wp_mission(self):
        if self.current_state.armed and not self.waypoints_sent:
            wp_push_request = WaypointPushRequest()
            wp_push_request.start_index = 0
            wp_push_request.waypoints = self.waypoints
            self.rate.sleep()
            wp_push_response = self.wp_push(wp_push_request)
            self.rate.sleep()
            self.waypoints_sent = True
            if wp_push_response.success:
                rospy.loginfo("Waypoints successfully pushed")
                self.set_mode_client.call(self.mission_set_mode)
                self.rate.sleep()
            else:
                rospy.loginfo("Failed to push waypoints")
    
    def land(self):
        landing_set_mode = SetModeRequest()
        landing_set_mode.custom_mode = 'AUTO.LAND'
        self.set_mode_client.call(landing_set_mode)
        self.rate.sleep()
                
    def RTL(self):
        rtl_set_mode = SetModeRequest()
        rtl_set_mode.custom_mode = 'RTL_TYPE_AUTO'
        self.set_mode_client.call(rtl_set_mode)
        
    #default threshold is 10m
    def near_waypoint(self,wp, threshold=0.01):
        distance = self.haversine(wp.x_lat, wp.y_long, self.current_latitude, self.current_longitude)
        return distance <= threshold
        
    def wp_handler(self):
        if self.waypoint_reached:
                #self.set_mode_client.call(self.offb_set_mode)
                if self.wp_ptr > fixed_wing.WP_OFFSET and self.drop_ptr < len(self.drop):
                    if self.drop[self.drop_ptr] == 1:
                        print(f"Drop bottle at waypoint {self.drop_ptr}, value: {self.drop[self.drop_ptr]}")
                        self.drop_bottle()
                    self.drop_ptr += 1
                #self.set_mode_client.call(self.mission_set_mode)
                self.waypoint_reached = False

        #function for dropping packages        
    @wait
    def drop_package(self):
        rospy.loginfo("Arrived at dropping location")
        sleep(2)
        self.send_pwm(1, 2000)
        sleep(2)
        rospy.loginfo("Package dropped")
        self.send_pwm(1, 1100)

            
    def main(self,launch_file_path):
        #initialize node, connect to ros
        self.initialize_connect_ros()
        #prepare waypoints (we will update this with our special tect file system later on)
        self.prepare_wp_mission(launch_file_path)
        # Wait for FCU connection
        while not self.current_state.armed:
            self.arm()
            self.rate.sleep()
            
        self.start_wp_mission()
        self.waypoints_sent = False
        
        while not rospy.is_shutdown():
            self.wp_handler()
            #if self.near_waypoint(self.waypoints[len(self.waypoints)-1]):
            #    self.land()
            #    break
            
        self.offboard_setpoint_thread_running = False
        self.setpoint_thread.join()
        
