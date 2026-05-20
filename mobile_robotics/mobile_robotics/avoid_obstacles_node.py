import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 
import numpy as np  
import signal  
import sys 
 
class LaserScanSub(Node): 
    def __init__(self): 
        super().__init__('closest_object_follower') 
        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 
         
 
        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.d_safety = 0.4 # Distance to keep from the closest object [m] 
        self.d_start_ao = 1.0 #Distance to start the avoid obstacles behavior [m]
        self.kv = 0.4 # Linear speed proportional gain  
        self.kw = 1.0 # Angular speed proportional gain 
        self.kao = 1.0 #Proportional gain for the avoid obstacles behavior
        self.robot_vel = Twist() # The required robot velocity  
        timer_period = 0.1 #10 Hz 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 
     
    def timer_callback(self): 
        if self.lidar.ranges: #Check that you have received at least one message from the lidar  
            #Get the closest object 
            closest_range, theta_closest = self.get_closest_object() 
            if np.isinf(closest_range): 
                print("There are no objects around") 
                v = 0.3 
                w = 0.0 
            else: 
                print("closest_range: ", closest_range) 
                print("theta_closest: ", theta_closest) 
                print("TODO: Add your code here") 
                # TODO: Modify the angular and linear speed values to make your robot avoid the obstacles.  
                if closest_range > self.d_start_ao: 
                    print(" Move to the front ") 
                    v = 0.3 #linear speed [m/s] 
                    w = 0.0 # angular speed [rad/s]
                elif closest_range < self.d_safety:
                    print("Stop, object too close")
                    v = 0.0
                    w = 0.0
                else:  
                    v = 0.3
                    w = 0.0
                    if theta_closest > - np.pi/2 and theta_closest < np.pi/2: #Checking for obstacles only in front of the robot
                        
                        print("Avoid obstacles")
                        theta_ao = theta_closest + np.pi
                        #Crop the angle to -pi to pi
                        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
                        v = 0.3 #Linear speed [m/s]
                        w = self.kao * theta_ao
            self.robot_vel.linear.x = v 
            self.robot_vel.angular.z = w 
            self.cmd_vel_pub.publish(self.robot_vel) 
        else: 
            print("No lidar data received") 
 
    def get_closest_object(self):

        closest_range = min(self.lidar.ranges) 
        closest_index = self.lidar.ranges.index(closest_range) 
        theta_closest = self.lidar.angle_min + closest_index*self.lidar.angle_increment 
        # Crop this angle to (-pi, pi] 
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest)) 
        return closest_range, theta_closest 
 
    def lidar_cb(self, lidar_msg): 
        ## This function receives the ROS LaserScan message 
        self.lidar =  lidar_msg  
     
    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = Twist()  # All zeros to stop the robot 
        self.cmd_vel_pub.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 
 
 
def main(args=None): 
    rclpy.init(args=args) 
    m_p=LaserScanSub() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 
     
if __name__ == '__main__': 
    main()