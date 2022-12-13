import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from numpy import pi
class Angle_turner:
    def __init__(self):
        rospy.init_node('driving_controller',anonymous=True)
        
        self.vel_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,queue_size=10)
        self.driving_bool_publisher = rospy.Publisher('/driving_controller/mooving', Bool,queue_size=10)
        
        self.pose_subscriber = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.update_imu)
        self.goal_angle_subscriber = rospy.Subscriber('/sound_identifier/goal_angle', Float32, self.update_goal_angle)
        
        self.imu = Imu()
        self.goal_angle = 0
        self.rate = rospy.Rate(10)
        
        self.is_driving = Bool()
        self.imu.orientation.z
    def update_imu(self, imu):
        self.imu = imu
        self.imu.orientation.z = round(self.imu.orientation.z,4)
    
    def update_goal_angle(self, data):
        goal_angle = Float32()
        goal_angle = data
        rospy.loginfo('\n\nRecieved angle (radians): %f',goal_angle.data)
        turn_start_angle = self.imu.orientation.z
        turn_start_angle = round(turn_start_angle,4)
        rospy.loginfo('Robot is currently at: %f', turn_start_angle)
        # Normalise goal angle
        goal_angle.data = round(goal_angle.data/pi,4)
        rospy.loginfo('Goal angle corrected to: %f',goal_angle.data)
        goal_angle_corrected = goal_angle.data + turn_start_angle
        rospy.loginfo ('Orientation corrected angle is: %f', goal_angle_corrected)
        while goal_angle_corrected > 1:
            goal_angle_corrected -= 2
        while goal_angle_corrected <= -1:
            goal_angle_corrected += 2
        self.goal_angle = goal_angle_corrected
        rospy.loginfo('Corrected for revolutions: %f %s',self.goal_angle, '\n')
        
        
    def angle_distance(self, goal_angle):
        difference = goal_angle - self.imu.orientation.z
        if abs(difference) > 1:
            difference = 2 - difference
        rospy.loginfo('Difference is %f', difference)
        return difference
    
    def angular_vel(self, goal_angle, constant = 2):
        
        angular_vel = constant * self.angle_distance(goal_angle)
        if abs(angular_vel) > 1:
            angular_vel = angular_vel/abs(angular_vel)*2
        elif abs(angular_vel) < 0.1:
            angular_vel = angular_vel/abs(angular_vel)*0.1
        rospy.loginfo('Returning angular velocity: %f',angular_vel)
        return angular_vel
        
    def go_to_goal_angle(self):
        angle_tolerance = 0.01
        
        vel_msg = Twist()
        
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        while not rospy.is_shutdown():
            if abs(self.angle_distance(self.goal_angle)) > angle_tolerance:
                vel_msg.angular.z = self.angular_vel(self.goal_angle)
                self.is_driving.data = True
                self.driving_bool_publisher.publish(self.is_driving)
            else:
                vel_msg.angular.z = 0
                self.is_driving.data = False
                self.driving_bool_publisher.publish(self.is_driving)
            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        turner = Angle_turner()
        turner.go_to_goal_angle()
    except rospy.ROSInterruptException:
        pass
