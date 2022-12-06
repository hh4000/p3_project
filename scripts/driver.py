import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


PI = 3.14159265359
class Angle_turner:
    def __init__(self):
        self.imu = Imu()
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.must_turn = False
        self.turn_angle = 0
        self.start_angle = 0
        
        rospy.init_node('Angle_turner', anonymous=True)
        rate = rospy.Rate(60)
        rospy.Subscriber('/tdoa_calculator/desired_angle',Float32,self.send_turn_signal)
        rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.update_imu)
        
    def update_imu(self, imu):
        self.imu = imu
        corrected_angle = self.imu.orientation.z-self.start_angle
        if corrected_angle > 1:
            corrected_angle = 2 - corrected_angle
        elif corrected_angle < -1:
            corrected_angle =-2 - corrected_angle
        
        if (corrected_angle-self.turn_angle/PI)<0.01:
            self.must_turn = False
            self.turn_angle = 10
        rospy.loginfo(corrected_angle)
        if self.must_turn:
            vel = Twist()
            if self.turn_angle >= 0:
                vel.angular.z = 0.5
            elif self.turn_angle < 0:
                vel.angular.z =-0.5
            rospy.loginfo('turning to ' + str(self.turn_angle/PI))
            self.pub.publish(vel)
        
    
    def send_turn_signal(self, angle):
        self.start_angle = self.imu.orientation.z
        self.turn_angle = angle.data
        self.must_turn = True
        rospy.loginfo('\n\nRecieved angle.\n»'+str(self.turn_angle)+"\n")
    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    turner = Angle_turner()
    turner.run()
