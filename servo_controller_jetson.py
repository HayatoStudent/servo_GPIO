import Jetson.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String, Int32, Int16, UInt8
from itolab_senior_car_msgs.msg import Servo
import json

#GPIO.cleanup()

class jetson_operator():
    def __init__ (self):
        print("1")
        self.servo_sub = rospy.Subscriber('servo_cmd', Servo,  servo.get_duty_ratio_callback,queue_size=1)
        #self.servo_sub = rospy.Subscriber("steering_angle", UInt8, servo.get_duty_ratio_callback)
        #self.alert_topic_sub = rospy.Subscriber("alert_topic", String, servo.stop_func_callback)
        # self.alert_topic_sub = rospy.Subscriber("alert_topic", Int16, servo.stop_func_callback)


class servo_controll:
    def __init__ (self):
        print("2")
        self.servo_pin = 18
        self.accel_pin = 15
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        GPIO.setup(self.accel_pin, GPIO.OUT)
        self.accel_flag = True
        
    def get_duty_ratio_callback(self,msg):
        print("3")
        self.servo_angle =  180 - msg.steering
        print(msg)
        print("steering",self.servo_angle)
        # self.servo_angle = 90

        if self.accel_flag:
            duty = self.servo_angle * 10.0 / 180 + 2.5
            pwm = GPIO.PWM(self.servo_pin, 50)
            pwm.start(duty)

            #--accel--

            print("get accel", msg.accel)
            self.accel_value = msg.accel+40

            self.max_accel = 100
            self.min_accel = 50
            if self.max_accel < self.accel_value:
                self.accel_value = self.max_accel
            elif self.min_accel > self.accel_value:
                self.accel_value = self.min_accel
            ac_duty = self.accel_value * 10.0 / 180 + 2.5
            ac_pwm = GPIO.PWM(self.accel_pin, 50)
            ac_pwm.start(ac_duty)
            time.sleep(0.4)
        else:
            print("accel off")
            self.accel_off()

        # motor contol 
        
    
    def accel_off(self):
        off_duty = self.min_accel * 10.0 / 180 + 2.5
        off_pwm = GPIO.PWM(self.accel_pin, 50)
        off_pwm.start(off_duty)
        time.sleep(0.4)


    def OFF_power2relay(self): #モータへ電流を切る函数
        self.accel_flag = False
            
    def ON_power2relay(self): #モータへ電流を流す函数
        self.accel_flag = True

    """   
    def stop_func_callback(self, msg):
        alert_dict= json.loads(msg.data)
        self.stop_sign=int(alert_dict["action"])
        print("subscribe now",msg.data)
            
        if self.stop_sign == 1:
            print("off power")
            self.OFF_power2relay() #stop current to relay
        elif self.stop_sign == 0:
            print("on power")
            self.ON_power2relay()
    """ 

def main():
    rospy.init_node("mbx_jetson_subscriber")
    jt_op = jetson_operator()
    rospy.spin()
    servo.accel_off()


if __name__ == "__main__":
    servo = servo_controll()
    main()
    GPIO.cleanup()
