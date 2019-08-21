#!/usr/bin/env python
import rospy
import time
from getch import getch

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String

class SimpleJetbot:
    def __init__(self):
        rospy.Subscriber('~cmd_str', String, self.on_cmd_str)

        # setup motor controller
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)
        self.motor_left_ID = 1
        self.motor_right_ID = 2
        self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
        self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)

    # sets motor speed between [-1.0, 1.0]
    def set_speed(self, motor_ID, value):
        max_pwm = 115.0
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

        if motor_ID == 1:
            motor = self.motor_left
        elif motor_ID == 2:
            motor = self.motor_right
        else:
            rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
            return

        motor.setSpeed(speed)

        if value > 0:
            motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            motor.run(Adafruit_MotorHAT.BACKWARD)


    # stops all motors
    def all_stop(self):
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)

        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)


    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

        if msg.data.lower() == "left":
            self.set_speed(self.motor_left_ID,  1.0)
            self.set_speed(self.motor_right_ID, -1.0)
        elif msg.data.lower() == "right":
            self.set_speed(self.motor_left_ID,  -1.0)
            self.set_speed(self.motor_right_ID,  1.0)
        elif msg.data.lower() == "forward":
            self.set_speed(self.motor_left_ID,   -1.0)
            self.set_speed(self.motor_right_ID,  -1.0)
        elif msg.data.lower() == "backward":
            self.set_speed(self.motor_left_ID,  1.0)
            self.set_speed(self.motor_right_ID, 1.0)
        elif msg.data.lower() == "stop":
            self.all_stop()
        else:
            rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)

    def spin(self):
        while not rospy.is_shutdown():
            key = ord(getch())
            message = ""
            if key in {83,115}: #S or s
                message = "FORWARD"
                self.set_speed(self.motor_left_ID,   -1.0)
                self.set_speed(self.motor_right_ID,  -1.0)
            elif key in {81,113}: #Q or q
                message = "STOP"
                self.all_stop()
            elif key == 3: #Ctr+C
                raise KeyboardInterrupt
            else:
                pass
            if message:
                message = message + " is pressed"
                print(message)

# initialization
if __name__ == '__main__':

    # setup ros node
    rospy.init_node('jetbot_motors')
    jetbot = SimpleJetbot()

    # stop the motors as precaution
    jetbot.all_stop()

    # start running
    #rospy.spin()
    try:
        jetbot.spin()
    except rospy.ROSInterruptException:
        # stop motors before exiting
        jetbot.all_stop()
    except KeyboardInterrupt:
        jetbot.all_stop()
