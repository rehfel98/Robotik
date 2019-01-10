import rospy
import sys
import matplotlib 
matplotlib.use("Agg")
import time
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16, UInt8


# class
class pid:
    # 32 Ticks = 1 Umdrehung 
    
    def __init__(self):
        self.velo = []
        self.error = []
        self.curr_speed = 200
        self.wanted_rpm = 150 # Output value
        self.prev_rpm = 0
        self.ticks = 0 # outp rpm
        self.kp = 0.2
        self.ki = 0.005
        self.rpm_sum = 0
        self.kd = 0.007
        self.prev_time = rospy.get_time()
        self.pulse_sensor_sub = rospy.Subscriber("/ticks", UInt8,self.callback, queue_size=20)
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=20)
        rospy.sleep(1)
        self.drive(self.curr_speed)
        
    def callback(self, data):
        #print(type(data))
        x = int(data.data)
        self.ticks += x
        #jede ti Ticks pruefen
        ti = 96
        if(self.ticks >= ti):
            curr_time = rospy.get_time()
            curr_rpm = self.ticksToRpm(self.ticks, curr_time - self.prev_time)
            self.velo.append(curr_rpm)
            print(curr_rpm)
            curr_error = self.wanted_rpm - curr_rpm
            #P
            delta = self.kp*(curr_error)
            #I
            self.rpm_sum += (curr_error)
            self.error.append(curr_error)
            delta += self.ki*(self.rpm_sum)
            #D
            delta += self.kd*( (curr_rpm - self.prev_rpm))
            print(delta)
            self.drive(int(self.curr_speed + delta))
            self.prev_time = curr_time
            self.prev_rpm = curr_rpm
            self.ticks = 0

    def drive(self, speed):
        self.curr_speed = speed
        self.speed_pub.publish(speed)

    def ticksToRpm(self, ticks, time):
        return ((ticks / 32.0) / time) * 60

    def rpmToTicks(self, rpm, time):
        return ((rpm / 60) * time) * 32

def main(args):
    rospy.init_node('pid', anonymous=True)
    pid = pid()
    
    def rescue():
         f = plt.figure()
         plt.plot(pid.velo)
         plt.plot(pid.error)
         plt.show()
         f.savefig("error.pdf", bbox_inches = "tight")
         print("Stop!")
         pid.drive(0)

    rospy.on_shutdown(rescue)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down!")
    

if __name__ == '__main__':
    main(sys.argv)

