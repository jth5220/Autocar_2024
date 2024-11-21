#!/usr/bin/env python3

import numpy as np

import rospy
import message_filters

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDrive

import tf
import time 

import matplotlib.pyplot as plt

class PID():
    def __init__(self, kp, ki, kd, offset = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.offset = offset
        
        self.windup_guard = 5
        
        self.integral = 0
        self.error_prev = 0
        # self.time_prev = time.perf_counter()
        return
    
    def update(self, setpoint, measurement):
        # PID calculations
        error = setpoint - measurement
        
        # time_interval = time.perf_counter() - self.time_prev
        time_interval = 0.1
        
        P = error
        
        # self.integral += error
        self.integral += error*(time_interval)
        if error < 0:
            self.integral = 0
            
        # if self.integral < -self.windup_guard:
        #     self.integral = -self.windup_guard
        # elif self.integral > self.windup_guard:
        #     self.integral = self.windup_guard
        
        # D = (error - self.error_prev)
        D = (error - self.error_prev)/time_interval
        
        p_term = self.kp*P
        i_term = self.ki*self.integral
        d_term = self.kd*D
        # calculate manipulated variable - MV 
        output = self.offset + p_term + i_term + d_term
        
        self.error_prev = error
        return output, (p_term, i_term, d_term)

class DataPlot():
    def __init__(self, data_name, title='Basic'):
        self.data = []
        self.timestamp = []
        self.data_name = data_name
        
        self.title = title
        
        self.first_time = None
        
        self.color = ['g','b','r', 'c', 'y', 'm', 'k']
        return
    
    def update(self, data):
        if self.first_time is None:
            self.first_time = time.time()
            
        self.data.append(data)
        self.timestamp.append(time.time() - self.first_time)
        
        return
    
    def draw(self):
        plt.figure(figsize=(10, 8))  # 그래프 크기 설정
        
        datum = np.array(self.data).T
        
        for i, data_name in enumerate(self.data_name):
            plt.plot(self.timestamp, datum[i], label=data_name, color=self.color[i])
            plt.legend()
        
        plt.xlabel('Time')
        plt.ylabel('Data')
        plt.title(self.title)
        
        plt.tight_layout()  # 그래프 간격 조정
        plt.show()
        return
    
    
class TargetSpeedUpdate():
    def __init__(self):        
        self.target_speed = 1.0
        self.alpha = 0.2
        self.target_speed_prev = 0.0
        
        # Graph Plotting
        self.data_plot = DataPlot(data_name=['current speed', 'target speed','Final speed'],\
                                  title='Test for improving the speed rise time')
        self.data_plot_2 = DataPlot(data_name=['P term', 'I term', 'D term'],\
                                  title='PID term')
        # self.data_plot = DataPlot(data_name=['current speed', 'target speed', 'P term', 'I term', 'D term', 'Final speed'],\
        #                           title='Test for improving the speed rise time')
        
        # PID controller
        self.pid = PID(kp=4.0, ki=0.0, kd=0.0)
        
        # ROS
        rospy.init_node('stanley_method', anonymous=True)

        self.speed_sub = rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.callback_speed_PID)
        # self.speed_sub = rospy.Subscriber("/speed", TwistWithCovarianceStamped, self.callback_speed_PID)
        self.cmd_pub = rospy.Publisher("/erp_command", AckermannDrive, queue_size=10)

        self.first_time = time.time()
        return
    
    def callback_speed_PID(self, speed_msg):     
        # Target speed (Low pass filter)
        if time.time()-self.first_time < 1:
            self.target_speed = 0.0
        else:
            self.target_speed = 2.0
        # target_speed = self.alpha*self.target_speed + (1-self.alpha)*self.target_speed_prev
        
        # Target speed (sin 형)
        target_speed = 1 * np.sin(time.time() - self.first_time + np.pi*3/2) + 1.5
            
        self.target_speed_prev = target_speed
        
        # Speed
        cur_speed = np.sqrt(speed_msg.twist.twist.linear.x**2 + speed_msg.twist.twist.linear.y**2)
        
        output, PID_term = self.pid.update(setpoint=target_speed, measurement=cur_speed)
        
        final_speed = target_speed + output
        
        final_break = 1 
        if final_speed > 5.0:
            final_speed = 5.0
            
        elif abs(cur_speed - target_speed) <= 0.3: 
            final_speed = target_speed
            final_break = 1
            
        elif cur_speed >= target_speed: # elif final_speed <= 0:
            final_speed = 0
            final_break = abs(PID_term[0] * 20)
            
        print("측정 속력: ", cur_speed)
        print("원래 목표 속력: ", target_speed)
        print("수정된 목표 속력: ", final_speed)
        print("")
        
        # Data for plot
        self.data_plot.update([cur_speed, target_speed, final_speed])
        self.data_plot_2.update([PID_term[0],  PID_term[1],  PID_term[2]])
        
        cmd_msg = AckermannDrive()
        cmd_msg.speed = final_speed
        cmd_msg.jerk = final_break
        self.cmd_pub.publish(cmd_msg)
        
        return

if __name__ == '__main__':
    target_speed_update = TargetSpeedUpdate()
    
    try :
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally :
        print("Finish")
        target_speed_update.data_plot.draw()
        target_speed_update.data_plot_2.draw()
        