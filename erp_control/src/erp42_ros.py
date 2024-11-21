#!/usr/bin/env python3
# -*- coding: utf8 -*-

import serial
import numpy as np

import rospy
from ackermann_msgs.msg import AckermannDrive

S = 0x53
T = 0x54
X = 0x58
AorM = 0x01
ESTOP = 0x00
GEAR = 0x00
SPEED0 = 0x00
SPEED1 = 0x00
STEER0 = 0X02
STEER1 = 0x02
BRAKE = 0x01
ALIVE = 0
ETX0 = 0x0d
ETX1 = 0x0a
count_alive=0

class ERP42():
    def __init__(self):
        rospy.init_node('ERP42')
        self.cmd_sub = rospy.Subscriber("/erp_command", AckermannDrive, self.callback_cmd)

        # 통신 포트 확인
        self.ser = serial.serial_for_url("/dev/ttyERP", baudrate=115200, timeout=1)
        
        self.gear = 0 # 0: 전진, 2: 후진
        self.target_speed = 0
        self.target_steer = np.radians(+0)
        self.target_brake = 1
        self.timer_erp42 = rospy.Timer(rospy.Duration(0.1), self.timer_callback_erp42)
        return
    
    def callback_cmd(self, cmd_msg):
        self.target_speed = cmd_msg.speed
        self.target_brake = cmd_msg.jerk
        self.target_steer = np.radians(cmd_msg.steering_angle)
        return
    
    def timer_callback_erp42(self, event):
        # 목표 속도가 음수 => 기어 2
        if self.target_speed < 0:
            self.gear = 2
            target_speed = -self.target_speed
        else: # 양수 or 0 => 기어 0
            self.gear = 0
            target_speed = self.target_speed

        self.send_to_ERP42(self.gear, target_speed, self.target_steer, self.target_brake)
        return
    
    def send_to_ERP42(self, gear, speed, steer, brake):
        global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive
        count_alive = count_alive+1
        if count_alive==0xff:
            count_alive=0x00
        
        AorM = self.GetAorM()
        GEAR = self.GetGEAR(gear)
        SPEED0, SPEED1 = self.GetSPEED(speed)
        STEER0, STEER1 = self.GetSTEER(steer)
        BRAKE = self.GetBRAKE(brake)
        ALIVE = count_alive
        
        vals = [S, T, X, AorM, ESTOP,GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
        for i in range(len(vals)):
            self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!

        print(vals)
        print("gear: ", gear)
        print("speed: ", speed)
        print("brake: ", brake)
        print("steer: ", steer)
        print("==============================")
        return
    
    def GetGEAR(self, gear):
        GEAR = gear
        return  GEAR

    def GetSPEED(self, speed):
        if speed > 5.0:
            speed = 5.0
        elif speed <= 0:
            speed = 0
            
        SPEED0 = 0x00
        SPEED = abs(int(speed*36)) # float to integer
        SPEED1 = SPEED # m/s to km/h*10
        
        SPEED0=(SPEED & 0b1111111100000000) >> 8
        SPEED1=SPEED & 0b0000000011111111
        return SPEED0, SPEED1

    def GetSTEER(self, steer): # steer은 rad 값으로 넣어줘야한다.
        # steer의 최대 각도: 약 28도
        # 조향 최대 입력 값 2000 => degree * 71 ~= 2000
        steer=np.rad2deg(steer)*71 # rad/s to degree/s*71

        # 허용 범위를 넘으면 최대, 최소값으로 제한
        if(steer>=2000):
            steer=1999
        elif(steer<=-2000):
            steer=-1999

        steer_max=0b0000011111010000 # +2000
        steer_0 = 0b0000000000000000
        steer_min=0b1111100000110000 # -2000

        if (steer>=0):
            angle=int(steer)
            STEER=steer_0+angle
        else:
            angle=int(-steer)
            angle=2000-angle
            STEER=steer_min+angle

        STEER0=(STEER & 0b1111111100000000) >> 8
        STEER1=STEER & 0b0000000011111111
        return STEER0, STEER1

    def GetBRAKE(self, brake):
        brake = abs(int(brake))
        if brake > 200:
            brake = 200
            
        BRAKE = brake
        return  BRAKE
    
    def GetAorM(self):
        AorM = 0x01
        return  AorM

if __name__ == '__main__':
    erp42 = ERP42()
    rospy.spin()