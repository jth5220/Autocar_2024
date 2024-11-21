#!/usr/bin/env python3

import numpy as np

class RiseTimeImprovement():
    def __init__(self, kp=4.0, ki=0.0, kd=0.0, brake_gain=20):
        # PID controller
        self.pid = PID(kp, ki, kd)
        self.brake_gain = brake_gain
        return
    
    def update(self, target_speed, measurement_speed):
        output, PID_term = self.pid.update(setpoint=target_speed, measurement=measurement_speed)
        final_speed = max(min(target_speed + output, 5.0), 0.0)

        # final_speed = target_speed
        final_break = 1

        # elif (measurement_speed - target_speed) < -1.0:
        #     final_speed += 0.5

        if (measurement_speed - target_speed) <= 0.05 and\
                (measurement_speed - target_speed) >= -0.2: 
            final_speed = target_speed
            
        elif measurement_speed - target_speed > 0 : # 측정 속도 > 목표 속도 / 오버슛 났을 때
            # if measurement_speed >= 2.0:
            #     final_speed = 1. # m/s
            # elif measurement_speed < 2.0:
            #     final_speed = target_speed # ERP 입력 속도가 0이면 ERP 내부에서 급감속시킨다고함
            final_speed = 0
           
           # 0705 이전 
            # if measurement_speed > 3.0:
            #     brake_gain = self.brake_gain * 1.5
            # elif measurement_speed > 2.5:
            #     brake_gain = self.brake_gain * 1.3
            # elif measurement_speed > 2.0:
            #     brake_gain = self.brake_gain * 1.15
            # elif measurement_speed < 1.5:
            #     brake_gain = self.brake_gain * 1.0
            # else:
            brake_gain = self.brake_gain
            print("차이있음,pid텀:",PID_term[0])
            # brake_gain = self.brake_gain
            final_break = abs(PID_term[0] * brake_gain)

            # # 2023년도 버전
            # final_speed = target_speed					
            # final_break = 40 + int( ( measurement_speed - target_speed )*35 ) 
        return final_speed, final_break
    

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
            
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
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