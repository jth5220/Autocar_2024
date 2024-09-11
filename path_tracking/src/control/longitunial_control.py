#!/usr/bin/env python3

import numpy as np

class RiseTimeImprovement():
    def __init__(self, kp=4.0, ki=0.0, kd=0.0):
        # PID controller
        self.pid = PID(kp, ki, kd)
        return
    
    def update(self, target_speed, speed):
        output, PID_term = self.pid.update(setpoint=target_speed, measurement=speed)
        final_speed = target_speed + output
        final_break = 1 
        
        if final_speed > 5.0:
            final_speed = 5.0
        
        elif abs(speed - target_speed) <= 0.3: 
            final_speed = target_speed
            final_break = 1
            
        elif speed >= target_speed: # elif final_speed <= 0:
            final_speed = 0
            final_break = abs(PID_term[0] * 20)

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