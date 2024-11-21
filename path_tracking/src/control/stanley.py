#! /usr/bin python
#-*- coding: utf-8 -*-

import numpy as np
import time

class Stanley(object):
    def __init__(self, k, ks=0.0, kd=0.0, L=2.8, k_yaw=0.5,
                  ld_long=3.0, ld_lat=0.5, scaling_factor=0.5, max_speed=1.5, min_speed=1.0):
        self.k = k
        self.k_yaw = k_yaw
        self.ks = ks
        self.kd = kd
        self.prev_yaw_term = 0
        self.L = L

        self.ld_long = ld_long
        self.ld_lat = ld_lat
        self.scaling_factor = scaling_factor
        self.max_speed = max_speed
        self.min_speed = min_speed

        self.prev_steer = 0
        self.stop_time = None

    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def feedback(self, x, y, yaw_, v, map_xs, map_ys, map_yaws, gear=0):
        # car yaw
        if gear == 2:
            yaw = self.normalize_angle(yaw_ + np.pi)
        else: yaw = yaw_

        # find nearest point
        min_dist = 1e9
        min_index = 0
        n_points = len(map_xs)

        front_x = x + self.L  * np.cos(yaw)
        front_y = y + self.L  * np.sin(yaw)

        LD_lat = self.ld_lat #* v
        LD_long = self.ld_long * v

        min_index_for_lat = 0
        min_dist_for_lat = np.inf

        min_index_for_long = 0
        min_dist_for_long = np.inf

        for i in range(n_points):
            dx = front_x - map_xs[i]
            dy = front_y - map_ys[i]

            map_x = map_xs[i]
            map_y = map_ys[i]
            dist_ = (map_x-front_x)**2 + (map_y-front_y)**2
            if dist_ < min_dist:
                min_dist = dist_
                min_index = i

            map_x_for_lat = map_xs[i] - LD_lat * np.cos(map_yaws[i])
            map_y_for_lat = map_ys[i] - LD_lat * np.sin(map_yaws[i])

            dist_for_lat = np.sqrt((map_x_for_lat-front_x)**2 + (map_y_for_lat-front_y)**2)
            if dist_for_lat < min_dist_for_lat:
                min_dist_for_lat = dist_for_lat
                min_index_for_lat = i

            map_x_for_long = map_xs[i] - LD_long * np.cos(map_yaws[i])
            map_y_for_long = map_ys[i] - LD_long * np.sin(map_yaws[i])
            
            dist_for_long = np.sqrt((map_x_for_long-front_x)**2 + (map_y_for_long-front_y)**2)
            if dist_for_long < min_dist_for_long:
                min_dist_for_long = dist_for_long
                min_index_for_long = i

        # compute cte at front axle
        # print('point length:', n_points)
        try:
            map_x = map_xs[min_index]
            map_y = map_ys[min_index]
            map_yaw = map_yaws[min_index]
            dx = map_x - front_x
            dy = map_y - front_y
        except:
            print("현재 waypoint 0개")
            return 0, 0

        perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
        cte = np.dot([dx, dy], perp_vec)

        map_yaw_lat = map_yaws[min_index_for_lat]
        # control law
        yaw_term = self.k_yaw * self.normalize_angle(map_yaw_lat - yaw)
        cte_term = np.arctan2(self.k*cte, max(v, 1.5) )

        print("yaw, cte", np.degrees(yaw_term), np.degrees(cte_term))
        # print("map yaws", map_yaws)
        print("map yaw vs. map_yaw_lat / car_yaw", map_yaw, map_yaw_lat, yaw)

        if gear == 0:
            # steering
            steer = (yaw_term + cte_term)
            self.prev_yaw_term = yaw_term

            # target_speed
            map_yaw_for_long = map_yaws[min_index_for_long]
            yaw_term_for_long = self.normalize_angle(map_yaw_for_long - yaw)
            speed = self.max_speed - abs(yaw_term_for_long)/self.scaling_factor*(self.max_speed-self.min_speed)
            speed = max(speed, self.min_speed)
            self.stop_time = None

        elif gear == 1:
            if self.stop_time is None:
                self.stop_time = time.time()
            
            if time.time() - self.stop_time < 2:
                steer = self.prev_steer
            else:
                steer = 0

            speed = 0

        elif gear == 2:
            speed = -1.2
            steer = -(yaw_term + cte_term)
            self.stop_time = None

        self.prev_steer = steer
        print("k: ",self.k ,"LD: ",self.ld_lat)

        return -np.degrees(steer), speed

    
