#! /usr/bin python3
#-*- coding: utf-8 -*-

import numpy as np

class PurePursuit(object):
    def __init__(self, k, ks=0.0, L=1.3):
        self.k = k
        self.ks = ks
        self.L_R = L/2 # 무게중심(GPS)와 rear wheel 거리
        self.L = L
        
        self.Lfc = 2.0 # 최소 LD

        self.old_nearest_point_index = None

    def feedback(self, car_x, car_y, car_yaw, car_v, map_xs, map_ys):
        speed = 2.5
        LD = max(self.k * speed, 1.5) # Lookahead distance
        
        rear_x = car_x - self.L_R * np.cos(car_yaw) # 후륜 x좌표
        rear_y = car_y - self.L_R * np.sin(car_yaw) # 후륜 y좌표

        LD_x = rear_x + LD * np.cos(car_yaw) # Lookahead distance x
        LD_y = rear_y + LD * np.sin(car_yaw) # Lookahead distance y

        LD_idx = None
        min_dist = np.inf
        for i, (map_x, map_y) in enumerate(zip(map_xs, map_ys)):
            _dist = (LD_x - map_x) ** 2 + (LD_y - map_y) ** 2
            if _dist < min_dist:
                min_dist = _dist
                LD_idx = i
            
        map_x = map_xs[LD_idx]
        map_y = map_ys[LD_idx]

        alpha = np.arctan2(map_y - rear_y, map_x - rear_x) - car_yaw

        steer = np.arctan2(2.0 * self.L * np.sin(alpha) / LD, 1.0)

        return LD_x, LD_y, -(steer), speed
    
        # for _x, _y in zip(map_xs, map_ys):
        #     d = (car_x-_x)**2 + (car_y-_y)**2
        #     if d > look_dist**2:
        #         yaw_vec = [self.L * np.cos(car_yaw), self.L * np.sin(car_yaw)]
        #         dist_vec = [_x - rear_x, _y - rear_y]
        #         steer = self.k * (-np.degrees(np.arctan2(2*np.cross(yaw_vec, dist_vec), d)))
        #         return _x, _y, steer, speed
                
    # def search_target_index(self, car_x, car_y, car_yaw, car_v):
    #     rear_x = car_x - self.L  * np.cos(car_yaw)
    #     rear_y = car_y - self.L  * np.sin(car_yaw)

    #     # To speed up nearest point search, doing it at only first time.
    #     if self.old_nearest_point_index is None:
    #         # search nearest point index
    #         dx = [rear_x - icx for icx in self.cx]
    #         dy = [rear_y - icy for icy in self.cy]
    #         d = np.hypot(dx, dy)
    #         ind = np.argmin(d)
    #         self.old_nearest_point_index = ind
    #     else:
    #         ind = self.old_nearest_point_index
    #         distance_this_index = state.calc_distance(self.cx[ind],
    #                                                   self.cy[ind])
    #         while True:
    #             distance_next_index = state.calc_distance(self.cx[ind + 1],
    #                                                       self.cy[ind + 1])
    #             if distance_this_index < distance_next_index:
    #                 break
    #             ind = ind + 1 if (ind + 1) < len(self.cx) else ind
    #             distance_this_index = distance_next_index
    #         self.old_nearest_point_index = ind

    #     LD = self.k * car_v + self.Lfc  # update look ahead distance

    #     # search look ahead target point index
    #     while LD > state.calc_distance(self.cx[ind], self.cy[ind]):
    #         if (ind + 1) >= len(self.cx):
    #             break  # not exceed goal
    #         ind += 1

    #     return ind, LD