# -*- coding: utf-8 -*-
import time
import numpy as np

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

class PID():

    #def __init__(self, kp, ki, kd):
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):

        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        #return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error
        return self.Kp*self.p_error + self.Kd*self.d_error + self.Ki*self.i_error
        
def StanleyControl(x, y, yaw, v, index, map_xs, map_ys, map_yaws, L, k):    ## 0908 수정
    #weight = 0.001
    # find nearest waypoint
    min_dist = 1e9
    min_index = index        ## 0908 수정
    n_points = len(map_xs)

    ## 0908 수정
    # 현재 index 기준 index ~ index+3000의 인덱스만 보도록
    low_index = index
    #high_index = index + 3000
    high_index = index + 1000
    # Saturation : low_index가 0보다 작지 않게, high_index가 최대 길이보다 크지 않게
    if low_index < 1000:  
        high_index = 2000        ## 처음 출발선에서 시작할 떄 5000대의 인덱스를 찾아서 특별한 경우 예외처리
    if high_index > n_points:
        high_index = n_points


    # convert rear_wheel x-y coordinate to front_wheel x-y coordinate
    # L = wheel base
    
    front_x = x - L * np.cos(yaw) 
    front_y = y - L * np.sin(yaw) 
    
    
    #front_x = x 
    #front_y = y 
    
    #print("present: ",present_time)
    for i in range(low_index, high_index):                            ## 0908 수정
        # calculating distance (map_xs, map_ys) - (front_x, front_y)
        dx = front_x - map_xs[i]
        dy = front_y - map_ys[i]
        dist = np.hypot(dx, dy)
        #print("time_log: ", time_log[i] - present_time)
        #print("time-present_time: ",time_log[i] - present_time )
        if dist < min_dist:
            min_dist = dist
            min_index = i


    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]

    # nearest x-y coordinate (map_x, map_y) - front_wheel coordinate (front_x, front_y)
    dx = map_x - front_x
    dy = map_y - front_y
    #print("map_time: ", map_time)
    perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)

    # control law
    # heading error = yaw_term

#    print("sensor yaw : ", yaw)
#    print("map yaw : ", map_yaw)
    yaw_term = normalize_angle(map_yaw - yaw)
    cte_term = np.arctan2(k*cte, v)
    if 10700 < index < 11500:
        pid = PID(1.8, 0.0, 0.0)
    elif 15900 < index < 18700:
        pid = PID(4.0, 0.0, 0.0)
    elif 21000 < index < 23000:
        pid = PID(4.0, 0.0, 0.0)
    else:
        pid = PID(1.0, 0.0, 0.0)
        
    cte_term = pid.pid_control(cte_term)
    
    if 24000 < index < 26000:
        cte_term -= 0.006
    if 28000 < index < 30950:
        cte_term += 0.003
    #print("CTE",cte_term*180/np.pi)
#    print("yaw_term",-yaw_term*180/np.pi)
    #rospy.sleep(0.2)
    # steering
    #steer = weight*(-yaw_term)*180/np.pi + (1-weight)*(-cte_term)*180/np.pi
    steer = (-yaw_term)*180/np.pi + cte_term*180/np.pi
    return steer, min_index, (-yaw_term)*180/np.pi

