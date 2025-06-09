#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Robot import Robot
import numpy as np
import SETTINGS
from sensor_msgs.msg import LaserScan
from math import ceil, floor, exp
        


def normalize_angle(angle): return (angle + 180) % 360 - 180

def cost_func_angle(x):
    # x = abs(x)
    # return 0.01 * x if x <= 10 else 0.1 * (x - 10)
    return abs(x)

def cost_func_distance(x):
    # return 0 if x > 7 else 10 - 10/7 * x
    return exp(- (x / 3) ** 2)

def laserscan_map(laserscan : LaserScan):
    angles = np.linspace(0, 2 * np.pi, len(laserscan.ranges))
    distances = np.array(laserscan.ranges)
    ld = [SETTINGS.MAX_RANGE] * 360

    for i in range(len(angles)):
        ld[int(np.rad2deg(angles[i]))%360] = distances[i]
    
    return ld


def calculate_safe_zone(angles, distances):
    
    safe_zone = [SETTINGS.AVOID_RANGE] * 360
    ld = [SETTINGS.AVOID_RANGE] * 360

    for i in range(len(angles)):
        ld[int(np.rad2deg(angles[i]))%360] = distances[i]

    for i in range(-180, 181):
        if 0 < ld[i] < SETTINGS.AVOID_RANGE:
            safe_zone[i] = 0

    temp = np.array(safe_zone)
    for i in range(-180, 180):
        if safe_zone[i] > safe_zone[i + 1]:
            for j in range(floor(i + 1 - np.arctan2(SETTINGS.ROBOT_WIDTH/2, ld[i + 1]) * 180 / np.pi), i + 1):
                temp[j] = 0
        if safe_zone[i] < safe_zone[i + 1]:
            for j in range(i, ceil(i + np.arctan2(SETTINGS.ROBOT_WIDTH/2, ld[i]) * 180 / np.pi) + 1):
                temp[j] = 0
    return temp

def calculate_optimal_psi_d(ld, safe_ld, goal_psi):
    print("GOAL_PSI", goal_psi)

    """
    Cost함수를 적용하여 각도별 Cost를 계산
    목적지 까지의 각도와 각도별 LaserScan 데이터에 대한 함수 사용

    Args:
        LaserScan ld
        Float[] safe_ld
        Float goal_psi

    Returns:
        Cost가 가장 낮은 각도 리턴
    """
    theta_list = [[180, 10000]]

    for i in range(-180, 180):
        if safe_ld[i] > 0:
            cost = (SETTINGS.GAIN_PSI * cost_func_angle(i - goal_psi) + 
                    SETTINGS.GAIN_DISTANCE * cost_func_distance(ld[i]))
            theta_list.append([i, cost])



    final_theta = 0
    final_cost = 100000
    for i in range(len(theta_list)):
        if(theta_list[i][1] < final_cost):
            final_cost = theta_list[i][1]
            final_theta = theta_list[i][0]
        
    print(final_theta)
    return final_theta


def pathplan(robot : Robot, laserscan : LaserScan, goal_x = None, goal_y = None):
    """
    LaserScan 데이터를 바탕으로 최적의 TauX, psi_e 값을 찾는 함수

    Args:
        Robot robot
        Float goal_x
        Float goal_y

    Returns:
        [psi_error, tauX]
    """

    angles = np.linspace(0, 2 * np.pi, len(laserscan.ranges))
    distances = np.array(laserscan.ranges)

    ld = laserscan_map(laserscan)
    
    Goal_Psi = 0
    Goal_Distance = 0


    dx = goal_x - robot.x
    dy = goal_y - robot.y

    Goal_Psi = normalize_angle(np.rad2deg(np.arctan2(dy, dx) - robot.get_theta_rad()))
    Goal_Distance = np.sqrt(np.power(dx, 2) + np.power(dy, 2))

    if len(ld) == 0:
        return [0, 0]

    safe_ld = calculate_safe_zone(angles, distances)

    psi_error = calculate_optimal_psi_d(ld, safe_ld, int(Goal_Psi))
    
    v = 0.2
    return [psi_error, v]

    

def goal_check(robot = Robot(), ld= [], goal_distance = None, goal_psi = None):
    """
    목적지 까지 경로에 장애물이 있는지 판단하는 함수

    Args:
        Robot robot

    Returns:
        장애물이 있는지 판단 결과를 리턴 [Boolean]
    """
    l = goal_distance
    theta = ceil(np.degrees(np.arctan2(SETTINGS.ROBOT_WIDTH/2, l)))

    check_ld = [0] * 360
    isAble = True

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(goal_psi) - 90 + i)
        r = SETTINGS.ROBOT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(ld[angle] == 0):
            continue
        if(r > ld[angle]):
            isAble = False

    for i in range(-theta, theta + 1):
        check_ld[normalize_angle(int(goal_psi) + i)] = l
        if(ld[normalize_angle(int(goal_psi) + i)] < l):
            isAble = False

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(goal_psi) + 90 - i)
        r = SETTINGS.ROBOT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(ld[angle] == 0):
            continue
        if(r > ld[angle]):
            isAble = False

    return isAble

def goal_passed(robot = Robot(), goal_x = 0, goal_y = 0, goal_threshold = 2):
    """
    목적지에 도착했는지 판단하는 함수

    Args:
        Robot 객체
        목적지의 x, y 좌표

    Returns:
        도착 결과를 리턴 [Boolean]
    """
    isPassed = False
    if((robot.position[0] - goal_x)**2 + (robot.position[1] - goal_y)**2 < goal_threshold ** 2):
        isPassed = True
    return isPassed






def rotate(robot = Robot(), psi_d=None):
    """
    psi_error를 받고 tau_x는 0으로 해서 실행

    """
    psi_error=normalize_angle(psi_d - robot.psi)
    return [psi_error,0]