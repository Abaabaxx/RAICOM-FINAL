#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Range,LaserScan
import math
import numpy as np

rospy.init_node("gap")

BUBBLE_RADIUS = 0.01
PREPROCESS_CONV_SIZE = 3
BEST_POINT_CONV_SIZE = 80
MAX_LIDAR_DIST = 5
STRAIGHTS_SPEED = 0.1
CORNERS_SPEED = 0.05
STRAIGHTS_STEERING_ANGLE = np.pi / 18
AHEAD_THREHOLD = 0.05
TRANSVERSE_THREHOLD = 0.02

radians_per_elem = 0.1

def preprocess_lidar(ranges):
    global radians_per_elem
    radians_per_elem = (2 * np.pi) / len(ranges)
    # print(radians_per_elem)
    proc_ranges = np.array(ranges[90:-90])
    proc_ranges = np.convolve(proc_ranges, np.ones(PREPROCESS_CONV_SIZE), 'same') / PREPROCESS_CONV_SIZE
    proc_ranges = np.clip(proc_ranges, 0, MAX_LIDAR_DIST)
    return proc_ranges

def find_max_gap(free_space_ranges):
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
    slices = np.ma.notmasked_contiguous(masked)
    max_len = slices[0].stop - slices[0].start
    chosen_slice = slices[0]
    for sl in slices[1:]:
        sl_len = sl.stop - sl.start
        if sl_len > max_len:
            max_len = sl_len
            chosen_slice = sl
    return chosen_slice.start, chosen_slice.stop

def find_best_point( start_i, end_i, ranges):
    averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(BEST_POINT_CONV_SIZE),'same') / BEST_POINT_CONV_SIZE
    return averaged_max_gap.argmax() + start_i

def get_angle(range_index, range_len):
    global radians_per_elem
    lidar_angle = (range_index - (range_len / 2)) * radians_per_elem
    steering_angle = lidar_angle
    return steering_angle

def process_lidar(ranges):
    proc_ranges = preprocess_lidar(ranges)
    closest = proc_ranges.argmin()
    # print(closest, ranges[closest])
    min_index = closest - BUBBLE_RADIUS
    max_index = closest + BUBBLE_RADIUS
    if min_index < 0: min_index = 0
    if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
    proc_ranges[int(min_index):int(max_index)] = 0
    # print(proc_ranges)
    gap_start, gap_end = find_max_gap(proc_ranges)
    print(gap_start, gap_end)
    best = find_best_point(gap_start, gap_end, proc_ranges)
    # print(ranges[best])
    # print(len(ranges))
    # print(ranges[-90])
    steering_angle = get_angle(best, len(proc_ranges))
    # print(steering_angle)
    # print(len(proc_ranges))

def laser_callback(data):
    process_lidar(data.ranges)
    
if __name__ == '__main__': 
  try:
    scan_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
