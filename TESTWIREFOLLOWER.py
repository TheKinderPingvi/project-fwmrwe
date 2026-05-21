#!/usr/bin/env python3
# segment_follower.py

import rospy
import math
import numpy as np
import tf2_ros
import sys
import threading
from collections import deque
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, Image  # добавлен Image
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf.transformations
import argparse
from std_msgs.msg import Bool   # <-- новый импорт

from clover import srv
from std_srvs.srv import Trigger
from config import *

rospy.init_node('TESTWIREFOLLOWER')

# ==============================
# ПАУЗА МИССИИ
# ==============================
pause_requested = False

def pause_callback(msg):
    global pause_requested
    pause_requested = msg.data

rospy.Subscriber('/mission/pause', Bool, pause_callback)

# Паблишер состояния камеры
camera_active_pub = rospy.Publisher('/camera2/active', Bool, queue_size=1, latch=True)

# Прокси сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)

# TF
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Буфер последних точек лидара
live_buffer = deque()
live_buffer_lock = threading.Lock()

# Пустые callback'и для подписок
def lidar_dummy_callback(msg):
    pass

def camera_dummy_callback(msg):
    pass

# Подписка на лидар (исправлено: добавлен callback)
rospy.Subscriber('/velodyne_points', PointCloud2, lidar_dummy_callback)

def get_cable_points(x, y, z_cable, timeout=LIVE_TIMEOUT,
                     y_tolerance=0.15, x_tolerance=0.25, min_points=15):
    now = rospy.Time.now().to_sec()
    candidates = []
    with live_buffer_lock:
        for ts, pt in live_buffer:
            if now - ts > timeout:
                continue
            dx = pt[0] - x
            dy = pt[1] - y
            if abs(dy) > y_tolerance:
                continue
            if dx*dx + dy*dy > LIVE_RADIUS*LIVE_RADIUS:
                continue
            if abs(pt[2] - z_cable) > CABLE_HEIGHT_TOL:
                continue
            candidates.append(pt)
    if len(candidates) < min_points:
        return np.empty((0,3))
    arr = np.array(candidates)
    median_x = np.median(arr[:,0])
    mask = np.abs(arr[:,0] - median_x) < x_tolerance
    filtered = arr[mask]
    if len(filtered) < min_points:
        return np.empty((0,3))
    return filtered

def fly_to_target(target_x, target_y, target_z, cable_height, disable_correction=False):
    rate = rospy.Rate(10)
    navigate(x=target_x, y=target_y, z=target_z, yaw=float('nan'),
             speed=SPEED_HORIZONTAL, frame_id='aruco_map', auto_arm=False)

    log_interval = 0

    while not rospy.is_shutdown():
        # Проверка паузы
        if pause_requested:
            telem = get_telemetry(frame_id='aruco_map')
            navigate(x=telem.x, y=telem.y, z=telem.z, speed=0, frame_id='aruco_map')
            while pause_requested and not rospy.is_shutdown():
                rospy.sleep(0.1)
            # После паузы восстанавливаем полёт к цели
            navigate(x=target_x, y=target_y, z=target_z, yaw=float('nan'),
                     speed=SPEED_HORIZONTAL, frame_id='aruco_map', auto_arm=False)

        telem = get_telemetry(frame_id='aruco_map')
        dx = target_x - telem.x
        dy = target_y - telem.y
        dz = target_z - telem.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < DIST_TOL:
            return True

        if not disable_correction and abs(target_z - (cable_height + TARGET_HEIGHT_ABOVE)) < 0.05:
            points = get_cable_points(telem.x, telem.y, cable_height,
                                      y_tolerance=0.15, x_tolerance=0.25, min_points=15)

            if len(points) >= 15:
                median_x = np.median(points[:,0])
                median_z_cable = np.median(points[:,2])
                std_z = np.std(points[:,2])

                corrected_z = median_z_cable + TARGET_HEIGHT_ABOVE

                if abs(median_x - target_x) > 0.02 and abs(median_x - telem.x) < 0.3:
                    target_x = median_x

                navigate(x=target_x, y=target_y, z=corrected_z,
                         speed=SPEED_HORIZONTAL, frame_id='aruco_map', auto_arm=False)

        log_interval += 1
        rate.sleep()
    return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_x', type=float, required=True)
    parser.add_argument('--target_y', type=float, required=True)
    parser.add_argument('--target_z', type=float, required=True)
    parser.add_argument('--cable_height', type=float, required=True)
    parser.add_argument('--disable_correction', action='store_true', default=False,
                        help='Disable live correction for this segment')
    args = parser.parse_args()

    success = fly_to_target(args.target_x, args.target_y, args.target_z,
                            args.cable_height, args.disable_correction)
    
    sys.exit(0 if success else 1)


    