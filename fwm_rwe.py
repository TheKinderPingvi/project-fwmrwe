#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('Agg')

import rospy
import math
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import tf.transformations
import matplotlib.pyplot as plt
from collections import deque
import threading
import subprocess
import sys
import os
import json
import shutil
import glob
from scipy.signal import find_peaks
from std_msgs.msg import Bool

from clover import srv
from std_srvs.srv import Trigger
from config import *

rospy.init_node('autonomous_flight')

# ==============================
# Вычисление порога объединения зон на основе размеров дрона
# ==============================
drone_diagonal = math.sqrt(DRONE_LENGTH**2 + DRONE_WIDTH**2)
MERGE_THRESHOLD = drone_diagonal + 0.3

# ==============================
# ПРОКСИ СЕРВИСОВ
# ==============================
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

pause_requested = False

def pause_callback(msg):
    global pause_requested
    pause_requested = msg.data

rospy.Subscriber('/mission/pause', Bool, pause_callback)

# ==============================
# TF ИНФРАСТРУКТУРА
# ==============================
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
static_broadcaster = tf2_ros.StaticTransformBroadcaster()

def setup_transforms():
    velodyne_transform = TransformStamped()
    velodyne_transform.header.frame_id = "base_link"
    velodyne_transform.child_frame_id = "velodyne"
    velodyne_transform.transform.translation.x = 0.0
    velodyne_transform.transform.translation.y = 0.06
    velodyne_transform.transform.translation.z = 0.08
    q = tf.transformations.quaternion_from_euler(1.571, 1.571, 0)
    velodyne_transform.transform.rotation.x = q[0]
    velodyne_transform.transform.rotation.y = q[1]
    velodyne_transform.transform.rotation.z = q[2]
    velodyne_transform.transform.rotation.w = q[3]
    velodyne_transform.header.stamp = rospy.Time.now()
    static_broadcaster.sendTransform(velodyne_transform)

# ==============================
# СБОР ЛИДАРНЫХ ДАННЫХ
# ==============================
scan_points_forward = []
scan_points_backward = []
scan_active = False
scan_direction = None
scan_lock = threading.Lock()
live_buffer = deque()
live_buffer_lock = threading.Lock()

def lidar_callback(msg):
    try:
        transform = tf_buffer.lookup_transform('aruco_map', msg.header.frame_id, rospy.Time(0))
        cloud_transformed = do_transform_cloud(msg, transform)
        points = pc2.read_points(cloud_transformed, field_names=("x", "y", "z"), skip_nans=True)
        now = rospy.Time.now().to_sec()
        with scan_lock:
            if scan_active and scan_direction is not None:
                for p in points:
                    if p[2] >= MIN_Z:
                        if scan_direction == 'forward':
                            scan_points_forward.append([p[0], p[1], p[2]])
                        elif scan_direction == 'backward':
                            scan_points_backward.append([p[0], p[1], p[2]])
        with live_buffer_lock:
            for p in points:
                if p[2] >= MIN_Z:
                    live_buffer.append((now, [p[0], p[1], p[2]]))
            while live_buffer and live_buffer[0][0] < now - MAX_BUFFER_AGE:
                live_buffer.popleft()
    except Exception:
        pass

rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback)

def get_live_points_near(x, y, z_cable, timeout=LIVE_TIMEOUT):
    now = rospy.Time.now().to_sec()
    result = []
    with live_buffer_lock:
        for ts, pt in live_buffer:
            if now - ts > timeout:
                continue
            dx = pt[0] - x
            dy = pt[1] - y
            if dx*dx + dy*dy > LIVE_RADIUS*LIVE_RADIUS:
                continue
            if abs(pt[2] - z_cable) > CABLE_HEIGHT_TOL:
                continue
            result.append(pt)
    return np.array(result) if result else np.empty((0,3))

# ==============================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ==============================
def log(message, level='info'):
    print(f"[FWM_RWE] {message}", flush=True)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False, tol=0.2):
    # Первый вызов
    res = navigate(x=x, y=y, z=z, yaw=float('nan'),
                   speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        # Проверка паузы
        if pause_requested:
            # Удержание позиции
            telem = get_telemetry(frame_id='aruco_map')
            navigate(x=telem.x, y=telem.y, z=telem.z, speed=0, frame_id='aruco_map')
            # Ждём снятия паузы
            while pause_requested and not rospy.is_shutdown():
                rospy.sleep(0.1)
            # После паузы снова отправляем команду на исходную цель
            res = navigate(x=x, y=y, z=z, yaw=float('nan'),
                           speed=speed, frame_id=frame_id, auto_arm=False)
            if not res.success:
                raise Exception(res.message)

        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tol:
            return
        rospy.sleep(0.2)

def hold_position():
    telem = get_telemetry(frame_id='aruco_map')
    navigate(x=telem.x, y=telem.y, z=telem.z, frame_id='aruco_map', speed=0)
    log("Position hold")

def correct_position(cable_height, stabilization_delay=1.0):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='aruco_map')
        live = get_live_points_near(telem.x, telem.y, cable_height)
        if len(live) >= 5:
            mean_x = np.mean(live[:,0])
            mean_z_cable = np.mean(live[:,2])
            target_z = mean_z_cable + TARGET_HEIGHT_ABOVE
            dx = mean_x - telem.x
            dz = target_z - telem.z
            if abs(dx) < 0.02 and abs(dz) < 0.02:
                break
            navigate(x=mean_x, y=telem.y, z=target_z,
                     speed=0.1, frame_id='aruco_map', auto_arm=False)
        rate.sleep()
    rospy.sleep(stabilization_delay)
    log("Position corrected")

def extract_cables_and_consoles(points):
    """
    Анализирует облако точек, выделяет верхний и нижний кабели (тросы),
    а также консоли (опоры) по гистограмме высот.
    
    Возвращает:
        high_points, low_points, console_points, console_centers, console_info
    """
    if len(points) == 0:
        return None, None, None, [], []
    
    heights = points[:, 2]
    hist, bin_edges = np.histogram(heights, bins=100)
    peaks, _ = find_peaks(hist, distance=10)

    if len(peaks) == 0:
        log("No peaks found in height histogram, aborting", 'error')
        return None, None, None, [], []

    # Координаты центров пиков
    peak_centers = (bin_edges[peaks] + bin_edges[peaks + 1]) / 2
    peak_heights_vals = hist[peaks]

    # Сортируем пики по убыванию "веса" (количества точек)
    sorted_indices = np.argsort(peak_heights_vals)[::-1]
    peak_centers = peak_centers[sorted_indices]
    peak_heights_vals = peak_heights_vals[sorted_indices]

    console_candidates = []
    cable_candidates = []
    for z, cnt in zip(peak_centers, peak_heights_vals):
        # Проверка, относится ли пик к консоли (ожидаемая высота ~1.8 м)
        if abs(z - 1.8) < CONSOLE_HEIGHT_TOL:
            console_candidates.append(z)
        # Всё, что выше 1.7 м, не может быть кабелем (это консоль или шум)
        elif z > 1.7:
            rospy.logdebug(f"Ignoring peak at {z:.2f} m as possible console")
            continue
        else:
            cable_candidates.append(z)

    # Если консоли не найдены, можно использовать fallback-диапазон
    if not console_candidates:
        # В исходном коде был fallback по диапазону 1.75-1.85, оставим его ниже
        pass

    # --- ИЗМЕНЁННЫЙ БЛОК ОПРЕДЕЛЕНИЯ ВЫСОТ ТРОСОВ (используем ожидаемые высоты из config) ---
    # Определяем высоты двух кабелей (верхний и нижний)
    if len(cable_candidates) >= 2:
        # Если заданы ожидаемые высоты, используем их для выбора
        if 'EXPECTED_HIGH_CABLE_HEIGHT' in globals() and 'EXPECTED_LOW_CABLE_HEIGHT' in globals():
            # Находим пики, близкие к ожидаемым высотам (допуск 0.15 м)
            high_candidates = [z for z in cable_candidates if abs(z - EXPECTED_HIGH_CABLE_HEIGHT) < 0.15]
            low_candidates = [z for z in cable_candidates if abs(z - EXPECTED_LOW_CABLE_HEIGHT) < 0.15]
            
            if high_candidates and low_candidates:
                # Выбираем ближайшие к ожидаемым
                high_z = min(high_candidates, key=lambda z: abs(z - EXPECTED_HIGH_CABLE_HEIGHT))
                low_z = min(low_candidates, key=lambda z: abs(z - EXPECTED_LOW_CABLE_HEIGHT))
                log(f"Selected cable heights using expected: high={high_z:.3f}, low={low_z:.3f}")
            else:
                # Fallback на два самых высоких пика
                cable_candidates.sort(reverse=True)
                high_z = cable_candidates[0]
                low_z = cable_candidates[1]
                log(f"Expected heights not matched, using two highest peaks: high={high_z:.3f}, low={low_z:.3f}")
        else:
            # Ожидаемые высоты не заданы, используем два самых высоких пика
            cable_candidates.sort(reverse=True)
            high_z = cable_candidates[0]
            low_z = cable_candidates[1]
            log(f"Detected cable heights: high={high_z:.3f}, low={low_z:.3f}")
    else:
        log("Not enough cable peaks, using expected heights", 'warn')
        high_z = EXPECTED_HIGH_CABLE_HEIGHT if 'EXPECTED_HIGH_CABLE_HEIGHT' in globals() else 1.5
        low_z = EXPECTED_LOW_CABLE_HEIGHT if 'EXPECTED_LOW_CABLE_HEIGHT' in globals() else 1.4
    # -----------------------------------------------------------------------------------

    # Маски для точек
    high_mask = np.abs(heights - high_z) < CABLE_HEIGHT_TOL
    low_mask = np.abs(heights - low_z) < CABLE_HEIGHT_TOL

    # Маска для консолей
    console_mask = np.zeros(len(heights), dtype=bool)
    console_peaks = []
    if console_candidates:
        console_peaks = console_candidates
    else:
        # Fallback: если нет явных пиков консолей, ищем точки в диапазоне CONSOLE_HEIGHT ± 0.05
        console_mask_fallback = (heights > CONSOLE_HEIGHT - 0.05) & (heights < CONSOLE_HEIGHT + 0.05)
        if np.any(console_mask_fallback):
            console_peaks = [np.median(heights[console_mask_fallback])]
            console_mask = console_mask_fallback

    # Расширяем маску консолей на основе найденных пиков
    if console_peaks:
        for cz in console_peaks:
            console_mask |= np.abs(heights - cz) < CONSOLE_HEIGHT_TOL

    # Извлекаем точки по маскам
    high_points = points[high_mask]
    low_points = points[low_mask]
    console_points = points[console_mask]

    # Кластеризация консолей по координате Y для определения их центров и половин ширины
    console_info = []
    if len(console_points) > 0:
        ys = console_points[:, 1]
        ys_sorted = np.sort(ys)
        current_ys = [ys_sorted[0]]
        for y in ys_sorted[1:]:
            if y - current_ys[-1] < CONSOLE_CLUSTER_THRESH:
                current_ys.append(y)
            else:
                if len(current_ys) >= CONSOLE_MIN_POINTS:
                    center = np.mean(current_ys)
                    min_y = np.min(current_ys)
                    max_y = np.max(current_ys)
                    half_width = (max_y - min_y) / 2
                    console_info.append({'center': center, 'half_width': half_width})
                current_ys = [y]
        if len(current_ys) >= CONSOLE_MIN_POINTS:
            center = np.mean(current_ys)
            min_y = np.min(current_ys)
            max_y = np.max(current_ys)
            half_width = (max_y - min_y) / 2
            console_info.append({'center': center, 'half_width': half_width})

    console_centers = [info['center'] for info in console_info]
    return high_points, low_points, console_points, console_centers, console_info

def merge_console_zones(console_info):
    if not console_info:
        return []
    intervals = []
    for info in console_info:
        offset = info['half_width'] + CROSSING_OFFSET - 0.3
        left = info['center'] - offset
        right = info['center'] + offset
        intervals.append((left, right))
    intervals.sort(key=lambda x: x[0])
    merged = []
    current = intervals[0]
    for next_interval in intervals[1:]:
        if next_interval[0] <= current[1]:
            current = (current[0], max(current[1], next_interval[1]))
        else:
            merged.append(current)
            current = next_interval
    merged.append(current)
    result = []
    for (left, right) in merged:
        center = (left + right) / 2
        half = (right - left) / 2
        result.append({'center': center, 'half_width': half})
    return result

def build_waypoints(points, safe_dist=SAFE_DIST, step=WAYPOINT_STEP):
    if len(points) == 0:
        return [], None
    cable_height = np.median(points[:,2])
    y_vals = np.round(points[:,1], 2)
    unique_y = np.unique(y_vals)
    median_x = []
    for y in unique_y:
        mask = (y_vals == y)
        median_x.append(np.median(points[mask,0]))
    sort_idx = np.argsort(unique_y)
    unique_y = unique_y[sort_idx]
    median_x = np.array(median_x)[sort_idx]

    y_min_raw = np.min(points[:,1])
    y_max_raw = np.max(points[:,1])
    y_min = y_min_raw + safe_dist
    y_max = y_max_raw - safe_dist

    if y_min >= y_max:
        log(f"Cable too short (raw range [{y_min_raw:.2f}, {y_max_raw:.2f}]), using midpoint with half safe_dist", 'warn')
        mid = (y_min_raw + y_max_raw) / 2
        half_dist = safe_dist / 2
        y_min = mid - half_dist
        y_max = mid + half_dist
        if y_min >= y_max:
            y_min = y_min_raw
            y_max = y_max_raw
            log("Fallback to raw bounds", 'warn')

    y_grid = np.arange(y_min, y_max + step/2, step)
    if len(y_grid) == 0:
        y_grid = np.array([(y_min + y_max) / 2])

    x_grid = np.interp(y_grid, unique_y, median_x, left=median_x[0], right=median_x[-1])
    z_fixed = cable_height + TARGET_HEIGHT_ABOVE
    return [(x_grid[i], y_grid[i], z_fixed) for i in range(len(y_grid))], cable_height

def find_crossing_point(high_points, low_points):
    if len(high_points) == 0 or len(low_points) == 0:
        return None, None
    high_y = np.round(high_points[:,1], 2)
    low_y = np.round(low_points[:,1], 2)
    unique_y_high = np.unique(high_y)
    unique_y_low = np.unique(low_y)

    y_min = max(np.min(high_points[:,1]), np.min(low_points[:,1]))
    y_max = min(np.max(high_points[:,1]), np.max(low_points[:,1]))
    if y_min >= y_max:
        return None, None

    y_grid = np.arange(y_min, y_max, 0.01)

    x_high_avg = []
    for y in y_grid:
        mask = (high_y == np.round(y,2))
        if np.any(mask):
            x_high_avg.append(np.mean(high_points[mask,0]))
        else:
            idx = np.argmin(np.abs(unique_y_high - y))
            nearest_y = unique_y_high[idx]
            x_high_avg.append(np.mean(high_points[high_y == nearest_y, 0]))
    x_high = np.array(x_high_avg)

    x_low_avg = []
    for y in y_grid:
        mask = (low_y == np.round(y,2))
        if np.any(mask):
            x_low_avg.append(np.mean(low_points[mask,0]))
        else:
            idx = np.argmin(np.abs(unique_y_low - y))
            nearest_y = unique_y_low[idx]
            x_low_avg.append(np.mean(low_points[low_y == nearest_y, 0]))
    x_low = np.array(x_low_avg)

    dist = np.abs(x_high - x_low)
    min_idx = np.argmin(dist)
    y_cross = y_grid[min_idx]
    return y_cross, dist[min_idx]

def build_avoidance_waypoints(original_wps, cable_height, y_cross, safe_altitude, offset):
    if y_cross is None:
        return original_wps, None, None

    zone_min = y_cross - offset
    zone_max = y_cross + offset

    before = [wp for wp in original_wps if wp[1] < zone_min]
    inside = [wp for wp in original_wps if zone_min <= wp[1] <= zone_max]
    after = [wp for wp in original_wps if wp[1] > zone_max]

    log(f"Zone {y_cross:.2f} (offset {offset:.2f}): before={len(before)}, inside={len(inside)}, after={len(after)}")

    if not inside:
        return original_wps, None, None

    if not before and original_wps:
        leftmost = min(original_wps, key=lambda wp: wp[1])
        if leftmost[1] < zone_min:
            before = [leftmost]
            log(f"Added leftmost point at Y={leftmost[1]:.2f} to before")

    if before:
        last_before = before[-1]
        first_inside = inside[0]
        if last_before[1] < zone_min < first_inside[1]:
            t = (zone_min - last_before[1]) / (first_inside[1] - last_before[1])
            x_in = last_before[0] + t * (first_inside[0] - last_before[0])
        else:
            x_in = first_inside[0]
    else:
        if len(inside) >= 2:
            y1, x1 = inside[0][1], inside[0][0]
            y2, x2 = inside[1][1], inside[1][0]
            t = (zone_min - y1) / (y2 - y1) if y2 != y1 else 0
            x_in = x1 + t * (x2 - x1)
        else:
            x_in = inside[0][0]

    if after:
        last_inside = inside[-1]
        first_after = after[0]
        if last_inside[1] < zone_max < first_after[1]:
            t = (zone_max - last_inside[1]) / (first_after[1] - last_inside[1])
            x_out = last_inside[0] + t * (first_after[0] - last_inside[0])
        else:
            x_out = last_inside[0]
    else:
        if len(inside) >= 2:
            y1, x1 = inside[-2][1], inside[-2][0]
            y2, x2 = inside[-1][1], inside[-1][0]
            t = (zone_max - y1) / (y2 - y1) if y2 != y1 else 1
            x_out = x1 + t * (x2 - x1)
        else:
            x_out = inside[0][0]

    new_wps = list(before)
    new_wps.append((x_in, zone_min, safe_altitude))
    for wp in inside:
        new_wps.append((wp[0], wp[1], safe_altitude))
    new_wps.append((x_out, zone_max, cable_height + TARGET_HEIGHT_ABOVE))
    new_wps.extend(after)
    return new_wps, (x_in, zone_min), (x_out, zone_max)

def plot_scan_data(high, low, console_points, waypoints_high, waypoints_low,
                   y_cross=None, merged_zones=None,
                   start_point=None, end_point=None,
                   return_points_low=None,
                   crossing_start_line=None, first_console_end_line=None,
                   is_merged_case=False,
                   filename='newroutes/scan_route.png'):
    try:
        plt.figure(figsize=(12, 8))
        if len(high) > 0:
            plt.scatter(high[:,1], high[:,0], c='red', s=1, label='High cable', alpha=0.6)
        if len(low) > 0:
            plt.scatter(low[:,1], low[:,0], c='blue', s=1, label='Low cable', alpha=0.6)
        if len(console_points) > 0:
            plt.scatter(console_points[:,1], console_points[:,0], c='green', s=1, label='Consoles', alpha=0.8)
        if waypoints_high:
            wx_h = [wp[1] for wp in waypoints_high]
            wy_h = [wp[0] for wp in waypoints_high]
            plt.plot(wx_h, wy_h, 'r--', linewidth=2, label='High route')
            plt.scatter(wx_h, wy_h, c='red', s=20, marker='o')
        if waypoints_low:
            wx_l = [wp[1] for wp in waypoints_low]
            wy_l = [wp[0] for wp in waypoints_low]
            plt.plot(wx_l, wy_l, 'b--', linewidth=2, label='Low route')
            plt.scatter(wx_l, wy_l, c='blue', s=20, marker='o')

        if return_points_low is not None and len(return_points_low) > 0:
            rx = [p[1] for p in return_points_low]
            ry = [p[0] for p in return_points_low]
            plt.scatter(rx, ry, c='yellow', s=15, marker='o', label='Return points', zorder=6, edgecolors='black')

        if merged_zones is not None and len(merged_zones) > 0:
            for i, zone in enumerate(merged_zones):
                left = zone['center'] - zone['half_width']
                right = zone['center'] + zone['half_width']
                plt.axvline(x=left, color='magenta', linestyle='--', linewidth=1, alpha=0.7)
                plt.axvline(x=right, color='magenta', linestyle='--', linewidth=1, alpha=0.7)
                if i == 0:
                    plt.axvline(x=left, color='magenta', linestyle='--', linewidth=1, alpha=0.7, label='Console zone')

        if y_cross is not None and not is_merged_case:
            plt.axvline(x=y_cross, color='green', linestyle=':', linewidth=1, label='Crossing Y')
            zone_min = y_cross - CROSSING_OFFSET
            zone_max = y_cross + CROSSING_OFFSET
            plt.axvline(x=zone_min, color='orange', linestyle='--', linewidth=1, label='Crossing zone start')
            plt.axvline(x=zone_max, color='orange', linestyle='--', linewidth=1, label='Crossing zone end')

        if start_point:
            plt.scatter(start_point[1], start_point[0], c='orange', s=100, marker='*',
                        edgecolors='black', linewidths=1, label='Avoid start', zorder=5)
        if end_point:
            plt.scatter(end_point[1], end_point[0], c='purple', s=100, marker='*',
                        edgecolors='black', linewidths=1, label='Avoid end', zorder=5)

        if crossing_start_line is not None:
            plt.axvline(x=crossing_start_line, color='yellow', linestyle='--', linewidth=2, label='Merged zone start')
        if first_console_end_line is not None:
            plt.axvline(x=first_console_end_line, color='purple', linestyle='--', linewidth=2, label='Merged zone end')

        plt.xlabel('Y (m)')
        plt.ylabel('X (m)')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.axis('equal')
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        log("The route is displayed on the screen")
    except Exception as e:
        log(f"Plot error: {e}")

def split_into_segments(waypoints, cable_height):
    targets = []
    current_segment = []
    for wp in waypoints:
        if abs(wp[2] - (cable_height + TARGET_HEIGHT_ABOVE)) < 0.05:
            current_segment.append(wp)
        else:
            if current_segment:
                targets.append(current_segment[-1])
                current_segment = []
    if current_segment:
        targets.append(current_segment[-1])
    return targets

def run_segment_follower(target_x, target_y, target_z, cable_height, disable_correction=False):
    cmd = [
        sys.executable,
        "TESTWIREFOLLOWER.py",                     # имя файла в текущей папке
        "--target_x", str(target_x),
        "--target_y", str(target_y),
        "--target_z", str(target_z),
        "--cable_height", str(cable_height)
    ]
    if disable_correction:
        cmd.append("--disable_correction")
    proc = subprocess.run(cmd)
    return proc.returncode == 0

def build_mission_structure(waypoints, cable_height, obstacle_centers):
    target_z = cable_height + TARGET_HEIGHT_ABOVE
    zones = []
    segments = []
    current_segment = []
    current_zone = []
    in_segment = False
    in_zone = False
    for wp in waypoints:
        if abs(wp[2] - target_z) < 0.05:
            if in_zone and current_zone:
                center_y = np.mean([p[1] for p in current_zone])
                zones.append({
                    'center_y': center_y,
                    'enter_point': current_zone[0],
                    'exit_point': current_zone[-1],
                    'points': current_zone.copy()
                })
                current_zone = []
                in_zone = False
            if not in_segment:
                in_segment = True
                current_segment = [wp]
            else:
                current_segment.append(wp)
        else:
            if in_segment and current_segment:
                segments.append(current_segment.copy())
                current_segment = []
                in_segment = False
            if not in_zone:
                in_zone = True
                current_zone = [wp]
            else:
                current_zone.append(wp)
    if in_segment and current_segment:
        segments.append(current_segment)
    if in_zone and current_zone:
        center_y = np.mean([p[1] for p in current_zone])
        zones.append({
            'center_y': center_y,
            'enter_point': current_zone[0],
            'exit_point': current_zone[-1],
            'points': current_zone.copy()
        })

    start_point = segments[0][0] if segments else None
    return {
        'start_point': start_point,
        'segments': segments,
        'obstacle_zones': zones
    }

def build_route_with_returns(base_waypoints, cable_height, zones, offsets, safe_altitude,
                             return_targets=None, extra_forward=None, takeoff_at_current=None):
   
    if not base_waypoints:
        return [], []
    n = len(zones)
    if return_targets is None:
        return_targets = [zone + offset for zone, offset in zip(zones, offsets)]
    if extra_forward is None:
        extra_forward = [False] * n
    if takeoff_at_current is None:
        takeoff_at_current = [False] * n

    base_y = [wp[1] for wp in base_waypoints]
    base_x = [wp[0] for wp in base_waypoints]
    y_min = base_y[0]
    y_max = base_y[-1]
    step = WAYPOINT_STEP
    target_z = cable_height + TARGET_HEIGHT_ABOVE

    def get_x(y):
        if y <= base_y[0]:
            return base_x[0]
        if y >= base_y[-1]:
            return base_x[-1]
        idx = np.searchsorted(base_y, y)
        if idx == 0:
            return base_x[0]
        y1, y2 = base_y[idx-1], base_y[idx]
        x1, x2 = base_x[idx-1], base_x[idx]
        return x1 + (x2 - x1) * (y - y1) / (y2 - y1)

    new_wps = []
    return_points = []
    current_y = y_min

    for i, (zone_center, offset) in enumerate(zip(zones, offsets)):
        zone_min = zone_center - offset
        zone_max = zone_center + offset
        ret_target = return_targets[i]
        extra = extra_forward[i]
        takeoff_curr = takeoff_at_current[i]

        if zone_max < y_min:
            continue
        if zone_min > y_max:
            break

        zone_min = max(zone_min, y_min)
        zone_max = min(zone_max, y_max)
        ret_target = max(y_min, min(y_max, ret_target))

        # Движение к началу зоны на рабочей высоте (если не takeoff_curr)
        if not takeoff_curr:
            if current_y < zone_min:
                y_vals = np.arange(current_y + step, zone_min + step/2, step)
                for y in y_vals:
                    if y > zone_min + 1e-6:
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, target_z))
                current_y = zone_min
            elif current_y > zone_min:
                y_vals = np.arange(current_y - step, zone_min - step/2, -step)
                for y in y_vals:
                    if y < zone_min - 1e-6:
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, target_z))
                    return_points.append((x, y, target_z))
                current_y = zone_min

        # Подъём на безопасную высоту (с предварительным челноком, если takeoff_curr)
        if takeoff_curr:
            # Челночный проход от current_y до zone_min и обратно на рабочей высоте
            if abs(current_y - zone_min) > 1e-6:
                # Движение к zone_min
                if current_y < zone_min:
                    y_to_min = np.arange(current_y + step, zone_min + step/2, step)
                else:
                    y_to_min = np.arange(current_y - step, zone_min - step/2, -step)
                for y in y_to_min:
                    if (current_y < zone_min and y > zone_min + 1e-6) or (current_y > zone_min and y < zone_min - 1e-6):
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, target_z))
                    return_points.append((x, y, target_z))
                # Движение обратно к current_y
                if zone_min < current_y:
                    y_back = np.arange(zone_min + step, current_y + step/2, step)
                else:
                    y_back = np.arange(zone_min - step, current_y - step/2, -step)
                for y in y_back:
                    if (zone_min < current_y and y > current_y + 1e-6) or (zone_min > current_y and y < current_y - 1e-6):
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, target_z))
                    return_points.append((x, y, target_z))
                # После челнока остаёмся в current_y
            # Подъём в current_y
            x_curr = get_x(current_y)
            new_wps.append((x_curr, current_y, safe_altitude))
            # Полёт на безопасной высоте от current_y до zone_max
            if current_y < zone_max:
                y_safe = np.arange(current_y + step, zone_max + step/2, step)
                for y in y_safe:
                    if y > zone_max + 1e-6:
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, safe_altitude))
            else:
                y_safe = np.arange(current_y - step, zone_max - step/2, -step)
                for y in y_safe:
                    if y < zone_max - 1e-6:
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, safe_altitude))
        else:
            # Стандартный подъём в начале зоны
            x_in = get_x(zone_min)
            new_wps.append((x_in, zone_min, safe_altitude))
            # Полёт внутри зоны от zone_min до zone_max
            y_inside = np.arange(zone_min + step, zone_max + step/2, step)
            for y in y_inside:
                if y > zone_max + 1e-6:
                    continue
                x = get_x(y)
                new_wps.append((x, y, safe_altitude))

        # Спуск на рабочую высоту в конце зоны
        x_out = get_x(zone_max)
        new_wps.append((x_out, zone_max, target_z))

        # Возврат к цели, если требуется
        if abs(ret_target - zone_max) > 1e-6:
            if ret_target > zone_max:
                y_vals = np.arange(zone_max + step, ret_target + step/2, step)
            else:
                y_vals = np.arange(zone_max - step, ret_target - step/2, -step)
            for y in y_vals:
                if (ret_target > zone_max and y > ret_target + 1e-6) or \
                   (ret_target < zone_max and y < ret_target - 1e-6):
                    continue
                x = get_x(y)
                new_wps.append((x, y, target_z))
                return_points.append((x, y, target_z))
            current_y = ret_target

            # Дополнительный проход вперёд до zone_max, если нужно
            if extra:
                if zone_max > ret_target:
                    y_vals = np.arange(ret_target + step, zone_max + step/2, step)
                else:
                    y_vals = np.arange(ret_target - step, zone_max - step/2, -step)
                for y in y_vals:
                    if (zone_max > ret_target and y > zone_max + 1e-6) or \
                       (zone_max < ret_target and y < zone_max - 1e-6):
                        continue
                    x = get_x(y)
                    new_wps.append((x, y, target_z))
                    return_points.append((x, y, target_z))
                current_y = zone_max
        else:
            current_y = zone_max

    # Финальный сегмент
    if current_y < y_max:
        y_vals = np.arange(current_y + step, y_max + step/2, step)
        for y in y_vals:
            if y > y_max + 1e-6:
                continue
            x = get_x(y)
            new_wps.append((x, y, target_z))

    return new_wps, return_points

# ==============================
# ФУНКЦИЯ: ФИЛЬТРАЦИЯ МАРШРУТА ПО СЕТКЕ X И Y (ТОЛЬКО РАБОЧАЯ ВЫСОТА)
# ==============================
def filter_waypoints_by_grid(waypoints, cable_height, step=WAYPOINT_STEP, tol=0.05):
    """
    Оставляет только те точки на рабочей высоте (cable_height + TARGET_HEIGHT_ABOVE),
    у которых и X, и Y попадают в сетку с шагом step.
    Точки на других высотах (safe_altitude) не фильтруются.
    """
    target_z = cable_height + TARGET_HEIGHT_ABOVE
    filtered = []
    for wp in waypoints:
        # Если точка на рабочей высоте (с допуском 0.05)
        if abs(wp[2] - target_z) < 0.05:
            x, y = wp[0], wp[1]
            kx = round(x / step)
            ky = round(y / step)
            grid_x = kx * step
            grid_y = ky * step
            if abs(x - grid_x) <= tol and abs(y - grid_y) <= tol:
                filtered.append(wp)
        else:
            # Точки подъёма/спуска сохраняем
            filtered.append(wp)
    return filtered

# ==============================
# ФУНКЦИЯ: ФИЛЬТРАЦИЯ ТОЧЕК НА БЕЗОПАСНОЙ ВЫСОТЕ
# ==============================
def filter_safe_altitude_points(waypoints, safe_altitude, step=WAYPOINT_STEP, tol=0.05):
    """
    Оставляет только те точки на безопасной высоте (в пределах tol от safe_altitude),
    у которых X и Y попадают в сетку с шагом step с допуском tol.
    Точки на других высотах сохраняются без изменений.
    """
    filtered = []
    for wp in waypoints:
        x, y, z = wp
        if abs(z - safe_altitude) < 0.05:  # точка на safe_altitude
            kx = round(x / step)
            ky = round(y / step)
            grid_x = kx * step
            grid_y = ky * step
            if abs(x - grid_x) <= tol and abs(y - grid_y) <= tol:
                filtered.append(wp)
        else:
            filtered.append(wp)
    return filtered

# ==============================
# ОСНОВНАЯ МИССИЯ
# ==============================
def execute_mission():
    global scan_active, scan_direction, scan_points_forward, scan_points_backward

    # Создание директории для сохранения изображения маршрута
    os.makedirs('newroutes', exist_ok=True)

    # Удаляем предыдущий recent, чтобы во время миссии не было видно старого маршрута
    recent_file = 'newroutes/scan_route_recent.png'
    if os.path.exists(recent_file):
        os.remove(recent_file)
        log(f"Removed old recent route image: {recent_file}")

    setup_transforms()
    rospy.sleep(1)

    log("Taking off to scan height")
    navigate_wait(z=SCAN_HEIGHT, frame_id='body', auto_arm=True)

    log("Moving to scan start")
    navigate_wait(x=SCAN_AREA_X, y=SCAN_Y_MIN, z=SCAN_HEIGHT,
                  frame_id='aruco_map', speed=SPEED_HORIZONTAL)

    # ==============================
    # ПЕРВЫЙ ПРОХОД (FORWARD) – ТОЛЬКО КОНСОЛИ
    # ==============================
    log(f"=== First pass (forward) at height {SCAN_HEIGHT:.2f} m ===")
    scan_points_forward = []
    scan_active = True
    scan_direction = 'forward'
    rospy.sleep(SCAN_START_DELAY)

    log("Starting scan forward along the length of zone")
    navigate_wait(x=SCAN_AREA_X, y=SCAN_Y_MAX, z=SCAN_HEIGHT,
                  frame_id='aruco_map', speed=SPEED_HORIZONTAL, tol=0.3)

    rospy.sleep(1)
    scan_active = False
    log(f"First pass completed, collected {len(scan_points_forward)} points")

    # Обработка первого прохода – извлекаем только информацию о консолях
    console_info_raw = []
    forward_max_console_z = None
    console_points_fwd = np.empty((0, 3))  # для визуализации
    if len(scan_points_forward) > 0:
        points_forward_np = np.array(scan_points_forward)
        # Вызываем extract_cables_and_consoles, но нас интересуют только консоли
        _, _, console_points_fwd, _, console_info_raw = extract_cables_and_consoles(points_forward_np)
        if console_points_fwd is not None and len(console_points_fwd) > 0:
            forward_max_console_z = np.max(console_points_fwd[:, 2])
            log(f"Forward pass: max console height = {forward_max_console_z:.2f} m")
            log(f"Forward pass: detected {len(console_info_raw)} raw consoles with half-widths: " +
                ", ".join([f"{info['center']:.2f}({info['half_width']:.3f})" for info in console_info_raw]))
        else:
            log("Forward pass: no consoles detected")

    if console_info_raw:
        console_info_merged = merge_console_zones(console_info_raw)
        log(f"After merging: {len(console_info_merged)} consolidated console zones with half-widths: " +
            ", ".join([f"{info['center']:.2f}({info['half_width']:.3f})" for info in console_info_merged]))
    else:
        console_info_merged = []

    console_half_width = {info['center']: info['half_width'] for info in console_info_merged}

    # Определяем высоту второго прохода (над консолями)
    if forward_max_console_z is not None:
        SCAN_HEIGHT_BACK = forward_max_console_z + 0.5
    else:
        SCAN_HEIGHT_BACK = SCAN_HEIGHT + 0.5
        log("Using fallback height for second pass")

    # ==============================
    # ВТОРОЙ ПРОХОД (BACKWARD) – ТОЛЬКО ПЛАТФОРМЫ (ТРОСЫ)
    # ==============================
    log(f"=== Second pass (backward) at height {SCAN_HEIGHT_BACK:.2f} m (above detected consoles) ===")

    telem = get_telemetry(frame_id='aruco_map')
    navigate_wait(x=telem.x, y=telem.y, z=SCAN_HEIGHT_BACK,
                  frame_id='aruco_map', speed=SPEED_VERTICAL)

    scan_points_backward = []
    scan_active = True
    scan_direction = 'backward'
    rospy.sleep(SCAN_START_DELAY)

    log("Starting scan backward along the length of zone")
    navigate_wait(x=SCAN_AREA_X, y=SCAN_Y_MIN, z=SCAN_HEIGHT_BACK,
                  frame_id='aruco_map', speed=SPEED_HORIZONTAL, tol=0.3)

    scan_active = False
    rospy.sleep(0.5)
    log(f"Second pass completed, collected {len(scan_points_backward)} points")

    # Фильтруем точки второго прохода: оставляем только те, что гарантированно ниже консолей
    if len(scan_points_backward) == 0:
        log("No lidar data from backward pass. Aborting.")
        return

    points_backward_np = np.array(scan_points_backward)
    # Порог: немного ниже минимальной ожидаемой высоты консолей (например, 1.6 м при CONSOLE_HEIGHT=1.8)
    height_threshold = CONSOLE_HEIGHT - 0.2
    mask_low = points_backward_np[:, 2] < height_threshold
    points_backward_low = points_backward_np[mask_low]

    if len(points_backward_low) == 0:
        log("No points below threshold in backward pass. Aborting.")
        return

    # Извлекаем только тросы (верхний и нижний) из отфильтрованных точек
    high_points, low_points, _, _, _ = extract_cables_and_consoles(points_backward_low)
    if high_points is None or low_points is None or len(high_points) == 0 or len(low_points) == 0:
        log("Failed to extract cables from backward pass. Aborting.")
        return

    log(f"High cable: {len(high_points)} points, Low cable: {len(low_points)} points")

    # ==============================
    # ПОСТРОЕНИЕ МАРШРУТОВ
    # ==============================
    waypoints_high, high_h = build_waypoints(high_points)
    waypoints_low, low_h = build_waypoints(low_points)

    if not waypoints_high or not waypoints_low:
        log("Failed to build waypoints.")
        return

    log(f"High cable Y range: {np.min(high_points[:,1]):.2f} - {np.max(high_points[:,1]):.2f}")
    log(f"High route start Y: {waypoints_high[0][1]:.2f}")
    log(f"Low cable Y range: {np.min(low_points[:,1]):.2f} - {np.max(low_points[:,1]):.2f}")
    log(f"Low route start Y: {waypoints_low[0][1]:.2f}")

    y_cross, min_dist = find_crossing_point(high_points, low_points)
    if y_cross:
        log(f"Crossing point Y = {y_cross:.2f}, min distance = {min_dist:.3f}")
    else:
        log("No crossing point found")

    # Безопасная высота на основе максимальной высоты консолей (из первого прохода)
    if forward_max_console_z is not None:
        safe_altitude = forward_max_console_z + SAFETY_MARGIN
        log(f"Safe altitude set to {safe_altitude:.2f} m (max console {forward_max_console_z:.2f} m + margin)")
    else:
        safe_altitude = SCAN_HEIGHT
        log(f"No consoles detected, using scan height as safe altitude: {safe_altitude:.2f} m")

    # Применяем обход консолей к маршруту верхнего троса
    wp_high_current = waypoints_high
    zones_high = list(console_half_width.keys())
    zones_high.sort()
    for z in zones_high:
        offset = console_half_width[z]
        wp_high_current, _, _ = build_avoidance_waypoints(wp_high_current, high_h, z, safe_altitude, offset)

    # Для нижнего троса учитываем ещё и точку пересечения (если есть)
    zones_low_centers = list(console_half_width.keys())
    if y_cross:
        zones_low_centers.append(y_cross)
    zones_low_centers.sort()

    offsets_low = []
    for z in zones_low_centers:
        if z == y_cross:
            offsets_low.append(CROSSING_OFFSET)
        else:
            offsets_low.append(console_half_width[z])

    # ----- Инициализация переменных для объединения (будут перезаписаны в блоке, если он активен) -----
    is_merged_case = False
    crossing_start_line = None
    first_console_end_line = None
    first_console_center = None

    # ----- БЛОК ОБЪЕДИНЕНИЯ ЗОН (закомментирован) -----
    """
    if y_cross is not None:
        cross_idx = None
        for idx, z in enumerate(zones_low_centers):
            if z == y_cross:
                cross_idx = idx
                break
        if cross_idx is not None and cross_idx + 1 < len(zones_low_centers):
            next_z = zones_low_centers[cross_idx + 1]
            if next_z != y_cross:
                zoneA_min = y_cross - offsets_low[cross_idx]
                zoneA_max = y_cross + offsets_low[cross_idx]
                zoneB_min = next_z - offsets_low[cross_idx + 1]
                zoneB_max = next_z + offsets_low[cross_idx + 1]
                distance = zoneB_min - zoneA_max
                if distance < MERGE_THRESHOLD:
                    is_merged_case = True
                    crossing_start_line = zoneA_min
                    first_console_end_line = zoneB_max
                    first_console_center = next_z
                    new_center = (zoneA_min + zoneB_max) / 2
                    new_offset = (zoneB_max - zoneA_min) / 2
                    new_zones = zones_low_centers[:cross_idx] + [new_center] + zones_low_centers[cross_idx+2:]
                    new_offsets = offsets_low[:cross_idx] + [new_offset] + offsets_low[cross_idx+2:]
                    zones_low_centers = new_zones
                    offsets_low = new_offsets
                    log(f"Merged crossing and first console into zone center={new_center:.3f}, offset={new_offset:.3f}")
                else:
                    log(f"Crossing and first console distance {distance:.3f} >= threshold {MERGE_THRESHOLD:.3f}, not merging")
    """

    # Формируем список отображаемых консольных зон
    if is_merged_case and first_console_center is not None:
        # Исключаем первую консоль (ту, что была объединена)
        display_console_zones = [zone for zone in console_info_merged if abs(zone['center'] - first_console_center) > 0.01]
    else:
        display_console_zones = console_info_merged

    # Формируем параметры для build_route_with_returns
    return_targets_low = []
    extra_forward_low = []
    takeoff_at_current_low = []
    for i, (z, offset) in enumerate(zip(zones_low_centers, offsets_low)):
        zone_min = z - offset
        zone_max = z + offset
        if z == y_cross:
            # Для пересечения: цель возврата - начало первой консоли (если есть)
            next_console_idx = None
            for j in range(i+1, len(zones_low_centers)):
                if zones_low_centers[j] != y_cross:
                    next_console_idx = j
                    break
            if next_console_idx is not None:
                return_targets_low.append(zones_low_centers[next_console_idx] - offsets_low[next_console_idx])
                extra_forward_low.append(True)
            else:
                return_targets_low.append(zone_min)
                extra_forward_low.append(False)
            takeoff_at_current_low.append(False)
        else:
            # Для консоли
            return_targets_low.append(zone_max)  # без возврата
            extra_forward_low.append(False)
            # Определяем, является ли эта консоль первой после пересечения
            is_first_after_crossing = False
            if y_cross is not None:
                cross_idx = None
                for idx, zz in enumerate(zones_low_centers):
                    if zz == y_cross:
                        cross_idx = idx
                        break
                if cross_idx is not None and i == cross_idx + 1:
                    is_first_after_crossing = True
            takeoff_at_current_low.append(is_first_after_crossing)

    wp_low_current, return_points_low = build_route_with_returns(
        waypoints_low, low_h, zones_low_centers, offsets_low, safe_altitude,
        return_targets=return_targets_low, extra_forward=extra_forward_low,
        takeoff_at_current=takeoff_at_current_low
    )

    # ===== ФИЛЬТРАЦИЯ МАРШРУТОВ ПО СЕТКЕ (X и Y) =====
    log("Filtering waypoints by grid (X and Y) at cable height...")
    wp_high_current = filter_waypoints_by_grid(wp_high_current, high_h)
    wp_low_current = filter_waypoints_by_grid(wp_low_current, low_h)
    log(f"After filtering: high route has {len(wp_high_current)} points, low route has {len(wp_low_current)} points")

    # ===== ДОПОЛНИТЕЛЬНАЯ ФИЛЬТРАЦИЯ ТОЧЕК НА БЕЗОПАСНОЙ ВЫСОТЕ =====
    log("Filtering safe altitude points by grid...")
    wp_high_current = filter_safe_altitude_points(wp_high_current, safe_altitude)
    wp_low_current = filter_safe_altitude_points(wp_low_current, safe_altitude)
    log(f"After safe altitude filtering: high route has {len(wp_high_current)} points, low route has {len(wp_low_current)} points")

    mission_structure = {
        'high_cable': build_mission_structure(wp_high_current, high_h, zones_high),
        'low_cable': build_mission_structure(wp_low_current, low_h, zones_low_centers)
    }

    # Создание директорий для сохранения результатов (относительные пути)
    os.makedirs('routes', exist_ok=True)
    with open('routes/mission_structure.json', 'w') as f:
        json.dump(mission_structure, f, indent=2)
    log("Mission structure saved to mission_structure.json")

    full_route = wp_high_current + wp_low_current
    with open('routes/current_mission.json', 'w') as f:
        json.dump(full_route, f)

    existing = glob.glob('newroutes/scan_route*.png')
    max_num = 0
    for f in existing:
        base = os.path.basename(f)
        if base.startswith('scan_route') and base.endswith('.png'):
            num_part = base[10:-4]  # удаляем 'scan_route' и '.png'
            if num_part.isdigit():
                num = int(num_part)
                if num > max_num:
                    max_num = num
    next_num = max_num + 1
    route_filename = f'newroutes/scan_route{next_num}.png'
    log(f"Saving route plot as {route_filename}")

    # Вызов функции построения графика с использованием точек консолей из первого прохода и точек тросов из второго
    plot_scan_data(high_points, low_points, console_points_fwd,
                   wp_high_current, wp_low_current,
                   y_cross, merged_zones=display_console_zones,
                   return_points_low=return_points_low,
                   crossing_start_line=crossing_start_line,
                   first_console_end_line=first_console_end_line,
                   is_merged_case=is_merged_case, filename=route_filename)

    # После сохранения копируем как recent (перезаписываем)
    shutil.copy2(route_filename, 'newroutes/scan_route_recent.png')
    log("Updated recent route image")

    # ==============================
    # ВЫПОЛНЕНИЕ МАРШРУТА (без изменений)
    # ==============================
    log("=== Route Phase 1: First cable (high) ===")
    first_wp = waypoints_high[0]

    navigate_wait(x=first_wp[0], y=first_wp[1], z=safe_altitude,
                  frame_id='aruco_map', speed=SPEED_HORIZONTAL)
    navigate_wait(x=first_wp[0], y=first_wp[1], z=first_wp[2],
                  frame_id='aruco_map', speed=SPEED_VERTICAL, tol=DIST_TOL)

    log("Correcting position above first cable start")
    correct_position(high_h, stabilization_delay=1.5)
    log("Coupling with the messenger wire in process")
    rospy.sleep(2.0)

    i = 0
    total = len(wp_high_current)
    obstacle_started = False
    while i < total:
        wp = wp_high_current[i]
        if abs(wp[2] - (high_h + TARGET_HEIGHT_ABOVE)) < 0.05:
            seg_start = i
            while i < total and abs(wp_high_current[i][2] - (high_h + TARGET_HEIGHT_ABOVE)) < 0.05:
                i += 1
            seg_end = i - 1
            target = wp_high_current[seg_end]
            log(f"Executing high segment from Y={wp_high_current[seg_start][1]:.2f} to Y={target[1]:.2f}")
            first_segment = (seg_start == 0)
            success = run_segment_follower(target[0], target[1], target[2], high_h,
                                           disable_correction=first_segment)
            if not success:
                log("Segment follower failed, initiating homecoming")
                subprocess.run([sys.executable, "homecoming.py"])
                return
        else:
            if not obstacle_started:
                log("Disconnection from the messenger wire is in progress")
                obstacle_started = True
                rospy.sleep(2.0)
                log("Obstacle avoidance started")
            navigate_wait(x=wp[0], y=wp[1], z=wp[2], frame_id='aruco_map', speed=SPEED_HORIZONTAL, tol=0.05)
            i += 1
            if i < total and abs(wp_high_current[i][2] - (high_h + TARGET_HEIGHT_ABOVE)) < 0.05:
                log("Obstacle avoidance completed. Correction in progress")
                correct_position(high_h, stabilization_delay=1.0)
                log("Coupling with the messenger wire in process")
                rospy.sleep(2.0)
                obstacle_started = False

    last_high = waypoints_high[-1]
    log("Flight over the high messenger wire completed. Ascent to safe altitude")
    navigate_wait(x=last_high[0], y=last_high[1], z=safe_altitude,
                  frame_id='aruco_map', speed=SPEED_ASCENT)

    log("=== Route Phase 2: Second cable (low) ===")
    first_low = wp_low_current[0]

    navigate_wait(x=first_low[0], y=first_low[1], z=safe_altitude,
                  frame_id='aruco_map', speed=SPEED_HORIZONTAL)
    navigate_wait(x=first_low[0], y=first_low[1], z=first_low[2],
                  frame_id='aruco_map', speed=SPEED_VERTICAL, tol=DIST_TOL)

    log("Correcting position above second cable start")
    correct_position(low_h, stabilization_delay=1.5)
    log("Coupling with the messenger wire in process")
    rospy.sleep(2.0)

    i = 0
    total = len(wp_low_current)
    obstacle_started = False
    while i < total:
        wp = wp_low_current[i]
        if abs(wp[2] - (low_h + TARGET_HEIGHT_ABOVE)) < 0.05:
            seg_start = i
            while i < total and abs(wp_low_current[i][2] - (low_h + TARGET_HEIGHT_ABOVE)) < 0.05:
                i += 1
            seg_end = i - 1
            target = wp_low_current[seg_end]
            log(f"Executing low segment from Y={wp_low_current[seg_start][1]:.2f} to Y={target[1]:.2f}")

            is_last_segment = (seg_end == total - 1)

            first_segment = (seg_start == 0)
            success = run_segment_follower(target[0], target[1], target[2], low_h,
                                           disable_correction=first_segment)
            if not success:
                log("Segment follower failed, initiating homecoming")
                subprocess.run([sys.executable, "homecoming.py"])
                return

            if is_last_segment:
                log("Final point of the second route reached")
        else:
            if not obstacle_started:
                log("Disconnection from the messenger wire is in progress")
                obstacle_started = True
                rospy.sleep(2.0)
                log("Obstacle avoidance started")
            navigate_wait(x=wp[0], y=wp[1], z=wp[2], frame_id='aruco_map', speed=SPEED_HORIZONTAL, tol=0.05)
            i += 1
            if i < total and abs(wp_low_current[i][2] - (low_h + TARGET_HEIGHT_ABOVE)) < 0.05:
                log("Obstacle flight completed")
                rospy.sleep(2.0)
                correct_position(low_h, stabilization_delay=1.0)
                log("Coupling with the messenger wire in process")
                rospy.sleep(2.0)
                obstacle_started = False

    log("Route completed. Disconnecting from the messenger wire")
    rospy.sleep(2.0)

    log("Mission completed, launching homecoming")
    subprocess.run([sys.executable, "homecoming.py"])

if __name__ == '__main__':
    try:
        execute_mission()
    except Exception as e:
        log(f"Critical failure: {str(e)}")
        hold_position()
        subprocess.run([sys.executable, "homecoming.py"])
        rospy.signal_shutdown("Emergency")