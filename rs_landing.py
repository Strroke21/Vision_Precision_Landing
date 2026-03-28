#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
import numpy as np
import time
import math
from pymavlink import mavutil

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

MAX_DISTANCE = 8.0

hfov = 87 * (math.pi/180)
vfov = 58 * (math.pi/180)

offset_x = 0.3
offset_y = 0.2

fcu_addr = '/dev/ttyACM0' 
min_diff_threshold = 0.2
safe_land_min_alt = 2
current_target = None
stable_count = 0

HYST_THRESHOLD = 3   # frames required to switch
DIFF_MARGIN = 0.05   # meters improvement required

###### functions #######

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def set_parameter(vehicle, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(vehicle.target_system,vehicle.target_component,param_name.encode('utf-8'),param_value,param_type)
    #usage set_parameter(vehicle, "PARAM_NAME", 1)

def send_land_message(x_ang,y_ang):
    msg = vehicle.mav.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x_ang,
        y_ang,
        0, #distance to marker (m)
        0, #size_x (rad)
        0, #size_y (rad)
        ) 
    vehicle.mav.send(msg)

def get_system_status(vehicle):
    # Wait for a 'HEARTBEAT' message
    message = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=10)

    if message is None:
        raise TimeoutError("Did not receive HEARTBEAT message in time")

    # Interpret the system status
    system_status = message.system_status

    status_mapping = {
        mavutil.mavlink.MAV_STATE_UNINIT: 'Uninitialized',
        mavutil.mavlink.MAV_STATE_BOOT: 'Booting',
        mavutil.mavlink.MAV_STATE_CALIBRATING: 'Calibrating',
        mavutil.mavlink.MAV_STATE_STANDBY: 'Standby',
        mavutil.mavlink.MAV_STATE_ACTIVE: 'Active',
        mavutil.mavlink.MAV_STATE_CRITICAL: 'Critical',
        mavutil.mavlink.MAV_STATE_EMERGENCY: 'Emergency',
        mavutil.mavlink.MAV_STATE_POWEROFF: 'Powered Off',
        mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION: 'Flight Termination',
    }

    return status_mapping.get(system_status, 'Unknown')

def get_rangefinder_data(vehicle):
    global rng_alt
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            rng_alt = msg.current_distance/100  # in meters
        return rng_alt
    
def compute_angle_scale_3x3(altitude, hfov, vfov,
                           base_gain=0.8,
                           min_scale=0.15,
                           max_scale=0.9,
                           smooth_factor=0.75,
                           prev_scale=[0.5]):
    """
    Scale factor for 3x3 grid-based landing control.

    Designed to:
    - avoid aggressive jumps between large cells
    - maintain smooth convergence
    """

    if altitude <= 0:
        return min_scale

    # --- Camera footprint (meters) ---
    width_m  = 2 * altitude * math.tan(hfov / 2)
    height_m = 2 * altitude * math.tan(vfov / 2)

    # --- Cell size (3x3 grid) ---
    cell_w = width_m / 3.0
    cell_h = height_m / 3.0

    # --- Effective motion scale ---
    # Larger cells → reduce aggressiveness
    avg_cell = (cell_w + cell_h) * 0.5

    # Normalize (prevents explosion at high altitude)
    scale = base_gain * avg_cell
    scale = scale / (1.0 + scale)

    # --- Clamp ---
    scale = max(min_scale, min(scale, max_scale))

    # --- Smooth (EMA) ---
    scale = smooth_factor * prev_scale[0] + (1 - smooth_factor) * scale
    prev_scale[0] = scale

    return scale
    
def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

vehicle = connect(fcu_addr)
enable_data_stream(vehicle,stream_rate=200)
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
set_parameter(vehicle,'PLND_ENABLED', 1)
set_parameter(vehicle,'PLND_TYPE',1) ##1 for companion computer
set_parameter(vehicle,'PLND_EST_TYPE', 0) # 0 for raw sensor, 1 for kalman filter pos estimation
set_parameter(vehicle,'LAND_SPEED',30) ##Descent speed of 30cm/s
set_parameter(vehicle,'PLND_XY_DIST_MAX', 8)
# Start streaming
pipeline.start(config)

status = get_system_status(vehicle) #input("Enter the vehicle status: ")
print("Vehicle State: ",status)

if ('Critical' in status) or ('Emergency' in status):
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        frame = np.asanyarray(depth_frame.get_data())

        mask = frame * depth_frame.get_units() <= MAX_DISTANCE
        frame = frame * mask

        height, width = frame.shape
        third_height = height // 3
        third_width = width // 3

        segments = [
            frame[:third_height, :third_width], frame[:third_height, third_width:2*third_width], frame[:third_height, 2*third_width:],
            frame[third_height:2*third_height, :third_width], frame[third_height:2*third_height, third_width:2*third_width], frame[third_height:2*third_height, 2*third_width:],
            frame[2*third_height:, :third_width], frame[2*third_height:, third_width:2*third_width], frame[2*third_height:, 2*third_width:]
        ]

        min_diffs = []
        for segment in segments:
            min_disp = np.min(segment)
            max_disp = np.max(segment)
            min_diffs.append(max_disp - min_disp)

        disparity_to_depth_scale = depth_frame.get_units()
        differences_in_meters = [diff * disparity_to_depth_scale for diff in min_diffs]

        # --- Original best candidate ---
        min_diff = min(differences_in_meters)
        min_diff_index = differences_in_meters.index(min_diff)

        # =========================
        #  HYSTERESIS START
        # =========================
        if current_target is None:
            current_target = min_diff_index
        else:
            current_diff = differences_in_meters[current_target]
            new_diff = differences_in_meters[min_diff_index]

            # Only switch if new cell is significantly better
            if (new_diff + DIFF_MARGIN) < current_diff:
                stable_count += 1

                if stable_count >= HYST_THRESHOLD:
                    current_target = min_diff_index
                    stable_count = 0
            else:
                stable_count = 0

        selected_index = current_target
        selected_diff = differences_in_meters[selected_index]
        # =========================
        #  HYSTERESIS END
        # =========================

        frame_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)

        cv2.line(frame_colored, (third_width, 0), (third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (2*third_width, 0), (2*third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, third_height), (width, third_height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, 2*third_height), (width, 2*third_height), (255, 255, 255), 2)

        for i, diff in enumerate(differences_in_meters):
            top_left_x = (i % 3) * third_width
            top_left_y = (i // 3) * third_height
            cv2.putText(frame_colored, f"{diff:.2f}m",
                        (top_left_x + 5, top_left_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        altitude = get_rangefinder_data(vehicle)

        # ✅ Use selected_diff instead of min_diff
        if (selected_diff <= min_diff_threshold) and (altitude >= safe_land_min_alt):

            top_left_x = (selected_index % 3) * third_width
            top_left_y = (selected_index // 3) * third_height
            bottom_right_x = top_left_x + third_width
            bottom_right_y = top_left_y + third_height

            cv2.rectangle(frame_colored,
                        (top_left_x, top_left_y),
                        (bottom_right_x, bottom_right_y),
                        (0, 255, 0), 2)

            x_avg = (top_left_x + bottom_right_x) * 0.5
            y_avg = (top_left_y + bottom_right_y) * 0.5

            scale_factor = compute_angle_scale_3x3(altitude, hfov, vfov)

            x_ang = (((x_avg - width * 0.5) * (hfov / width)) + offset_x) * scale_factor
            y_ang = (((y_avg - height * 0.5) * (vfov / height)) + offset_y) * scale_factor

            print(f"[HYST] idx:{selected_index} | x:{x_ang:.3f} y:{y_ang:.3f} | scale:{scale_factor:.3f} | alt:{altitude:.2f}")

            send_land_message(x_ang, y_ang)

        else:
            print("[HYST] No stable safe landing spot yet...")