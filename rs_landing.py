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
    
def altitude_scale_factor(altitude, k=2.0, min_scale=0.2, max_scale=1.0):
    """
    Returns a scale factor (0–1) to adjust angular corrections based on altitude.

    altitude : current altitude (meters)
    k        : tuning constant (larger = slower response)
    min_scale: prevents zero correction near ground
    max_scale: caps aggressive corrections at high altitude
    """

    if altitude <= 0:
        return min_scale

    scale = altitude / (altitude + k)

    # clamp for safety
    scale = max(min_scale, min(scale, max_scale))

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

        # Convert depth frame to numpy array
        frame = np.asanyarray(depth_frame.get_data())
         # Mask out values greater than MAX_DISTANCE
        mask = frame * depth_frame.get_units() <= MAX_DISTANCE
        frame = frame * mask

        height, width = frame.shape
        third_height = height // 3
        third_width = width // 3

        # Split the frame into 3x3 grid
        segments = [
            frame[:third_height, :third_width], frame[:third_height, third_width:2*third_width], frame[:third_height, 2*third_width:],
            frame[third_height:2*third_height, :third_width], frame[third_height:2*third_height, third_width:2*third_width], frame[third_height:2*third_height, 2*third_width:],
            frame[2*third_height:, :third_width], frame[2*third_height:, third_width:2*third_width], frame[2*third_height:, 2*third_width:]
        ]

        # Calculate min and max disparity in each segment
        min_diffs = []
        for segment in segments:
            min_disp = np.min(segment)
            max_disp = np.max(segment)
            min_diffs.append(max_disp - min_disp)

        # Convert disparity differences to meters
        disparity_to_depth_scale = depth_frame.get_units()
        differences_in_meters = [diff * disparity_to_depth_scale for diff in min_diffs]

        # Find the segment with the lowest difference
        min_diff = min(differences_in_meters)
        min_diff_index = differences_in_meters.index(min_diff)

        # Apply color map to the original frame for visualization
        frame_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)

        # Draw grid lines
        cv2.line(frame_colored, (third_width, 0), (third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (2*third_width, 0), (2*third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, third_height), (width, third_height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, 2*third_height), (width, 2*third_height), (255, 255, 255), 2)

        # Annotate each segment with the difference in meters
        for i, diff in enumerate(differences_in_meters):
            top_left_x = (i % 3) * third_width
            top_left_y = (i // 3) * third_height
            cv2.putText(frame_colored, f"{diff:.2f}m", (top_left_x + 5, top_left_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Check if the lowest difference is less than or equal to 0.2m and draw bounding box
        altitude = get_rangefinder_data(vehicle)
        if (min_diff <= min_diff_threshold) and (altitude>=safe_land_min_alt):
            top_left_x = (min_diff_index % 3) * third_width
            top_left_y = (min_diff_index // 3) * third_height
            bottom_right_x = top_left_x + third_width
            bottom_right_y = top_left_y + third_height
            cv2.rectangle(frame_colored, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)

            top_left_x = (min_diff_index % 3) * third_width
            top_left_y = (min_diff_index // 3) * third_height
            bottom_right_x = top_left_x + third_width
            bottom_right_y = top_left_y + third_height
                
            x_sum = top_left_x + bottom_right_x
            y_sum = top_left_y + bottom_right_y
                
            x_avg = x_sum * 0.5 
            y_avg = y_sum * 0.5
                
            scale_factor = altitude_scale_factor(altitude)

            x_ang = (((x_avg - width * 0.5) * (hfov / width)) + offset_x) * scale_factor
            y_ang = (((y_avg - height * 0.5) * (vfov / height)) + offset_y) * scale_factor
            print(f"x angle: {x_ang} y angle: {y_ang}, scale factor: {scale_factor}, altitude: {altitude}")
            send_land_message(x_ang, y_ang)
                #break                                 #break the loop if safe place is found
            # time.sleep(1/10)
                        
            # cv2.rectangle(frame_colored, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)
            # print(f"x angle: {x_ang} y angle: {y_ang}")

        else:
            # send_land_message(0,0.5) #move to another location this should be respective of direction with no obstacles.
            print("No safe landing spot detected. Continuing search...")


