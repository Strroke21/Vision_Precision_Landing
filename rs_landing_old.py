#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
from pymavlink import mavutil
import time
from std_msgs.msg import String

hfov, vfov = np.radians(87.0), np.radians(58.0)
flatness, final_alt = 0.2, 4
disparity_to_depth_scale = 0.0010000000474974513
MAX_DISTANCE = 15.0
max_alt = 10.0
safe_land_radius = 2

fcu_addr = 'udp:127.0.0.1:14561' 
current_target = None

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
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is not None:
            rng_alt = msg.current_distance/100  # in meters
        return rng_alt

def compute_angle_scale_5x5(altitude, hfov, vfov,
                           base_gain=0.8,
                           min_scale=0.15,
                           max_scale=0.9,
                           smooth_factor=0.75,
                           prev_scale=[0.5]):
    """
    Scale factor for 5x5 grid (using inner 3x3 cells).

    Adjustments:
    - Smaller cells → less aggressive inherently
    - Inner region → reduced effective FOV → compensate
    """

    if altitude <= 0:
        return min_scale

    # --- Convert FOV to radians (IMPORTANT if not already) ---
    hfov_rad = hfov
    vfov_rad = vfov

    # --- Camera footprint ---
    width_m  = 2 * altitude * math.tan(hfov_rad / 2)
    height_m = 2 * altitude * math.tan(vfov_rad / 2)

    # --- 5x5 grid cell size ---
    cell_w = width_m / 5.0
    cell_h = height_m / 5.0

    avg_cell = 0.5 * (cell_w + cell_h)

    # --- Compensation factor ---
    # Inner usable grid = 5/5 of full → boost response
    compensation = 5.0 / 5.0  

    # --- Base scaling ---
    scale = base_gain * avg_cell * compensation

    # Normalize (prevents blow-up)
    scale = scale / (1.0 + scale)

    # Clamp
    scale = max(min_scale, min(scale, max_scale))

    # Smooth (EMA)
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

def status_check(vehicle):
    while True:
        status = get_system_status(vehicle)
        print("Vehicle State: ",status)
        if ('Critical' in status) or ('Emergency' in status):
            return status
        
def flightMode(vehicle):
    while True:
        vehicle.recv_match(type='HEARTBEAT', blocking=False)
        # Wait for a 'HEARTBEAT' message
        mode = vehicle.flightmode

        return mode

def VehicleMode(vehicle,mode):

    modes = ["STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE","", "LAND"]
    if mode in modes:
        mode_id = modes.index(mode)
    else:
        mode_id = 12
    ##### changing to guided mode #####
    #mode_id = 0:STABILIZE, 1:ACRO, 2: ALT_HOLD, 3:AUTO, 4:GUIDED, 5:LOITER, 6:RTL, 7:CIRCLE, 9:LAND 12:None
    vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)


def send_position_setpoint(vehicle, pos_x, pos_y, pos_z,FRAME):

    # Send MAVLink command to set position
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        FRAME,  # frame
        0b110111111000,        # type_mask (only for postion)
        pos_x, pos_y, pos_z,   # position 
        0, 0, 0,                 # velocity in m/s (not used)
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )

def dist_to_target(x_dist, y_dist):
    return math.sqrt(x_dist**2 + y_dist**2)

vehicle = connect(fcu_addr)
enable_data_stream(vehicle,stream_rate=200)
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
set_parameter(vehicle,'PLND_ENABLED', 1)
set_parameter(vehicle,'PLND_TYPE',1) ##1 for companion computer
set_parameter(vehicle,'PLND_EST_TYPE', 0) # 0 for raw sensor, 1 for kalman filter pos estimation
set_parameter(vehicle,'LAND_SPEED',30) ##Descent speed of 30cm/s
set_parameter(vehicle,'PLND_XY_DIST_MAX', 8)
set_parameter(vehicle,'PLND_LAG',0.042)
# Start streaming


class SafeLander(Node):
    def __init__(self):
        super().__init__('safe_lander')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        # self.aruco_subscription = self.create_subscription(String,'/aruco_detection_state',self.aruco_callback,10)

        self.last_valid_altitude = 2.0

        # 🔹 Persistent selected cell (hysteresis)
        self.current_cell = None  # (i, j)
        self.current_diff = float('inf')
        self.bad_frame_count = 0
        self.hyst_threshold = 30   # frames
        self.land_counter = 0
        self.aruco_counter = 0

    # def aruco_callback(self,msg):
    #     if msg.data == "True":
    #         self.aruco_counter += 1

    def depth_callback(self, msg):
        st = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

        mask = (frame <= (MAX_DISTANCE * 1000)).astype(np.uint16)
        frame = frame * mask

        height, width = frame.shape

        # ---- ALTITUDE ----
        cx, cy = width // 2, height // 2
        center_depth_m = frame[cy, cx] * disparity_to_depth_scale
        altitude = get_rangefinder_data(vehicle)
        if altitude is None:
            if 0.1 < center_depth_m < MAX_DISTANCE:
                altitude = center_depth_m
                self.last_valid_altitude = altitude
            else:
                altitude = self.last_valid_altitude

        mode = flightMode(vehicle)

        self.get_logger().info(f"[Altitude: {altitude:.2f} m FlightMode: {mode}]")

        # ---- GRID (5x5, ignore edges) ----
        grid_rows, grid_cols = 5, 5
        cell_w, cell_h = width // grid_cols, height // grid_rows

        differences = []
        valid_indices = []

        for i in range(grid_rows):
            for j in range(grid_cols):
                seg = frame[
                    i * cell_h:(i + 1) * cell_h,
                    j * cell_w:(j + 1) * cell_w
                ]

                seg = seg[seg > 0]

                if seg.size == 0:
                    diff = float('inf')
                else:
                    diff = np.max(seg) - np.min(seg)

                scaled_diff = diff * disparity_to_depth_scale

                # Priority weighting (central 3x3 preferred)
                if 1 <= i <= 3 and 1 <= j <= 3:
                    weight = 1.0   # central cells (no penalty)
                else:
                    weight = 1.1   # outer cells penalized

                differences.append(scaled_diff * weight)
                valid_indices.append((i, j))

        # ---- FIND BEST CELL ----
        min_diff = min(differences)
        min_idx = differences.index(min_diff)
        best_cell = valid_indices[min_idx]

        # ---- HYSTERESIS LOGIC (with frame persistence) ----
        if self.current_cell is None:
            self.current_cell = best_cell
            self.current_diff = min_diff
            self.bad_frame_count = 0

        else:
            # Update current cell diff
            if self.current_cell in valid_indices:
                idx = valid_indices.index(self.current_cell)
                self.current_diff = differences[idx]
            else:
                self.current_diff = float('inf')

            # Check if current cell is bad
            if self.current_diff > flatness:
                self.bad_frame_count += 1
            else:
                self.bad_frame_count = 0  # reset if good again

            # Switch only if bad for N frames
            if self.bad_frame_count >= self.hyst_threshold:
                self.current_cell = best_cell
                self.current_diff = min_diff
                self.bad_frame_count = 0  # reset after switching

        grid_i, grid_j = self.current_cell

        # ---- VISUALIZATION ----
        depth_norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        frame_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        # Draw grid
        for i in range(1, grid_cols):
            cv2.line(frame_colored, (i * cell_w, 0), (i * cell_w, height), (255, 255, 255), 1)
        for i in range(1, grid_rows):
            cv2.line(frame_colored, (0, i * cell_h), (width, i * cell_h), (255, 255, 255), 1)

        # Annotate
        for idx, diff in enumerate(differences):
            i, j = valid_indices[idx]
            x, y = j * cell_w, i * cell_h
            cv2.putText(frame_colored, f"{diff:.2f}",
                        (x + 5, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1)

        # ---- LANDING ----
        if (self.current_diff <= flatness) and (max_alt <= altitude >= final_alt):
            scale_factor = compute_angle_scale_5x5(altitude, hfov, vfov)
            top_left_x = grid_j * cell_w
            top_left_y = grid_i * cell_h
            bottom_right_x = top_left_x + cell_w
            bottom_right_y = top_left_y + cell_h

            x_avg = (top_left_x + bottom_right_x) / 2
            y_avg = (top_left_y + bottom_right_y) / 2

            x_ang = (x_avg - width / 2) * (hfov / width)
            y_ang = (y_avg - height / 2) * (vfov / height)

            x_dist = altitude * np.tan(-y_ang)
            y_dist = altitude * np.tan(x_ang)

            if (abs(x_dist))>=0.2 and (abs(y_dist))>=0.2: #only send command if the target is outside a 20cm radius to prevent jitter
                if abs(x_dist) <= safe_land_radius and abs(y_dist) <= safe_land_radius:
                    self.land_counter += 1

            self.get_logger().info(f"[Landing target → x: {x_dist:.2f}m., y: {y_dist:.2f}m., scale: {scale_factor:.2f}]")
            self.get_logger().info(f"[Selected Cell: ({grid_i}, {grid_j}), Flatness: {self.current_diff:.2f}]")
            self.get_logger().info(f"[X angle: {x_ang:.2f}rad, Y angle: {y_ang:.2f}rad, Mode: {mode}]")
            time.sleep(0.04) #25Hz
            # send_land_message(x_ang*scale_factor, y_ang*scale_factor)
            ct = time.time ()
            delay_s = ct - st
            self.get_logger().info(f"Solver Time: {delay_s:.3f} seconds")
            if self.land_counter == 1:
                VehicleMode(vehicle,"GUIDED")
                self.get_logger().info("Entering GUIDED mode for landing...")
                time.sleep(0.1)
                send_position_setpoint(vehicle, x_dist, y_dist, -altitude, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED)
                # time.sleep(3)
                while True:
                    dist = dist_to_target(x_dist, y_dist)
                    self.get_logger().info(f"Distance to target: {dist:.2f} m")
                    if dist <= 0.5:  
                        self.get_logger().info("Within 0.5m of target, initiating LAND mode")
                        time.sleep(1)
                        break

                VehicleMode(vehicle,"LAND")
                self.get_logger().info("Landing initiated...")

            if altitude <= final_alt:
                self.get_logger().info("Landing Final Altitude Reached")
                self.destroy_node()

            # Draw selected cell
            cv2.rectangle(
                frame_colored,
                (top_left_x, top_left_y),
                (bottom_right_x, bottom_right_y),
                (0, 255, 0),
                2
            )
            

def main(args=None):
    #status = status_check(vehicle)
    status = input("status:")
    print("Vehicle State: ",status)

    if ('Critical' in status) or ('Emergency' in status):
        rclpy.init(args=args)
        node = SafeLander()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
