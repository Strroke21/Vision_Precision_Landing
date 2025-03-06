import cv2
import cv2.aruco as aruco
import numpy as np
import time
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2

############ARUCO/CV2############
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
###########drone port##################

fcu_address = 'udp:'

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def flightMode(vehicle):
    vehicle.recv_match(type='HEARTBEAT', blocking=True)
    # Wait for a 'HEARTBEAT' message
    mode = vehicle.flightmode

    return mode

def arm(vehicle):
    #arm the drone
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
def drone_takeoff(vehicle, altitude):
    
    # Send MAVLink command to takeoff
    vehicle.mav.command_long_send(
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,                          # confirmation
        0,                          # param1 (min pitch, not used)
        0,                          # param2 (empty for now, not used)
        0,                          # param3 (empty for now, not used)
        0,                          # param4 (yaw angle in degrees, not used)
        0,                          # param5 (latitude, not used)
        0,                          # param6 (longitude, not used)
        altitude                    # param7 (altitude in meters)
    )


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


def home_location(vehicle):

    vehicle.mav.command_long_send(vehicle.target_system,vehicle.target_component,mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,0,0,0,0,0,0,0,0)
    msg=vehicle.recv_match(type='HOME_POSITION',blocking=True)
    if msg:
        return [msg.latitude * 1e-7, msg.longitude * 1e-7,msg.altitude * 1e-3]
    
    #0:lat 1:lon 2:alt  

def get_global_position(vehicle):
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat/1e7 # lat
    lon = msg.lon/1e7 # lon
    alt = msg.alt/1000  # alt
    vx = msg.vx/100 #in m/s
    vy= msg.vy/100 #in m/s
    vz = msg.vz/100 #in m/s
    relative_alt = msg.relative_alt/100 #in m
    hdg = msg.hdg/100 #in deg
    time_boot_ms = msg.time_boot_ms
    return [lat,lon,alt,vx,vy,vz,relative_alt,hdg, time_boot_ms]

def distance_to_home(vehicle):

    msg1=home_location(vehicle)
    
    home_lat=msg1[0]
    home_lon=msg1[1]
    #home_alt=msg1.altitude * 1e-3

    msg2 = get_global_position(vehicle)
    
    current_lat = msg2[0] # lat
    current_lon = msg2[1] # lon
    #current_alt = msg2[2] # alt
    
    R = 6371000  # Earth radius in meters
    dlat = radians(current_lat - home_lat)
    dlon = radians(current_lon - home_lon)
    a = sin(dlat / 2)**2 + cos(radians(home_lat)) * cos(radians(current_lat)) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance #in meters


def front_camera():
    cap=cv2.VideoCapture(4)
    cap.set(3,640)
    cap.set(4,480)

    cameraMatrix   = np.array([[266.82523151,   0,         340.24328138],
                                [  0,         265.35396197, 241.02371401],
                                [  0,           0,           1        ]])
    cameraDistortion   = np.array([-0.29218741,  0.15190219, -0.00076712, -0.00037857, -0.0637175])
    marker_size = 3
    id_to_find=75

    while True:

        ret, frame = cap.read() #for Threaded webcam
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)

        if ret:

            cv2.imshow('Aruco Tracker',frame)

        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break

        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        if ids is not None:
            print("Found these IDs in the frame:")
            print(ids)
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
            R, _ = cv2.Rodrigues(rvec)
            x = tvec[0]
            y = tvec[1]
            z = tvec[2]
            print(f"Found Aruco: {id_to_find}")
            #message to be sent to drone port#
            roll_rad = np.arctan2(R[2,1], R[2,2])
            roll_deg = np.degrees(roll_rad)
            yaw_rad = np.arctan2(R[1,0], R[0,0])
            yaw_deg = np.degrees(yaw_rad)
            pitch_rad = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
            pitch_deg = np.degrees(pitch_rad)
            marker_position=f"MARKER POSITION: x={x:.2f} y={y:.2f} z={z:.2f} Roll: {roll_deg:.2f} Yaw: {yaw_deg:.2f} Pitch: {pitch_deg:.2f}"
            print(marker_position)
            
            if (19<abs(z)<55) and (abs(x)<19) and (abs(pitch_deg)<20):
                print("Drone Orientation on Drone port is valid...")
                # VehicleMode(vehicle,"LOITER")
                # print("Loiter Mode activated...")
                break

            else:
                print("Drone Orientation on Drone port is invalid...")
                print("Taking-off drone to correct orientation...")
                # VehicleMode(vehicle,"GUIDED")
                # print("Guided Mode activated...")
                # time.sleep(1)
                # arm(vehicle)
                # time.sleep(0.5)
                # drone_takeoff(vehicle, 5)
                # print("Taking-off to 5m...")
                # time.sleep(5)
                # VehicleMode(vehicle,"LAND")
                # print("Landing for re-orientation...")
                # time.sleep(0.5)

        else:
            print("Marker not found")


# vehicle = connect(fcu_address)
# print("connected to drone...")

while True:
    # mode = flightMode(vehicle)
    # dist_to_home = distance_to_home(vehicle)
    # print(f"Current Mode: {mode} Distance to Home: {dist_to_home} m.")
    # if (mode=='LAND') and (dist_to_home<=5):
    #     front_camera()

    # else:
    #     print("Waiting for drone to reach home position and enter LAND mode...")
    front_camera()
        
        






