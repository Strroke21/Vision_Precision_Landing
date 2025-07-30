###########DEPENDENCIES################
import time
import math
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
from math import radians, cos, sin, sqrt, atan2

#######VARIABLES####################
fcu_addr = '/dev/ttyACM0'
##Aruco
ids_to_find = [72,75,99]
marker_sizes = [26,13,8] #cm
altitudes = [4,1.5]
takeoff_height = 10
lander_height = 10

marker_scan_not_found = 0

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

arm_status_condition = False
parameters = aruco.DetectorParameters_create()
##

##Camera
horizontal_res = 640
vertical_res = 480
cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path="/home/pi/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
##
home_radius = 10
##Counters and script triggers
found_count=0
notfound_count=0
mode = 'STABILIZE'
first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
counter  = 0
manualArm=False ##If True, arming from RC controller, If False, arming from this script.
rng_alt = 0
prev_state = np.array([[0],[0],[0],[0]]) #initialisation state
prev_cov = np.eye(4)*0.025 #initial covariance
#########FUNCTIONS#################

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle


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

############## desired yaw function ##############  
def condition_yaw(heading,relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.mav.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw, 0 for short turn
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.mav.send(msg)

#################### Landing Target Function ##############  
  
def send_land_message(x_ang,y_ang, pos_x, pos_y):
    msg = vehicle.mav.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x_ang,
        y_ang,
        0, #distance to marker (m)
        0, #size_x (rad)
        0, #size_y (rad)
        pos_x, #position_x of target (m)
        pos_y, #position_y of target (m)
        ) 
    vehicle.mav.send(msg)

def controlServo(servo_number,pwm_value):
    msg = vehicle.mav.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value,
            0,
            0,
            0,
            0,
            0)
    vehicle.mav.send(msg)

def set_parameter(vehicle, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(vehicle.target_system,vehicle.target_component,param_name.encode('utf-8'),param_value,param_type)
    #usage set_parameter(vehicle, "PARAM_NAME", 1)

def get_rangefinder_data(vehicle):
    global rng_alt
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is not None:
            rng_alt = msg.current_distance/100  # in meters
        return rng_alt

def get_global_position(vehicle):
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
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

    while True:
        msg1=home_location(vehicle)
        if msg1 is not None:
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

def geo_distance_components(lat1,lon1,lat2,lon2):
    # Earth's radius in meters
    R = 6378137

    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Calculate differences
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    # Normalize delta_lon to [-π, π]
    delta_lon = (delta_lon + math.pi) % (2 * math.pi) - math.pi

    # Debugging intermediate values
    mean_lat = (lat1 + lat2) / 2
    # North-South component (y): R * delta_lat
    y = R * delta_lat

    # East-West component (x): R * delta_lon * cos(mean_latitude)
    x = R * delta_lon * math.cos(mean_lat)

    return x,y


def kalman_filter(prev_state, prev_cov, x_gps, y_gps, x_cam, y_cam):

    #state transition model (constant velocity assumed)
    A=np.eye(4)
    # Measurement model (Only measuring position)
    H = np.array([[1, 0, 0, 0],  
                  [0, 1, 0, 0]])  
    Q = np.eye(4) * 0.02 #process noise
    R_gps = np.eye(2) * 1.0  #GPS has more noise 
    R_cam = np.eye(2) * 0.05 #camera noise
    #Predict step
    pred_state = np.dot(A, prev_state)
    pred_cov = np.dot(np.dot(A, prev_cov), A.T) + Q 
    # GPS update
    z_gps = np.array([[x_gps], [y_gps]])  # Measurement
    y_gps = z_gps - np.dot(H, pred_state)  # Residual
    S_gps = np.dot(H, np.dot(pred_cov, H.T)) + R_gps  # Innovation covariance
    K_gps = np.dot(np.dot(pred_cov, H.T), np.linalg.inv(S_gps))  # Kalman gain
    updated_state = pred_state + np.dot(K_gps, y_gps)  # Updated state
    updated_cov = np.dot((np.eye(4) - np.dot(K_gps, H)), pred_cov)  # Updated covariance
        # Camera update
    z_cam = np.array([[x_cam], [y_cam]])  
    y_cam = z_cam - np.dot(H, updated_state)
    S_cam = np.dot(H, np.dot(updated_cov, H.T)) + R_cam
    K_cam = np.dot(np.dot(updated_cov, H.T), np.linalg.inv(S_cam))
    updated_state = updated_state + np.dot(K_cam, y_cam)
    updated_cov = np.dot((np.eye(4) - np.dot(K_cam, H)), updated_cov)

    return updated_state, updated_cov

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def lander():
    global first_run,notfound_count,found_count,marker_size,start_time,prev_state,prev_cov
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()

    altitude = get_rangefinder_data(vehicle)

    if (altitude > altitudes[0]):
        id_to_find = ids_to_find[0]
        marker_size = marker_sizes[0]

    elif (altitudes[1] < altitude < altitudes[0]):
        id_to_find = ids_to_find[1]
        marker_size = marker_sizes[1]

    elif altitude < altitudes[1]:
        id_to_find = ids_to_find[2]
        marker_size = marker_sizes[2]
        
    ret, frame = cap.read()
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    frame_np = np.array(frame)    #array transformation 
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)   #grey image conversion
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

    try:
                 
        if ids is not None and ids[0] == id_to_find:
             
            ############ markers position estimation from opencv############
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion) #markers position 
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])   # rotation and translation vectors
            
            ######heading calculation#######
            R, _ = cv2.Rodrigues(rvec) 
            yaw_rad = np.arctan2(R[1,0], R[0,0])
            yaw_deg = np.degrees(yaw_rad) 
            yaw = round((yaw_deg+360)%360,2)  #%360 sets limit of the yaw scale to 0-360  # 'round' makes heading yaw upto 2 decimals
            ################################
            
            
            ########### Roll #########
            roll_rad = np.arctan2(R[2,1], R[2,2])
            roll_deg = np.degrees(roll_rad)
            roll = round((roll_deg+360)%360,2)
            ##########################B
            
            ########## Pitch #########
            pitch_rad = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
            pitch_deg = np.degrees(pitch_rad)
            pitch = round((pitch_deg+360)%360,2)
            ##########################
            
            ########## marker position extraction ######
            x, y, z = tvec[0], tvec[1], tvec[2]
            ##############################################

            ############ x,y angle calculation in radians #########
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            #########################################################
            mode = flightMode(vehicle)
                
            if mode!='LAND':
                VehicleMode(vehicle,'LAND')
                
                while mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(x_ang,y_ang,x_sum,y_sum)
            else:
                send_land_message(x_ang,y_ang,x_sum,y_sum)
                pass

            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print(f" MARKER POSITION (cartesian): x={x:.2f} y={y:2f} z={z:.2f}")
            found_count = found_count+1
            
            ########### yaw alignment command ###########
            
            if found_count==1:   # it will take yaw value only once and put it in condition_yaw() function
                condition_yaw(yaw,1) #clockwise
                print(f"yaw alignment: {yaw:.2f} deg.")
                print("Heading is aligned with marker")            
            else:
                pass
            #############################################
            
        else:
            notfound_count = notfound_count+1
            
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1


def home_location(vehicle):
    while True:
        msg=vehicle.recv_match(type='HOME_POSITION',blocking=True)
        if msg is not None:
            return [msg.latitude * 1e-7, msg.longitude * 1e-7,msg.altitude * 1e-3]
        print("waiting for home location")
        
def arm_status(vehicle):
    global arm_status_condition
    vehicle.recv_match(type='HEARTBEAT', blocking=False)
    armed = vehicle.motors_armed()
    if armed==128:
        arm_status_condition = True
    elif armed==0:
        arm_status_condition = False
    return arm_status_condition

def home_loc():
    global home_coords
    arm_c = arm_status(vehicle)
    if arm_c==True:
        home_coords = home_location(vehicle)
        print(f"[Home Location]: [lat:] {home_coords[0]:.7f} [lon:] {home_coords[1]:.7f}")
        return home_coords[0],home_coords[1]
    else:
        print("waiting to be armed")

def flightMode(vehicle):
    global mode
    vehicle.recv_match(type='HEARTBEAT', blocking=False)
    mode = vehicle.flightmode

    return mode
    
def main_lander():
    global counter, found_count, notfound_count, start_time
    while True:
        arm_c = arm_status(vehicle)
        if (arm_c==True):
            lander()
        elif (arm_c==False):
            end_time = time.time()
            total_time = end_time - start_time
            total_time = abs(int(total_time))
            total_count = found_count + notfound_count
            freq_lander = total_count / total_time
            print("Total iterations: " + str(total_count))
            print("Total seconds: " + str(total_time))
            print("------------------")
            print("Lander function had a frequency of: " + str(freq_lander))
            print("------------------")
            counter = 0
            found_count = 0
            notfound_count = 0
            time.sleep(1)
            break


def first_arm_loc_check():
    while True:
        arm_c = arm_status(vehicle)
        if arm_c == True:
            print("[Home Location Set.]")
            break
        else:
            print("[Waiting for vehicle to be armed to set Home Location...]")

####################### MAIN DRONE PARAMETERS ###########################

########### main vehicle parameters #####
vehicle = connect(fcu_addr)
enable_data_stream(vehicle,stream_rate=200)
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
set_parameter('PLND_ENABLED', 1)
set_parameter('PLND_TYPE',1) ##1 for companion computer
set_parameter('PLND_EST_TYPE', 0) # 0 for raw sensor, 1 for kalman filter pos estimation
set_parameter('LAND_SPEED',30) ##Descent speed of 30cm/s

#storing first arm location as home location
first_arm_loc_check()
home_coords = [home_location(vehicle)[0], home_location(vehicle)[1]]

while True:
    ########### home location coordinates (Dynamic) ########

    altitude = get_rangefinder_data(vehicle)
    dist_to_home = distance_to_home(vehicle)
    arm_c = arm_status(vehicle)
    mode = flightMode(vehicle)
    if (mode=='RTL'):
        if (altitude <= lander_height) and (dist_to_home <= home_radius):
            VehicleMode(vehicle,'LAND')
            print("[Vehicle is now in LAND mode]")
            time.sleep(1)
            main_lander()

        else:
            print("[Waiting to reach home location for landing...]")

    elif (mode=='LAND' and arm_c==True):
        main_lander()
    
    elif (mode=='AUTO') and (altitude<=lander_height):
        main_lander()
    
    time.sleep(0.5)
    print(f"[Distance to Home]: {dist_to_home:.2f} [m.] [Altitude]: {altitude:.2f} [m.] [Waiting to acquire Landing Point...]")

##################### END OF SCRIPT ############################