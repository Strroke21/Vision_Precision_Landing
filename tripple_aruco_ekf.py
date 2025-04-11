###########DEPENDENCIES################
import time
import math
import argparse

from dronekit import connect, VehicleMode,LocationGlobalRelative
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np
#######VARIABLES####################
##Aruco
ids_to_find = [72,75,99]
marker_sizes = [26,13,8] #cm
altitudes = [4,1.5]
takeoff_height = 10
lander_height = 10

marker_scan_not_found = 0

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)


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

first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
counter  = 0
manualArm=False ##If True, arming from RC controller, If False, arming from this script.

prev_state = np.array([[0],[0],[0],[0]]) #initialisation state
prev_cov = np.eye(4)*0.025 #initial covariance
#########FUNCTIONS#################

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    if not connection_string:
        connection_string = '/dev/ttyACM0'

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle

def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon

    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        if currentDistance<distanceToTargetLocation*.02:
            print("Reached target waypoint.")
            time.sleep(2)
            break
        time.sleep(1)
    return None


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    
    vehicle.mode = VehicleMode("GUIDED")
            
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    if manualArm == False:
        vehicle.armed = True
        while vehicle.armed == False:
            print("Waiting for vehicle to be armed")
            time.sleep(1)
    else:
        if vehicle.armed == False:
            print("Exiting script. manualArm set to True but vehicle not armed.")
            print("Set manualArm to True if desiring script to arm the drone.")
            return None

    print("Propellers are spinning...")
    vehicle.simple_takeoff(targetHeight)

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

    return None

############## desired yaw function ##############  
def condition_yaw(heading,relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw, 0 for short turn
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    

########### fake rangefinder ########
def send_distance_message(z):
    msg = vehicle.message_factory.distance_sensor_encode(
        0, #time sync system boot !not used
        20, #minimum distance
        7000, #max distance
        z, #current distance must be integer
        0, #type=raw camera !not used
        0, #onboard id !not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, #camera facing down
        0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()


def send_position_setpoint(pos_x, pos_y, pos_z):

    # Send MAVLink command to send position
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111111000,        # type_mask (only for postion)
        pos_x, pos_y, pos_z,   # position 
        0, 0, 0,                 # velocity in m/s (not used)
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
  
  
#################### Landing Target Function ##############  
  
def send_land_message(x_ang,y_ang, pos_x, pos_y):
    msg = vehicle.message_factory.landing_target_encode(
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
    vehicle.send_mavlink(msg)
    vehicle.flush()


def controlServo(servo_number,pwm_value):
    msg = vehicle.message_factory.command_long_encode(
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
    vehicle.send_mavlink(msg)
    vehicle.flush()

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


def lander():
    global first_run,notfound_count,found_count,marker_size,start_time,prev_state,prev_cov
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()

    altitude = vehicle.rangefinder.distance

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
            #############################################

            ########### (drone current location) #########
            dronelat = vehicle.location.global_relative_frame.lat
            dronelon = vehicle.location.global_relative_frame.lon
            ##############################################

            ########### (drone port gps location) ########
            dport_lat, dport_lon = home_loc()
            ##############################################
            x_gps, y_gps = geo_distance_components(dronelat, dronelon, dport_lat, dport_lon)
            ############## fused state ###################
            updated_state, updated_cov = kalman_filter(prev_state,prev_cov,x_gps,y_gps,x,y)
            x_fuse = (updated_state[0, 0])/100 #fused x position (m)
            y_fuse = (updated_state[1, 0])/100 #fused y position (m)
            prev_cov = updated_cov
            prev_state = updated_state
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
            
                
            if vehicle.mode!='LAND':
                vehicle.mode = VehicleMode('LAND')
                
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(x_ang,y_ang,x_fuse,y_fuse)
            else:
                send_land_message(x_ang,y_ang,x_fuse,y_fuse)
                pass

            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print(f" MARKER POSITION (cartesian): x={x:.2f} y={y:2f} z={z:.2f}")
            #print(" MARKER POSITION (angular): Yaw=" +str(yaw)+ " Roll=" +str(roll)+ " Pitch="+str(pitch))
            print(f"ekf_X={x_fuse:.2f} ekf_Y={y_fuse:.2f} ekf_variance={updated_cov}")
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


def home_loc():
    global home_coords
    if vehicle.armed==True:
        home_lat = vehicle.home_location.lat
        home_lon = vehicle.home_location.lon
        if (home_lat!=None) and (home_lon!=None):
            home_coords[0] = home_lat
            home_coords[1] = home_lon

    return home_coords[0],home_coords[1]
    

def main_lander():
    global counter, found_count, notfound_count, start_time
    while True:
        if (vehicle.armed==True):
            lander()
        elif (vehicle.armed!=True):
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
            controlServo(8,2000)
            print("[Battery UnLocked.]")
            counter = 0
            found_count = 0
            notfound_count = 0
            time.sleep(1)
            break


def first_arm_loc_check():
    while True:
        if vehicle.armed==True:
            print("[Home Location Set.]")
            break
        else:
            print("[Waiting for vehicle to be armed to set Home Location...]")

####################### MAIN DRONE PARAMETERS ###########################

########### main vehicle parameters #####
vehicle = connectMyCopter()
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 # 0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 30 ##Descent speed of 30cm/s

#storing first arm location as home location
first_arm_loc_check()
home_coords = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon]

while True:
    
    counter+=1
    if counter==1:
        while True:
            if vehicle.armed==True:
                controlServo(8,1000)
                print("[Battery Locked.]")
                break
            else:
                print("[Waiting for vehicle to be armed...]")

    ########### home location coordinates (Dynamic) ########
    home_lat, home_lon = home_loc() 
    wp_home = LocationGlobalRelative(home_lat,home_lon,takeoff_height)
    ##############################################
    
    ########### current location from drone #######
    lat_current = vehicle.location.global_relative_frame.lat
    lon_current = vehicle.location.global_relative_frame.lon
    current_altitude = vehicle.location.global_relative_frame.alt
    
    ######### distance_to_home calculation ########
    wp_current = LocationGlobalRelative(lat_current,lon_current, current_altitude)
    distance_to_home = get_distance_meters(wp_current,wp_home)
    altitude = vehicle.rangefinder.distance
        
    if (vehicle.mode=='RTL'):
        if (altitude <= lander_height) and (distance_to_home <= home_radius):
            vehicle.mode = VehicleMode('LAND')
            print("[Vehicle is now in LAND mode]")
            time.sleep(1)
            main_lander()

        else:
            print("[Waiting to reach home location for landing...]")

    elif (vehicle.mode=='LAND' and vehicle.armed==True):
        main_lander()
    
    time.sleep(1)
    print(f"[Distance to Home]: {distance_to_home:.2f} [m.] [Altitude]: {altitude:.2f} [m.] [Waiting to acquire Landing Point...]")


##################### END OF SCRIPT ############################