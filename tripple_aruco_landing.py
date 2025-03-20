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
altitudes = [3,1]
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
  
def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
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

def aruco_marker_scanner():
    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame,(horizontal_res,vertical_res))
        frame_np = np.array(frame)    #array transformation 
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)   #grey image conversion
        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        if ids is not None:
            ids[0] = ids_to_find
            print(f"Marker ID Found: {ids[0]}")
            vehicle.mode = VehicleMode('LAND')
            time.sleep(0.1)
            break
        else:
            print("No marker found. Scanning...")
            marker_scan_not_found+=1
            if marker_scan_not_found==5:
                vehicle.mode = VehicleMode('GUIDED')
                print("Vehicle now in GUIDED mode")
                time.sleep(0.1)

            if marker_scan_not_found==5:
                send_position_setpoint(-2,0,0)
                print("Moving forward to find marker...")
                time.sleep(1)
            
            if marker_scan_not_found==10:
                send_position_setpoint(2,2,0)
                print("Moving right to find marker...")
                time.sleep(1)

            if marker_scan_not_found==15:
                send_position_setpoint(0,-4,0)
                print("Moving left to find marker...")

            if marker_scan_not_found==20:
                send_position_setpoint(2,2,0)
                print("Moving forward to find marker...")
                time.sleep(1)

            if marker_scan_not_found==25:
                print("No marker found")
                vehicle.mode = VehicleMode('LAND')
                print("Vehicle now in LAND mode")
                time.sleep(0.1)
                break
        print("Scanning for marker...")


def lander():
    global first_run,notfound_count,found_count,marker_size,start_time
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
            ##########################
            
            ########## Pitch #########
            pitch_rad = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
            pitch_deg = np.degrees(pitch_rad)
            pitch = round((pitch_deg+360)%360,2)
            ##########################
            
            ########## marker position extraction ######
            x, y, z = tvec[0], tvec[1], tvec[2]
            #############################################
            
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
                send_land_message(x_ang,y_ang)
            else:
                send_land_message(x_ang,y_ang)
                pass
            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print(f" MARKER POSITION (cartesian): x={x:.2f} y={y:2f} z={z:.2f}")
            print(" MARKER POSITION (angular): Yaw=" +str(yaw)+ " Roll=" +str(roll)+ " Pitch="+str(pitch))
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

    if vehicle.armed==True:
        home_lat = vehicle.home_location.lat
        home_lon = vehicle.home_location.lon
        if (home_lat!=None) and (home_lon!=None):
            home_coords[0] = home_lat
            home_coords[1] = home_lon

    return home_coords[0],home_coords[1]
    

def main_lander():
    global counter
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
############### first 3D fix location as home location (Static Home Location) ######### 
#home_lat = vehicle.location.global_relative_frame.lat
#home_lon = vehicle.location.global_relative_frame.lon
#wp_home =  LocationGlobalRelative(home_lat,home_lon,takeoff_height)
###################################################

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
            aruco_marker_scanner()
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