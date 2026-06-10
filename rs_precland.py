###########DEPENDENCIES################
import time
import math
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
from math import atan2
import pyrealsense2 as rs
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

#######VARIABLES####################
fcu_addr = '/dev/ttyACM0' #'udp:127.0.0.1:14560'
##Aruco
id_to_find = 72
marker_size = 40
final_alt = 2
cm_to_m = 100
cam_orient = 3 #1 for 0 deg downfacing, 2 for 90 deg yaw, 3 180 deg yaw, 4 for 270 deg yaw
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

arm_status_condition = False
parameters = aruco.DetectorParameters_create()
# parameters = aruco.DetectorParameters() #linux
##
calib_path="/home/deathstroke/Vision_Precision_Landing/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
##
##Counters and script triggers
found_count=0
notfound_count=0
first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
counter  = 0
rng_alt = 0
prev_state = np.array([[0],[0],[0],[0]]) #initialisation state
prev_cov = np.eye(4)*0.025 #initial covariance

frame_np = None
bridge = CvBridge()
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

def set_parameter(vehicle, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(vehicle.target_system,vehicle.target_component,param_name.encode('utf-8'),param_value,param_type)
    #usage set_parameter(vehicle, "PARAM_NAME", 1)

def get_rangefinder_data(vehicle):
    global rng_alt
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            rng_alt = msg.current_distance/100  # in meters
        return rng_alt

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def image_callback(msg):
    global frame_np
    frame_np = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def lander():
    global first_run,notfound_count,found_count,marker_size,start_time,prev_state,prev_cov, frame_np
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()
        
    rclpy.spin_once(ros_node, timeout_sec=0.01)

    if frame_np is None:
        return

    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)

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
            
            x_ang = atan2((x_avg - cameraMatrix[0][2]), cameraMatrix[0][0])
            y_ang = atan2((y_avg - cameraMatrix[1][2]), cameraMatrix[1][1])

            #########################################################
                
            if cam_orient == 1:
                send_land_message(x_ang,y_ang) #for 0 degree rotation of camera
            elif cam_orient == 2:
                send_land_message(-y_ang,x_ang) #for 90 degree rotation of camera
            elif cam_orient == 3:
                send_land_message(-x_ang,-y_ang) #for 180 degree rotation of camera
            elif cam_orient == 4:
                send_land_message(y_ang,-x_ang) #for 270 degree rotation of camera

            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print(f" MARKER POSITION (cartesian): x={x:.2f} y={y:2f} z={z:.2f}")
            found_count = found_count+1
            msg = String()
            msg.data = "True"
            aruco_data.publish(msg)
            
        else:
            msg = String()
            msg.data = "False"
            aruco_data.publish(msg)
            notfound_count = notfound_count+1
            print("ARUCO: "+str(id_to_find)+" NOT FOUND IN FRAME.")
            
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1


def flightMode(vehicle):
    global mode
    vehicle.recv_match(type='HEARTBEAT', blocking=False)
    mode = vehicle.flightmode

    return mode

####################### MAIN DRONE PARAMETERS ###########################

########### main vehicle parameters #####
vehicle = connect(fcu_addr)
enable_data_stream(vehicle,stream_rate=200)
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
rclpy.init()
ros_node = rclpy.create_node('Precision_Landing')

image_sub = ros_node.create_subscription(
    Image,
    '/camera/camera/color/image_raw',
    image_callback,
    1
)
aruco_data = ros_node.create_publisher(String, 'aruco_detection_state', 1)

set_parameter(vehicle,'PLND_ENABLED', 1)
set_parameter(vehicle,'PLND_TYPE',1) ##1 for companion computer
set_parameter(vehicle,'PLND_EST_TYPE', 0) # 0 for raw sensor, 1 for kalman filter pos estimation
set_parameter(vehicle,'LAND_SPEED',40) ##Descent speed of 30cm/s
set_parameter(vehicle,'PLND_XY_DIST_MAX', 8)
set_parameter(vehicle,'PLND_LAG',0.0519) #lag with ros2 and rsd455

while True:
    lander()
##################### END OF SCRIPT ############################
