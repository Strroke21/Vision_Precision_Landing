import cv2
from cv2 import aruco
import numpy as np
import time
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
#############################

width = 640
height = 480
bridge = CvBridge()
latency_sum = 0
latency_count = 0
global frame_np
frame_np = None
counter=0
st = time.time()
start_time=time.time()
viewVideo=True
############ARUCO/CV2############
id_to_find=72 
marker_size= 30 #cm
realWorldEfficiency=.7 ##Iterations/second are slower when the drone is flying. This accounts for that
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters() #linux

calib_path="/home/deathstroke/Desktop/Vision_Precision_Landing/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
#############################

def avg_delay(latency):
    global latency_sum, latency_count
    latency_sum += latency
    latency_count += 1
    return latency_sum / latency_count

def image_callback(msg):
    global frame_np, topic_delay
    frame_np = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    topic_delay = time.time() - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9

rclpy.init()
ros_node = rclpy.create_node('Aruco_Tester')

image_sub = ros_node.create_subscription(
    Image,
    '/camera/camera/color/image_raw',
    image_callback,
    1)

while True:

    rclpy.spin_once(ros_node, timeout_sec=0.01)

    if frame_np is None:
        continue

    ct = time.time()
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
        
    cv2.imshow('Aruco Tracker',frame_np)
        
    if cv2.waitKey(1) & 0xFF == ord('Q'):
        break
    
    ids=''
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(image=gray_img)
    if ids is not None:
        counter+=1
        print(f"detection per second: {counter/(ct-st)}")
        print("Found these IDs in the frame:")
        print(ids)
    if ids is not None and ids[0] == id_to_find:
        ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
        rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
        
        ####### heading (yaw) #####
        R, _ = cv2.Rodrigues(rvec)
        yaw_rad = np.arctan2(R[1,0], R[0,0])
        yaw_deg = (np.degrees(yaw_rad))
        yaw = round((yaw_deg+360) %360,2)
        
        ########### Roll #########
        roll_rad = np.arctan2(R[2,1], R[2,2])
        roll_deg = (np.degrees(roll_rad))
        roll = round((roll_deg+360) %360,2)
        ##########################
        
        ########## Pitch #########
        pitch_rad = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
        pitch_deg = (np.degrees(pitch_rad))
        pitch = round((pitch_deg+360) %360,2)
        ##########################
        
        
        x="{:.2f}".format(tvec[0])
        y="{:.2f}".format(tvec[1])
        z="{:.2f}".format(tvec[2])
        
        marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
        print("Yaw:" +str(yaw)+ " Roll:" +str(roll)+ " Pitch:"+str(pitch), marker_position)
        latency = (time.time() - ct) + topic_delay
        average_latency = avg_delay(latency)

        print(f"Latency: {latency} seconds, Average Latency: {average_latency} seconds")
        if viewVideo==True:
            aruco.drawDetectedMarkers(frame_np,corners)
            #aruco.drawAxis(frame_np,cameraMatrix,cameraDistortion,rvec,tvec,10) #rpi debian 
            text="{}".format(z)
            frame_np = cv2.putText(frame_np,text,(5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,0),1)
            cv2.imshow('Aruco Tracker',frame_np)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break        
            
    else:
        print(f"ARUCO: {id_to_find} NOT FOUND IN FRAME.")
        print("")


