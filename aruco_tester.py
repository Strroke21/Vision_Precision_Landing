import cv2
from cv2 import aruco
import numpy as np
import time
import sys
#############################

width=640
height=480
align_yaw=None
#cap = WebcamVideoStream(src=0).start()

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)

output_file = 'output_video.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30.0
frame_size = (width,height)
out = cv2.VideoWriter(output_file,fourcc,fps,frame_size)

viewVideo=True
if len(sys.argv)>1:
    viewVideo=sys.argv[1]
    if viewVideo=='0' or viewVideo=='False' or viewVideo=='false':
        viewVideo=False
############ARUCO/CV2############
id_to_find=72
marker_size= 30 #cm
realWorldEfficiency=.7 ##Iterations/second are slower when the drone is flying. This accounts for that
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) 
parameters = aruco.DetectorParameters() #_create() #rpi debian

# calib_path="/home/deathstroke/Desktop/Vision_Precision_Landing/video2calibration/calibrationFiles/"
# cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
# cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
cameraMatrix = np.array([[1.11566446e+03, 0.00000000e+00, 2.83173405e+02],
                        [0.00000000e+00, 1.09675922e+03, 2.63276316e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

cameraDistortion = np.array([-1.03854256e-01,  1.82737214e+01, 1.18521341e-02, -1.35474190e-02, -2.48315901e+02])

latency_sum = 0
latency_count = 0
latency_min = float('inf')
latency_max = float('-inf')

def avg_delay(latency):
    global latency_sum, latency_count, latency_min, latency_max

    latency_sum += latency
    latency_count += 1

    # update min
    if latency < latency_min:
        latency_min = latency

    # update max
    if latency > latency_max:
        latency_max = latency

    avg = latency_sum / latency_count

    return avg, latency_min, latency_max

#############################
seconds=0
if viewVideo==True:
    seconds=1000000
    print("Showing video feed if X11 enabled.")
    print("Script will run until you exit.")
    print("-------------------------------")
    print("")
else:
    seconds=5
counter=0
counter=float(counter)
st = time.time()
start_time=time.time()
while time.time()-start_time<seconds:
    ct = time.time()
    ret,frame = cap.read() #for Threaded webcam
#    frame = cv2.resize(frame,(width,height))
    
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
    
    if ret: 
        
        cv2.imshow('Aruco Tracker',frame)
        
    if cv2.waitKey(1) & 0xFF == ord('Q'):
        break
    
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
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
        latency = time.time() - ct
        avg_latency, latency_min, latency_max = avg_delay(latency)
        print(f"Average Latency: {avg_latency * 1000} ms")
        print(f"Minimum Latency: {latency_min * 1000} ms")
        print(f"Maximum Latency: {latency_max * 1000} ms")
        if viewVideo==True:
            aruco.drawDetectedMarkers(frame_np,corners)
            #aruco.drawAxis(frame_np,cameraMatrix,cameraDistortion,rvec,tvec,10) #rpi debian 
            text="{}".format(z)
            frame_np = cv2.putText(frame_np,text,(5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,0),1)
            cv2.imshow('Aruco Tracker',frame_np)
            out.write(frame_np)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:
        print(f"ARUCO: {id_to_find} NOT FOUND IN FRAME.")
        print("")
    counter=float(counter+1)
out.release()

if viewVideo==False:
    frequency=realWorldEfficiency*(counter/seconds)
    print("")
    print("")
    print("---------------------------")
    print("Loop iterations per second:")
    print(frequency)
    print("---------------------------")

    print("Performance Diagnosis:")
    if frequency>10:
        print("Performance is more than enough for great precision landing.")
    elif frequency>5:
        print("Performance likely still good enough for precision landing.")
        print("This resolution likely maximizes the detection altitude of the marker.")
    else:
        print("Performance likely not good enough for precision landing.")
        print("MAKE SURE YOU HAVE A HEAT SINK ON YOUR PI!!!")
    print("---------------------------")
cv2.destroyAllWindows()


