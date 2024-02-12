# Vision Precision Landing using computer vision
The vision precision landing and position alignment can be achieved using computer vision:

- The camera distortion and camera matrix are calculated for picam v2. specify these file paths in the script.
- if you are using a different camera then calibrate the camera using calibration script.

The supported platform for this are:
* Ardupilot
* PX4

Hardware Required:
* Raspberry Pi4 with debian Bullseye
* Picam v2
  
Aruco Marker:
* Dictionary: Original Dictionary
* ID: 72
* Size: 16 cm

# camera calibration:
if you are using a different camera then edit calibrate.py script as per your convinience where you want to save your camera instrinsics
- the camera calibration can be done using 9x6 chessboard in the terminal using following command
- use the following resolution
- >> calibrate.py  --mm 16 --width 640 --height 480
- [Note: --mm is size of one block in chessboard and --width is horizontal resolution --height is vertical resolution]
- take 30 good images of chessboard for camera calibration.
- ![image](https://github.com/Strroke21/Vision_Precision_Landing/assets/93963494/165b8a65-d5db-4406-b580-ecfc854aa831)


# Landing Target Protocol
The landing target services/message communicates the position of one or more targets from MAVLink positioning system(s) to an autopilot. A multicopter or VTOL system can use the message to land with far greater positional accuracy than provided by conventional GPS (GPS provides position within several meters while a landing-target system might reasonably provide centimetre-level precision landing).

A positioning system might typically consist of an onboard companion computer with a vision system that can detect a light beacon or target image. Radio beacons and different types of visual markers and tags are also supported.

# Protocol Messages
The message used by this protocol is [LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET). This is *broadcast* by the positioning system to indicate the position of a particular target at a particular time.

**(The required broadcast rate depends on the landing speed and desired accuracy; start with rates between 10 Hz and 50 Hz and tune performance as needed.)**

The original MAVLink 1 message was designed with the assumption that the target is captured from a downward facing camera, and provides fields that are relative to the captured image. MAVLink 2 extended the message to provide positional information in terms of standard frames ([MAV_FRAME](https://mavlink.io/en/messages/common.html#MAV_FRAME)), a quaternion and the type of landing targets ([LANDING_TARGET_TYPE](https://mavlink.io/en/messages/common.html#LANDING_TARGET_TYPE)). This allows more flexibility for the types of target that can be supported, simplifies the code required by the autopilot, and allows the autopilot to control both landing position and orientation on (some) targets.

Different systems may support either (or presumably both) sets of fields. These are discussed below.

# Target Relative to Captured Images
The [LANDING_TARGET](https://mavlink.io/en/messages/common.html#LANDING_TARGET) fields that are relative to a captured image are shown below:

| Field Name | Type | Units | Values | Description |
| --- | --- | --- | --- | --- |
| angle_x | float | rad |  | X-axis angular offset of the target from the center of the image |
| angle_y | float | rad |  | Y-axis angular offset of the target from the center of the image |
| distance | float | m |  | Distance to the target from the vehicle |
| size_x | float | rad |  | Size of target along x-axis |
| size_y | float | rad |  | Size of target along y-axis |

The positional information can be interpreted as described below.

Imagine a ray pointing from the camera's principal point to the target. The x angle (`angle_x`) is the angle spanned by that ray and the x-axis of the image plane. The same holds for the y angle (`angle_y`). In other words, the x angle is a function of the x pixel coordinate of the target (denoted by *u̅* in the image below), the y angle is a function of the y pixel coordinate (denoted *v* in the image below). Using the angle rather than *u̅/v̅* pixel coordinates has the advantage that the effect of the camera lens is already accounted for. Otherwise the receiver of the message would need to know the camera field of view etc.

![Untitled](https://github.com/Strroke21/Vision_Precision_Landing/assets/93963494/1631cdbd-516c-4914-a093-b522f280b94e)

The sizes in x and y direction are analogous (`size_x`/`size_y`). They describe the angle between the smallest and biggest pixel in x/y direction respectively of the target as seen in the image.

**(ArduPilot supports messages with these fields if position_valid is `0`.)**

## **Target as Position/Quaternion (MAVLink 2 and later)**

The message fields that are used to provide target information as a position/quaternion are shown below. Field meaning and use is clear from the description.

| Field Name | Type | Units | Values | Description |
| --- | --- | --- | --- | --- |
| frame | uint8_t |  | https://mavlink.io/en/messages/common.html#MAV_FRAME | Coordinate frame used for following fields. |
| x | float | m |  | X Position of the landing target in MAV_FRAME |
| y | float | m |  | Y Position of the landing target in MAV_FRAME |
| z | float | m |  | Z Position of the landing target in MAV_FRAME |
| q | float[4] |  |  | Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0) |
| type | uint8_t |  | https://mavlink.io/en/messages/common.html#LANDING_TARGET_TYPE | Type of landing target |
| position_valid | uint8_t |  |  | Boolean indicating whether these position field values are populated with valid position target information (1: valid, 0: invalid). The default is '0', so that if the fields are not populated the default-zero values are not interpreted as a valid target position. |
- **PX4 supports this form of positioning in [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) (only). The original (MAVLink 1) fields are ignored.**
- **ArduPilot supports this form of positioning in [MAV_FRAME_BODY_FRD](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD) (only). `position_valid` must be `1` and `distance` must be filled.**


