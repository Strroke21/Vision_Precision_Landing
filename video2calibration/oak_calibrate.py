#!/usr/bin/env python3
import cv2
import numpy as np
import depthai as dai
import os

# ---------------------------
# SETTINGS
# ---------------------------
WIDTH = 640
HEIGHT = 480
PATTERN_SIZE = (8, 5)   # inner corners (IMPORTANT)
SQUARE_SIZE = 20        # mm (match your real chessboard)
IMAGE_GOAL = 30         # number of good frames

OUTPUT_DIR = "./calibrationFiles"
DEBUG_DIR = "./pictures"

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(DEBUG_DIR, exist_ok=True)

# ---------------------------
# DEPTHAI PIPELINE
# ---------------------------
pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(WIDTH, HEIGHT)
cam.setInterleaved(False)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.preview.link(xout.input)

# ---------------------------
# CHESSBOARD SETUP
# ---------------------------
pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
pattern_points *= SQUARE_SIZE

obj_points = []
img_points = []

# ---------------------------
# CAPTURE LOOP
# ---------------------------
print("Press 'q' to quit early")
print("Move chessboard around for good calibration...\n")

frame_count = 0
good_frames = 0

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("video")

    while True:
        frame = q.get().getCvFrame()
        frame_count += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            PATTERN_SIZE,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()

        if found:
            cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            )

            obj_points.append(pattern_points)
            img_points.append(corners)

            good_frames += 1
            print(f"[INFO] Captured {good_frames}/{IMAGE_GOAL}")

            cv2.drawChessboardCorners(display, PATTERN_SIZE, corners, found)

            # Save debug image
            cv2.imwrite(os.path.join(DEBUG_DIR, f"{good_frames:02d}.png"), display)

        cv2.imshow("Calibration", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if good_frames >= IMAGE_GOAL:
            break

# ---------------------------
# CALIBRATION
# ---------------------------
if len(obj_points) < 10:
    print("❌ Not enough valid frames for calibration")
    exit(1)

h, w = gray.shape[:2]

print("\n[INFO] Calibrating...")

ret, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points,
    img_points,
    (w, h),
    None,
    None
)

# ---------------------------
# RESULTS
# ---------------------------
print("\n=== CALIBRATION RESULTS ===")
print("RMS Error:", ret)
print("\nCamera Matrix:\n", camera_matrix)
print("\nDistortion Coefficients:\n", dist_coefs.ravel())

# ---------------------------
# SAVE FILES
# ---------------------------
np.savetxt(os.path.join(OUTPUT_DIR, "cameraMatrix.txt"),
           camera_matrix, delimiter=',')

np.savetxt(os.path.join(OUTPUT_DIR, "cameraDistortion.txt"),
           dist_coefs, delimiter=',')

print("\n✅ Saved to:", OUTPUT_DIR)

cv2.destroyAllWindows()