#!/usr/bin/env python
import numpy as np
import cv2
import os
import argparse
import yaml
import pickle
import pyrealsense2 as rs

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate D455 RGB camera using chessboard.')
    parser.add_argument('--debug_dir', default='./pictures')
    parser.add_argument('--output_dir', default='./calibrationFiles')
    parser.add_argument('-c', '--corners', default=None)
    parser.add_argument('--height', default=480, type=int)
    parser.add_argument('--width', default=640, type=int)
    parser.add_argument('--mm', default=22, type=int)  # square size in mm
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    os.makedirs(args.debug_dir, exist_ok=True)

    # -----------------------------
    # RealSense D455 RGB Setup
    # -----------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, 30)
    pipeline.start(config)

    # -----------------------------
    # Chessboard config
    # -----------------------------
    pattern_size = (8, 5)  # ✅ for 9x6 squares

    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= args.mm

    obj_points = []
    img_points = []

    image_count = 0
    image_goal = 30

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            cv2.imshow("RGB", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            print(f"Searching frame {image_count}...")

            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_NORMALIZE_IMAGE |
                cv2.CALIB_CB_FAST_CHECK
            )

            if not found:
                print("Not found")
                continue

            # refine corners
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)

            obj_points.append(pattern_points)
            img_points.append(corners)

            image_count += 1

            # visualization
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            cv2.imshow("Detected", vis)

            # save debug
            cv2.imwrite(os.path.join(args.debug_dir, f"{image_count:04d}.png"), vis)

            print(f"Captured {image_count}/{image_goal}")

            # delay for pose variation
            cv2.waitKey(300)

            if image_count >= image_goal:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    # -----------------------------
    # Calibration
    # -----------------------------
    print("\nPerforming calibration...")
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (args.width, args.height), None, None
    )

    print("RMS:", rms)
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion:\n", dist_coefs.ravel())

    # save results
    np.savetxt(os.path.join(args.output_dir, "cameraMatrix.txt"), camera_matrix, delimiter=',')
    np.savetxt(os.path.join(args.output_dir, "cameraDistortion.txt"), dist_coefs, delimiter=',')

    if args.corners:
        with open(args.corners, 'wb') as fw:
            pickle.dump(img_points, fw)
            pickle.dump(obj_points, fw)
            pickle.dump((args.width, args.height), fw)