import cv2
import numpy as np
import glob
import os

# ---------------- CONFIGURATION ----------------
# (Columns, Rows) of INTERNAL CORNERS
CHECKERBOARD_SIZE = (9, 6)
# Size of one square in your real world unit (e.g., 2.5 cm)
SQUARE_SIZE = 2.5
IMG_DIR = "stereo_calibration_imgs"
# -----------------------------------------------

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpointsL = []  # 2d points in image plane.
imgpointsR = []  # 2d points in image plane.

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

# Get images
images_left = sorted(glob.glob(f"{IMG_DIR}/left_*.jpg"))
images_right = sorted(glob.glob(f"{IMG_DIR}/right_*.jpg"))

print(f"Found {len(images_left)} pairs. Processing...")

for imgL_path, imgR_path in zip(images_left, images_right):
    imgL = cv2.imread(imgL_path)
    imgR = cv2.imread(imgR_path)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    retL, cornersL = cv2.findChessboardCorners(grayL, CHECKERBOARD_SIZE, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, CHECKERBOARD_SIZE, None)

    # If found, add object points, image points
    if retL and retR:
        objpoints.append(objp)

        # Refine corners for higher accuracy
        cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)
        print(f"Processed {os.path.basename(imgL_path)}")

print("Calibrating cameras... (This may take a minute)")

# 1. Calibrate each camera individually
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

# 2. Stereo Calibrate (Find relation between cameras)
flags = cv2.CALIB_FIX_INTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

retS, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    mtxL, distL, mtxR, distR,
    grayL.shape[::-1], criteria=criteria, flags=flags)

print(f"Stereo Calibration RMS Error: {retS}")
print("Saving parameters to 'calibration_data.npz'...")

np.savez("calibration_data.npz",
         mtxL=mtxL, distL=distL,
         mtxR=mtxR, distR=distR,
         R=R, T=T)

print("Done!")