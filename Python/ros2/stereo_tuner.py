import cv2
import numpy as np

# Load Calibration
data = np.load('calibration_data.npz')
mtxL, distL = data['mtxL'], data['distL']
mtxR, distR = data['mtxR'], data['distR']
R, T = data['R'], data['T']

# Calculate Rectification transforms
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    mtxL, distL, mtxR, distR, (640,480), R, T
)

# Create lookup maps (run once)
left_map_x, left_map_y = cv2.initUndistortRectifyMap(
    mtxL, distL, R1, P1, (640,480), cv2.CV_16SC2)
right_map_x, right_map_y = cv2.initUndistortRectifyMap(
    mtxR, distR, R2, P2, (640,480), cv2.CV_16SC2)

# ---------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------
# Check your camera indices. usually 0 and 2, or 0 and 1.
LEFT_CAM_ID = 0
RIGHT_CAM_ID = 1
RESOLUTION = (640, 480)

# ---------------------------------------------------------------------
# INITIALIZATION
# ---------------------------------------------------------------------
print("Initializing cameras...")
cap_left = cv2.VideoCapture(LEFT_CAM_ID)
cap_right = cv2.VideoCapture(RIGHT_CAM_ID)

# Force resolution (Critical for performance on Pi)
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])

if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: Could not open one or both cameras.")
    exit()


def nothing(x):
    pass


# Create a window for tuning
cv2.namedWindow('Disp Tuning')
cv2.createTrackbar('Num Disparities', 'Disp Tuning', 1, 16, nothing)  # Multiples of 16
cv2.createTrackbar('Block Size', 'Disp Tuning', 5, 50, nothing)
cv2.createTrackbar('Uniqueness', 'Disp Tuning', 10, 20, nothing)

# Initialize the Stereo SGBM Matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16,
    blockSize=5,
    P1=8 * 3 * 5 ** 2,
    P2=32 * 3 * 5 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

print("Press 'q' to quit.")

while True:
    # -----------------------------------------------------------------
    # 1. SYNCHRONIZATION HACK
    # -----------------------------------------------------------------
    # We grab() both first to minimize the time delay between shutters
    if not (cap_left.grab() and cap_right.grab()):
        print("No more frames")
        break

    # Then we retrieve() to decode the actual image data
    _, frame_left = cap_left.retrieve()
    _, frame_right = cap_right.retrieve()


    # -----------------------------------------------------------------
    # 2. PRE-PROCESSING
    # -----------------------------------------------------------------
    # Before converting to grayscale
    frame_left = cv2.remap(frame_left, left_map_x, left_map_y, cv2.INTER_LINEAR)
    frame_right = cv2.remap(frame_right, right_map_x, right_map_y, cv2.INTER_LINEAR)
    # Convert to grayscale (Stereo matching works on luminance)
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    # -----------------------------------------------------------------
    # 3. UPDATE TUNING PARAMETERS
    # -----------------------------------------------------------------
    # Get slider values
    num_disp = cv2.getTrackbarPos('Num Disparities', 'Disp Tuning') * 16
    block_size = cv2.getTrackbarPos('Block Size', 'Disp Tuning')
    uniqueness = cv2.getTrackbarPos('Uniqueness', 'Disp Tuning')

    # Safety checks
    if num_disp < 16: num_disp = 16
    if block_size % 2 == 0: block_size += 1  # Block size must be odd
    if block_size < 5: block_size = 5

    # Update the matcher
    stereo.setNumDisparities(num_disp)
    stereo.setBlockSize(block_size)
    stereo.setUniquenessRatio(uniqueness)
    # P1/P2 are controlled by block size
    stereo.setP1(8 * 3 * block_size ** 2)
    stereo.setP2(32 * 3 * block_size ** 2)

    # -----------------------------------------------------------------
    # 4. COMPUTE DEPTH (DISPARITY MAP)
    # -----------------------------------------------------------------
    # The actual heavy lifting
    disparity = stereo.compute(gray_left, gray_right)

    # -----------------------------------------------------------------
    # 5. VISUALIZATION
    # -----------------------------------------------------------------
    # Normalize the disparity for display (it comes in as 16-bit int)
    # We map it to 0-255 range for grayscale visualization
    disp_vis = cv2.normalize(disparity, None, alpha=0, beta=255,
                             norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Optional: Apply a color map (JET) to make it look cool (Blue=Far, Red=Close)
    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    # Show the results
    # Stack images horizontally: Left Input | Depth Map
    combined_view = np.hstack((frame_left, disp_color))

    cv2.imshow('Stereo Vision', combined_view)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()