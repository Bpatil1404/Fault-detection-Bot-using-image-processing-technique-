import cv2
import urllib.request
import numpy as np
import os
from datetime import datetime
import time
import threading

url = 'http://192.168.84.83/cam-hi.jpg'  # URL for webcam feed

# File to log detected faults
fault_log_file = "Fault_Log.csv"
if os.path.exists(fault_log_file):
    print("Fault_Log.csv exists..")
    os.remove(fault_log_file)

# Initialize fault log
with open(fault_log_file, 'w') as f:
    f.writelines("Time, Fault_Description\n")

# Function to log faults
def log_fault(fault_description):
    now = datetime.now()
    dt_string = now.strftime('%Y-%m-%d %H:%M:%S')
    with open(fault_log_file, 'a') as f:
        f.writelines(f'{dt_string}, {fault_description}\n')

# Function to capture image from URL
def capture_image(url):
    try:
        img_resp = urllib.request.urlopen(url, timeout=10)
        imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        frame = cv2.imdecode(imgnp, -1)
        return frame
    except urllib.error.URLError as e:
        print(f"Failed to capture image: {e.reason}")
        return None
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
        return None

# Load known fault images and compute their descriptors
orb = cv2.ORB_create(nfeatures=1000)  # Adjust the number of keypoints
known_faults = []

# Set the path to your known fault images directory
known_fault_dir = r"C:\Users\Gurupratap\AppData\Local\Programs\Python\Python312\fault_images"

# Check if the directory exists
if not os.path.exists(known_fault_dir):
    raise FileNotFoundError(f"The directory {known_fault_dir} does not exist. Please check the path.")

for filename in os.listdir(known_fault_dir):
    if filename.endswith(".jpg") or filename.endswith(".png"):
        fault_img = cv2.imread(os.path.join(known_fault_dir, filename), cv2.IMREAD_GRAYSCALE)
        keypoints, descriptors = orb.detectAndCompute(fault_img, None)
        if descriptors is not None:  # Ensure descriptors are not None
            known_faults.append((os.path.splitext(filename)[0], fault_img, keypoints, descriptors))

# Function to detect and match faults in the frame
def detect_fault(frame, known_faults):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints_frame, descriptors_frame = orb.detectAndCompute(gray_frame, None)
    if descriptors_frame is None:
        return []

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    detected_faults = []

    for fault_name, fault_img, keypoints_fault, descriptors_fault in known_faults:
        matches = bf.match(descriptors_frame, descriptors_fault)
        matches = sorted(matches, key=lambda x: x.distance)

        good_matches = [m for m in matches if m.distance < 50]  # Example threshold for good matches

        if len(good_matches) > 10:  # Example threshold
            src_pts = np.float32([keypoints_frame[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([keypoints_fault[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Apply RANSAC to filter matches
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()

            # Consider a match only if the number of inliers is significant
            if sum(matchesMask) > 10:
                detected_faults.append(fault_name)

    return detected_faults

# Function to handle fault detection in a separate thread
def fault_detection_thread():
    global frame, frame_lock, detected_faults
    while True:
        with frame_lock:
            if frame is not None:
                # Detect faults in the frame
                detected_faults = detect_fault(frame, known_faults)

                # Log detected faults
                for fault in detected_faults:
                    fault_description = f'Fault detected: {fault}'
                    log_fault(fault_description)
                    print(fault_description)

        time.sleep(1)  # Wait for 1 second before the next detection

# Initialize frame, detected_faults and lock
frame = None
detected_faults = []
frame_lock = threading.Lock()

# Start fault detection thread
detection_thread = threading.Thread(target=fault_detection_thread)
detection_thread.start()

while True:
    new_frame = capture_image(url)

    with frame_lock:
        frame = new_frame

    if frame is not None:
        with frame_lock:
            for fault in detected_faults:
                cv2.putText(frame, fault, (10, 30 + 30 * detected_faults.index(fault)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        cv2.imshow('Live Fault Detection', frame)

    key = cv2.waitKey(5)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
detection_thread.join()
