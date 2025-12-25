import cv2
import mediapipe as mp
import numpy as np
from scipy.spatial import distance as dist
import serial
import time
import math

# Eye Aspect Ratio (EAR) calculation
def eye_aspect_ratio(eye):
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    C = dist.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

# Calculate roll geometrically from eye positions
def calculate_roll_from_eyes(landmarks, w, h):
    """
    Calculate head roll (tilt) by measuring the angle between the eyes.
    This is MORE RELIABLE than Euler angle decomposition!
    """
    # Get left and right eye centers
    left_eye_points = [33, 133, 160, 159, 158, 157, 173]  # Left eye landmarks
    right_eye_points = [362, 263, 387, 386, 385, 384, 398]  # Right eye landmarks
    
    # Calculate center of each eye
    left_eye_x = np.mean([landmarks[i].x * w for i in left_eye_points])
    left_eye_y = np.mean([landmarks[i].y * h for i in left_eye_points])
    
    right_eye_x = np.mean([landmarks[i].x * w for i in right_eye_points])
    right_eye_y = np.mean([landmarks[i].y * h for i in right_eye_points])
    
    # Calculate angle between eyes
    delta_x = right_eye_x - left_eye_x
    delta_y = right_eye_y - left_eye_y
    
    # Calculate roll angle in degrees
    # Positive = head tilted left (left eye higher)
    # Negative = head tilted right (right eye higher)
    roll = math.degrees(math.atan2(delta_y, delta_x))
    
    # Normalize: when head is straight, angle should be ~0
    # The eyes form a horizontal line when upright
    roll = roll  # atan2 already gives us the angle we want
    
    return roll

# Head pose estimation (for pitch and yaw only)
def get_head_pose(landmarks, frame_shape):
    h, w = frame_shape[:2]
    
    # 3D model points
    model_points = np.array([
        (0.0, 0.0, 0.0),             # Nose tip
        (0.0, -330.0, -65.0),        # Chin
        (-225.0, 170.0, -135.0),     # Left eye
        (225.0, 170.0, -135.0),      # Right eye
        (-150.0, -150.0, -125.0),    # Left mouth
        (150.0, -150.0, -125.0)      # Right mouth
    ])
    
    # 2D image points from landmarks
    image_points = np.array([
        (landmarks[1].x * w, landmarks[1].y * h),      # Nose tip
        (landmarks[152].x * w, landmarks[152].y * h),  # Chin
        (landmarks[263].x * w, landmarks[263].y * h),  # Left eye
        (landmarks[33].x * w, landmarks[33].y * h),    # Right eye
        (landmarks[287].x * w, landmarks[287].y * h),  # Left mouth
        (landmarks[57].x * w, landmarks[57].y * h)     # Right mouth
    ], dtype="double")
    
    # Camera matrix
    focal_length = w
    center = (w / 2, h / 2)
    camera_matrix = np.array([
        [focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]
    ], dtype="double")
    
    dist_coeffs = np.zeros((4, 1))
    
    # Solve PnP
    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    # Convert rotation vector to euler angles
    rotation_mat, _ = cv2.Rodrigues(rotation_vector)
    pose_mat = cv2.hconcat((rotation_mat, translation_vector))
    _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
    
    pitch, yaw = euler_angles[0, 0], euler_angles[1, 0]
    # DON'T use roll from Euler angles - it's unreliable!
    
    return pitch, yaw

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Eye landmark indices (MediaPipe Face Mesh)
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

# ============== IMPROVED THRESHOLDS ==============
EAR_THRESHOLD = 0.21
EAR_BLINK_THRESHOLD = 0.18
DROWSY_FRAMES = 48

YAW_THRESHOLD = 35
PITCH_THRESHOLD = 30
ROLL_THRESHOLD = 30 # Geometric roll is more accurate - can use tighter threshold
DISTRACTION_FRAMES = 90

NO_FACE_FRAMES = 45
RECOVERY_FRAMES = 45

# ============== STATE TRACKING ==============
frame_counter = 0
head_pose_counter = 0
no_face_counter = 0
recovery_counter = 0
alert_active = False
current_alert_type = 0

blink_counter = 0
total_blinks = 0
last_blink_time = time.time()

ear_buffer = []
EAR_BUFFER_SIZE = 5

roll_buffer = []
ROLL_BUFFER_SIZE = 8  

# Serial communication setup
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    time.sleep(2)
    print("Serial connected to STM32")
except:
    ser = None
    print("Serial connection failed - running without STM32")

def send_to_stm32(status_code):
    if ser:
        ser.write(f"{status_code}".encode())

def smooth_ear(ear_value):
    ear_buffer.append(ear_value)
    if len(ear_buffer) > EAR_BUFFER_SIZE:
        ear_buffer.pop(0)
    return np.mean(ear_buffer)

def smooth_roll(roll_value):
    """Simple smoothing for geometric roll - no wrapping issues!"""
    roll_buffer.append(roll_value)
    if len(roll_buffer) > ROLL_BUFFER_SIZE:
        roll_buffer.pop(0)
    
    if len(roll_buffer) == 0:
        return 0
    
    # Simple median - geometric roll doesn't have wrapping issues
    return np.median(roll_buffer)

cap = cv2.VideoCapture(0)

print("=== Driver Attention Monitor Started ===")
print("Using GEOMETRIC roll calculation (more stable!)")
print(f"- Drowsiness alert after {DROWSY_FRAMES} frames (~{DROWSY_FRAMES/30:.1f}s)")
print(f"- Distraction alert after {DISTRACTION_FRAMES} frames (~{DISTRACTION_FRAMES/30:.1f}s)")
print(f"- Head turn tolerance: Yaw ±{YAW_THRESHOLD}°, Pitch ±{PITCH_THRESHOLD}°, Roll ±{ROLL_THRESHOLD}°")
print("Press 'q' to quit\n")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)
    
    if results.multi_face_landmarks:
        no_face_counter = 0
        
        landmarks = results.multi_face_landmarks[0].landmark
        h, w = frame.shape[:2]
        
        # Get pitch and yaw from solvePnP (reliable)
        pitch, yaw = get_head_pose(landmarks, frame.shape)
        
        # Get roll from geometric calculation (NEW - more stable!)
        roll_raw = calculate_roll_from_eyes(landmarks, w, h)
        roll = smooth_roll(roll_raw)
        
        # Get eye coordinates
        left_eye = [(int(landmarks[i].x * w), int(landmarks[i].y * h)) for i in LEFT_EYE]
        right_eye = [(int(landmarks[i].x * w), int(landmarks[i].y * h)) for i in RIGHT_EYE]
        
        # Calculate EAR with smoothing
        left_ear = eye_aspect_ratio(left_eye)
        right_ear = eye_aspect_ratio(right_eye)
        ear = (left_ear + right_ear) / 2.0
        smoothed_ear = smooth_ear(ear)
        
        # Draw eyes
        for point in left_eye + right_eye:
            cv2.circle(frame, point, 2, (0, 255, 0), -1)
        
        # ============== BLINK DETECTION ==============
        if smoothed_ear < EAR_BLINK_THRESHOLD:
            blink_counter += 1
        else:
            if 1 <= blink_counter <= 6:
                total_blinks += 1
                last_blink_time = time.time()
            blink_counter = 0
        
        # ============== DROWSINESS DETECTION ==============
        if smoothed_ear < EAR_THRESHOLD and blink_counter > 6:
            frame_counter += 1
            # Cap the counter so it doesn't grow infinitely
            if frame_counter > DROWSY_FRAMES:
                frame_counter = DROWSY_FRAMES
        else:
            frame_counter = max(0, frame_counter - 2)
        
        # ============== DISTRACTION DETECTION ==============
        looking_away = abs(yaw) > YAW_THRESHOLD or abs(pitch) > PITCH_THRESHOLD or abs(roll) > ROLL_THRESHOLD
        
        if looking_away:
            head_pose_counter += 1
            # Cap the counter so it doesn't grow infinitely
            if head_pose_counter > DISTRACTION_FRAMES:
                head_pose_counter = DISTRACTION_FRAMES
        else:
            head_pose_counter = max(0, head_pose_counter - 2)
        
        # ============== ALERT LOGIC ==============
        new_alert = False
        new_alert_type = 0
        
        if frame_counter >= DROWSY_FRAMES:
            new_alert = True
            new_alert_type = 1
            cv2.putText(frame, "ALERT! DROWSINESS DETECTED!", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.rectangle(frame, (0, 0), (w, h), (0, 0, 255), 8)
        elif head_pose_counter >= DISTRACTION_FRAMES:
            new_alert = True
            new_alert_type = 2
            cv2.putText(frame, "ALERT! EYES OFF ROAD!", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 100, 255), 2)
            cv2.rectangle(frame, (0, 0), (w, h), (0, 100, 255), 8)
        else:
            if alert_active:
                recovery_counter += 1
                if recovery_counter >= RECOVERY_FRAMES:
                    alert_active = False
                    recovery_counter = 0
                    cv2.putText(frame, "Alert Cleared", (10, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, f"Recovering... {recovery_counter}/{RECOVERY_FRAMES}", (10, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                cv2.putText(frame, "Driver Attentive", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if new_alert:
            alert_active = True
            current_alert_type = new_alert_type
            recovery_counter = 0
        
        if alert_active:
            send_to_stm32(current_alert_type)
        else:
            send_to_stm32(0)
        
        # ============== DISPLAY METRICS ==============
        cv2.putText(frame, f"EAR: {smoothed_ear:.2f} | Blinks: {total_blinks}", (10, 80),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Yaw: {yaw:.1f}° | Pitch: {pitch:.1f}° | Roll: {roll:.1f}°", (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Drowsy: {frame_counter}/{DROWSY_FRAMES} | Away: {head_pose_counter}/{DISTRACTION_FRAMES}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
    else:
        no_face_counter += 1
        
        if no_face_counter >= NO_FACE_FRAMES:
            cv2.putText(frame, "ALERT! NO DRIVER DETECTED!", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (255, 0, 0), 8)
            send_to_stm32(3)
        else:
            cv2.putText(frame, f"Searching for face... {no_face_counter}/{NO_FACE_FRAMES}", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        frame_counter = 0
        head_pose_counter = 0
    
    cv2.imshow('Driver Attention Monitor', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()