import cv2
import mediapipe as mp
import serial
import time
import math

class HandTracker:
    def __init__(self, serial_port='COM8', baud_rate=9600):
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize serial communication
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)  # Reduced timeout
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to {serial_port}")
        except:
            print(f"Could not connect to {serial_port}")
            self.ser = None
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Reduced resolution for faster processing
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 60)  # Increase FPS
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to minimize delay
        
        # Calibration variables
        self.calibration_data = {
            'thumb1': {'min': 90, 'max': 0},
            'thumb2': {'min': 180, 'max': 0},
            'index': {'min': 180, 'max': 0},
            'middle': {'min': 180, 'max': 0},
            'ring': {'min': 180, 'max': 0},
            'pinky': {'min': 180, 'max': 0}
        }
        self.is_calibrated = False
        
        # Performance optimization variables
        self.frame_skip_count = 0
        self.frame_skip_threshold = 3  # Process every 3rd frame
        self.last_sent_angles = None
        self.angle_threshold = 3  # Only send if angle changed by more than 3 degrees
        self.last_send_time = 0
        self.min_send_interval = 0.03  # Minimum 30ms between sends (33 FPS max)
        
    def calculate_finger_angle(self, landmarks, finger_joints):
        """Calculate angle for a finger based on joint positions"""
        # Get the three points for angle calculation
        p1_x, p1_y = landmarks[finger_joints[0]].x, landmarks[finger_joints[0]].y
        p2_x, p2_y = landmarks[finger_joints[1]].x, landmarks[finger_joints[1]].y
        p3_x, p3_y = landmarks[finger_joints[2]].x, landmarks[finger_joints[2]].y
        
        # Calculate coordinates
        v1_x, v1_y = p1_x - p2_x, p1_y - p2_y
        v2_x, v2_y = p3_x - p2_x, p3_y - p2_y
        
        # Calculate angle using dot product
        dot_product = v1_x * v2_x + v1_y * v2_y
        magnitude_v1 = math.sqrt(v1_x**2 + v1_y**2)
        magnitude_v2 = math.sqrt(v2_x**2 + v2_y**2)
        
        if magnitude_v1 == 0 or magnitude_v2 == 0:
            return 90
        
        cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
        cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp between -1 and 1
        angle = math.degrees(math.acos(cos_angle))
        
        return int(angle)
    
    def get_servo_angles(self, landmarks):
        """Calculate servo angles for all fingers"""
        # Define finger joint indices
        finger_joints = {
            'thumb1': [1, 2, 3],     # Thumb servo 1 (0-90 degrees)
            'thumb2': [2, 3, 4],     # Thumb servo 2 (0-180 degrees) 
            'index': [5, 6, 7],      # Index finger joints
            'middle': [9, 10, 11],   # Middle finger joints
            'ring': [13, 14, 15],    # Ring finger joints
            'pinky': [17, 18, 19]    # Pinky joints
        }
        
        servo_angles = {}
        
        for finger, joints in finger_joints.items():
            try:
                angle = self.calculate_finger_angle(landmarks, joints)
                servo_angles[finger] = angle
            except:
                if finger == 'thumb1':
                    servo_angles[finger] = 0  # Default open position
                else:
                    servo_angles[finger] = 0  # Default open position
        
        return servo_angles
    
    def calibrate(self):
        """Calibration mode"""
        print("CALIBRATION MODE")
        
        start_time = time.time()
        calibration_duration = 5  # 10 seconds
        
        while time.time() - start_time < calibration_duration:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Show countdown
            remaining_time = int(calibration_duration - (time.time() - start_time))
            cv2.putText(frame, f"CALIBRATION: {remaining_time}s", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, "Move fingers entirely", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Get current angles
                    servo_angles = self.get_servo_angles(hand_landmarks.landmark)
                    
                    # Update calibration data
                    for finger, angle in servo_angles.items():
                        if angle < self.calibration_data[finger]['min']:
                            self.calibration_data[finger]['min'] = angle
                        if angle > self.calibration_data[finger]['max']:
                            self.calibration_data[finger]['max'] = angle
            
            cv2.imshow('Hand Tracking - Calibration', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.is_calibrated = True
        print("\nCALIBRATION COMPLETE")
        for finger, data in self.calibration_data.items():
            print(f"{finger.capitalize()}: {data['min']} - {data['max']}")
        print("Starting hand tracking")
        time.sleep(1)
        
        # Close calibration window
        cv2.destroyWindow('Hand Tracking - Calibration')
    
    def map_angle_to_servo(self, angle, finger):
        """Map calibrated angle to servo range (inverted for closed hand at max angle)"""
        if not self.is_calibrated:
            if finger == 'thumb1':
                return max(0, min(90, angle // 2))  # Map to 0-90 range
            else:
                return max(0, min(180, angle))
        
        min_angle = self.calibration_data[finger]['min']
        max_angle = self.calibration_data[finger]['max']
        
        if max_angle - min_angle == 0:
            return 0 if finger == 'thumb1' else 0
        
        if finger == 'thumb1':
            mapped = int((max_angle - angle) * 90 / (max_angle - min_angle))
            return max(0, min(90, mapped))
        else:
            mapped = int((max_angle - angle) * 180 / (max_angle - min_angle))
            return max(0, min(180, mapped))
    
    def angles_changed_significantly(self, new_angles):
        """Check if angles changed enough to warrant sending new data"""
        if self.last_sent_angles is None:
            return True
            
        for finger in new_angles:
            if abs(new_angles[finger] - self.last_sent_angles[finger]) > self.angle_threshold:
                return True
        return False
    
    def send_servo_data(self, servo_angles):
        """Send servo angles to ESP32 as binary data"""
        current_time = time.time()
        
        if (current_time - self.last_send_time < self.min_send_interval or 
            not self.angles_changed_significantly(servo_angles)):
            return
            
        if self.ser:
            try:
                # Send as binary data
                # Format: [255, thumb1, thumb2, index, middle, ring, pinky, 254]
                data = bytearray([255])  # Start marker
                data.append(servo_angles['thumb1'])
                data.append(servo_angles['thumb2'])
                data.append(servo_angles['index'])
                data.append(servo_angles['middle'])
                data.append(servo_angles['ring'])
                data.append(servo_angles['pinky'])
                data.append(254)  # End marker
                
                self.ser.write(data)
                
                # Resest data in buffer
                self.ser.reset_output_buffer()
                
                self.last_sent_angles = servo_angles.copy()
                self.last_send_time = current_time
                
                # Reduced console output frequency
                if int(current_time * 10) % 5 == 0:  # Print every 0.5 seconds
                    print(f"T1:{servo_angles['thumb1']} T2:{servo_angles['thumb2']} I:{servo_angles['index']} M:{servo_angles['middle']} R:{servo_angles['ring']} P:{servo_angles['pinky']}")
                
            except Exception as e:
                print(f"Error sending data: {e}")
    
    def run(self):
        """Main loop for hand tracking"""
        print("Hand tracking starting")
        
        self.calibrate()
        
        print("Hand tracking started.")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Frame skipping for better performance
            self.frame_skip_count += 1
            if self.frame_skip_count < self.frame_skip_threshold:
                continue
            self.frame_skip_count = 0
                
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Draw hand landmarks and get servo angles
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Calculate servo angles
                    raw_angles = self.get_servo_angles(hand_landmarks.landmark)
                    
                    # Map to servo range using calibration
                    servo_angles = {}
                    for finger, angle in raw_angles.items():
                        servo_angles[finger] = self.map_angle_to_servo(angle, finger)
                    
                    # Send data to ESP32
                    self.send_servo_data(servo_angles)
                    
                    # Simplified display
                    cv2.putText(frame, f"T1:{servo_angles['thumb1']} T2:{servo_angles['thumb2']}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, f"I:{servo_angles['index']} M:{servo_angles['middle']} R:{servo_angles['ring']} P:{servo_angles['pinky']}", (10, 50), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                # No hand detected
                cv2.putText(frame, "No hand detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Display frame
            cv2.imshow('Hand Tracking', frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        if self.ser:
            self.ser.close()

tracker = HandTracker(serial_port='COM8', baud_rate=9600)
tracker.run()