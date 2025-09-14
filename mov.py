# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove

class RobotVisionSystem:
    def __init__(self):
        # Robot initial position
        self.initial_pos = {
            'x': 20.0000,
            'y': -593.0000,
            'z': 214.0000,
            'rx': 179.0000,
            'ry': -0.0000,
            'rz': 176.0000
        }
        
        # Robot connection parameters
        self.ip = "192.168.1.6"
        self.dashboard_port = 29999
        self.move_port = 30003
        self.feed_port = 30004
        
        # RealSense camera parameters
        self.pipeline = None
        self.align = None
        
        # YOLO model
        self.model = YOLO('yolov8n.pt')  # Using YOLOv8 nano model
        
        # Detection parameters
        self.hover_height = 100  # Height to hover above object in mm
        self.detection_running = False
        
        # Initialize robot connection
        self.connect_robot()

    def connect_robot(self):
        """Establish connection to the robot"""
        try:
            self.client_dash = DobotApiDashboard(self.ip, self.dashboard_port)
            self.client_move = DobotApiMove(self.ip, self.move_port)
            self.client_feed = DobotApi(self.ip, self.feed_port)
            
            # Enable robot
            self.client_dash.EnableRobot()
            time.sleep(0.5)
            self.client_dash.SpeedFactor(10)  # Set speed to 10% for much slower movement
            
            print("Robot connected successfully")
        except Exception as e:
            print(f"Robot connection failed: {e}")
            raise

    def initialize_camera(self):
        """Initialize RealSense camera"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable depth and color streams
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start streaming
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            # Get depth scale
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            print("Camera initialized successfully")
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            raise

    def detect_object(self):
        """Detect object using YOLO and return 3D coordinates"""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            # Get depth and color frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Perform YOLO detection
            results = self.model(color_image)
            boxes = results[0].boxes
            
            if len(boxes) == 0:
                return None
            
            # Take the highest confidence detection
            best_box = max(boxes, key=lambda x: x.conf)
            x_min, y_min, x_max, y_max = best_box.xyxy[0].int().tolist()
            
            # Calculate center
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)
            
            # Get depth at center
            depth = depth_frame.get_distance(center_x, center_y)
            if depth <= 0:
                return None
            
            # Get intrinsics and convert to 3D coordinates (in meters)
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            X = (center_x - intr.ppx) / intr.fx * depth
            Y = (center_y - intr.ppy) / intr.fy * depth
            Z = depth
            
            return [X, Y, Z], color_image, (x_min, y_min, x_max, y_max)
            
        except Exception as e:
            print(f"Detection error: {e}")
            return None

    def move_to_object(self, point):
        """Move robot to hover over detected object"""
        if point is None:
            print("No object detected")
            return
        
        try:
            # Convert meters to mm and adjust coordinates
            target_x = self.initial_pos['x'] + (point[0] * 1000)  # Convert m to mm
            target_y = self.initial_pos['y'] + (point[1] * 1000)
            target_z = self.initial_pos['z'] - (point[2] * 1000) + self.hover_height
            
            # Apply correction for diagonal movement
            correction_x = 50  # Move 50 mm to the right (adjust based on testing)
            correction_y = -30  # Move 30 mm forward (adjust based on testing)
            target_x += correction_x
            target_y += correction_y
            
            # Move robot
            self.client_move.MovL(
                target_x,
                target_y,
                target_z,
                self.initial_pos['rx'],
                self.initial_pos['ry'],
                self.initial_pos['rz']
            )
            print(f"Moving to: X:{target_x:.2f}, Y:{target_y:.2f}, Z:{target_z:.2f}")
            
        except Exception as e:
            print(f"Move error: {e}")

    def run(self):
        """Main execution loop"""
        self.initialize_camera()
        self.detection_running = True
        
        try:
            # Move to initial position first
            print("Moving to initial position...")
            self.client_move.MovL(
                self.initial_pos['x'],
                self.initial_pos['y'],
                self.initial_pos['z'],
                self.initial_pos['rx'],
                self.initial_pos['ry'],
                self.initial_pos['rz']
            )
            time.sleep(2)  # Wait 2 seconds to ensure the robot reaches the position
            print("Initial position reached. Starting object detection and robot control...")
            
            while self.detection_running:
                # Detect object
                result = self.detect_object()
                if result:
                    point, color_image, bbox = result
                    print(f"Object detected at (camera coords): {point}")
                    self.move_to_object(point)
                    
                    # Optional: Show detection
                    x_min, y_min, x_max, y_max = bbox
                    cv2.rectangle(color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.imshow("Detection", color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                time.sleep(0.5)  # Control loop frequency to avoid overwhelming the robot
                
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.detection_running = False
        if self.pipeline:
            self.pipeline.stop()
        if self.client_dash:
            self.client_dash.DisableRobot()
            self.client_dash.close()
        if self.client_move:
            self.client_move.close()
        if self.client_feed:
            self.client_feed.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Required dependencies
    try:
        system = RobotVisionSystem()
        system.run()
    except Exception as e:
        print(f"Error: {e}")