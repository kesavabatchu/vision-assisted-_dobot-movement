# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time
from dobot_api import DobotApiDashboard, DobotApiMove

class RobotVisionSystem:
    def __init__(self):
        # Robot initial position [x, y, z, rx, ry, rz]
        self.initial_pos = [-516.816, 69.4502, 435.7052, -177.9571, 0.0, 96.0]
        
        # Camera-to-robot offsets (mm)
        self.camera_offset_x = -50
        self.camera_offset_y = 0
        self.camera_offset_z = 230
        
        # Movement parameters (mm)
        self.hover_height = 100
        self.pick_height = 50
        
        # Robot connection settings
        self.ip = "192.168.1.6"
        self.dashboard_port = 29999
        self.move_port = 30003
        
        # Initialize components
        self.connect_robot()
        self.initialize_camera()
        self.model = YOLO('yolov8n.pt')  # Ensure yolov8n.pt is in the working directory

    def connect_robot(self):
        """Connect to Dobot Nova5 and move to initial position."""
        try:
            self.client_dash = DobotApiDashboard(self.ip, self.dashboard_port)
            self.client_move = DobotApiMove(self.ip, self.move_port)
            
            print("Enabling robot...")
            self.client_dash.EnableRobot()
            time.sleep(1)
            self.client_dash.SpeedFactor(5)  # Slightly higher speed for testing
            self.client_dash.AccJ(5)
            self.client_dash.AccL(5)
            
            print("Moving to initial position...")
            self.client_move.MovL(*self.initial_pos)
            time.sleep(5)  # Wait for movement completion
            print("Initial position reached.")
            
        except Exception as e:
            print(f"Robot connection failed: {e}")
            raise

    def initialize_camera(self):
        """Initialize RealSense camera with robust setup."""
        try:
            # Reset all connected RealSense devices
            ctx = rs.context()
            devices = ctx.query_devices()
            if not devices:
                raise RuntimeError("No RealSense devices detected.")
            for dev in devices:
                print(f"Resetting device: {dev.get_info(rs.camera_info.name)}")
                dev.hardware_reset()
            time.sleep(3)  # Wait for devices to stabilize
            
            # Configure pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # Start pipeline
            print("Starting camera pipeline...")
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            
            # Verify stream is working
            for _ in range(10):  # Try for 10 frames
                frames = self.pipeline.wait_for_frames(5000)
                if frames:
                    print("Camera initialized successfully.")
                    return
            raise RuntimeError("Camera started but no frames received.")
            
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            raise

    def get_frames(self):
        """Get aligned frames with retry logic."""
        for attempt in range(5):  # Increased retries
            try:
                frames = self.pipeline.wait_for_frames(5000)  # 5s timeout
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if depth_frame and color_frame:
                    return aligned_frames
                print(f"Attempt {attempt + 1}: Missing depth or color frame.")
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
            time.sleep(1)  # Brief pause between retries
        raise RuntimeError("Failed to get frames after 5 attempts.")

    def detect_object(self):
        """Detect object with robust frame handling."""
        try:
            aligned_frames = self.get_frames()
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            # Convert to image for YOLO
            color_image = np.asanyarray(color_frame.get_data())
            results = self.model(color_image)
            boxes = results[0].boxes
            
            if len(boxes) == 0:
                print("No objects detected.")
                return None
            
            # Select highest-confidence detection
            best_box = max(boxes, key=lambda x: x.conf)
            x_min, y_min, x_max, y_max = best_box.xyxy[0].int().tolist()
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)
            depth = depth_frame.get_distance(center_x, center_y)
            
            if depth <= 0 or depth > 2.0:  # Filter invalid depths (e.g., >2m)
                print(f"Invalid depth: {depth}m")
                return None
            
            # Convert to 3D coordinates (meters)
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            X = (center_x - intr.ppx) / intr.fx * depth
            Y = (center_y - intr.ppy) / intr.fy * depth
            Z = depth
            
            print(f"Detected object at: X={X*1000:.1f}mm, Y={Y*1000:.1f}mm, Z={Z*1000:.1f}mm")
            return [X, Y, Z], color_image, (x_min, y_min, x_max, y_max)
        
        except Exception as e:
            print(f"Detection error: {e}")
            return None

    def execute_movement(self, target_pos, movement_type="linear"):
        """Execute movement with verification."""
        try:
            print(f"Moving to: X={target_pos[0]:.1f}, Y={target_pos[1]:.1f}, Z={target_pos[2]:.1f}")
            if movement_type == "linear":
                self.client_move.MovL(*target_pos)
            else:
                self.client_move.MovJ(*target_pos)
            time.sleep(2)  # Adjusted delay for stability
            return True
        except Exception as e:
            print(f"Movement failed: {e}")
            return False

    def pick_and_return(self, point):
        """Execute pick-and-return sequence."""
        if not point:
            print("No valid point for picking.")
            return
        
        X_mm, Y_mm, Z_mm = [p * 1000 for p in point]  # Convert to mm
        
        # Calculate positions
        hover_pos = [
            self.initial_pos[0] + Y_mm + self.camera_offset_x,
            self.initial_pos[1] - X_mm + self.camera_offset_y,
            self.initial_pos[2] - Z_mm + self.camera_offset_z + self.hover_height,
            *self.initial_pos[3:]
        ]
        pick_pos = hover_pos.copy()
        pick_pos[2] -= (self.hover_height - self.pick_height)
        
        # Execute sequence
        print("\n--- Starting Pick Sequence ---")
        if not self.execute_movement(hover_pos):
            return
        if not self.execute_movement(pick_pos):
            return
        print("Simulating pick operation...")
        time.sleep(1)  # Replace with gripper control
        if not self.execute_movement(hover_pos):
            return
        if not self.execute_movement(self.initial_pos):
            return
        print("--- Sequence Complete ---")

    def run(self):
        """Main execution loop."""
        try:
            print("Starting main loop. Press 'q' to quit.")
            while True:
                result = self.detect_object()
                if result:
                    point, color_image, bbox = result
                    x_min, y_min, x_max, y_max = bbox
                    cv2.rectangle(color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.imshow("Detection", color_image)
                    self.pick_and_return(point)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Quit signal received.")
                    break
                time.sleep(0.1)  # Reduce CPU usage
                
        except KeyboardInterrupt:
            print("Interrupted by user.")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Release resources."""
        print("Cleaning up...")
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
        if hasattr(self, 'client_dash'):
            self.client_dash.DisableRobot()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == "__main__":
    try:
        system = RobotVisionSystem()
        system.run()
    except Exception as e:
        print(f"Startup failed: {e}")