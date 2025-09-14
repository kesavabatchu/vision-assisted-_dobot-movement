# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time
from dobot_api import DobotApiDashboard, DobotApiMove

class RobotVisionSystem:
    def __init__(self):
        # Robot initial position [x, y, z, rx, ry, rz] (in mm)
        # This position is treated as the origin for displacement calculation.
        self.initial_pos = [-427.65, -180.59, 480.69, 83.77, -0.39, -84.01]
        # Camera-to-robot offsets (mm)
        # Only a Y-axis offset of 145 mm is kept.
        self.camera_offset_x = 0      # Removed X offset
        self.camera_offset_y = 0    # 145mm lateral offset on the Y axis
        self.camera_offset_z = 0      # Removed vertical offset
        
        # Movement parameters (mm)
        self.hover_height = 0 # Additional height above target for safety
        self.pick_height = 0    # Height to pick the object
        
        # Robot connection settings
        self.ip = "192.168.1.6"
        self.dashboard_port = 29999
        self.move_port = 30003
        
        # Initialize components
        self.connect_robot()
        self.initialize_camera()
        self.model = YOLO('yolov8n.pt')  # Ensure yolov8n.pt is in the working directory
        

    # Gripper control functions
    def gripper_open(self):
        self.client_dash.ToolDO(1, 1)
        self.client_dash.ToolDO(2, 0)
        print("Gripper opening...")
        time.sleep(1)

    def gripper_close(self):
        self.client_dash.ToolDO(1, 0)
        self.client_dash.ToolDO(2, 1)
        print("Gripper closing...")
        time.sleep(1.5)

    def gripper_stop(self):
        self.client_dash.ToolDO(1, 0)
        self.client_dash.ToolDO(2, 0)
        print("Gripper stopped")


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
            self.gripper_open()
            
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
            for _ in range(10):
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
        for attempt in range(5):
            try:
                frames = self.pipeline.wait_for_frames(5000)
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if depth_frame and color_frame:
                    return aligned_frames
                print(f"Attempt {attempt + 1}: Missing depth or color frame.")
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
            time.sleep(1)
        raise RuntimeError("Failed to get frames after 5 attempts.")

    def detect_object(self):
        """Detect object with robust frame handling and ignore persons."""
        try:
            aligned_frames = self.get_frames()
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            # Convert to image for YOLO detection
            color_image = np.asanyarray(color_frame.get_data())
            results = self.model(color_image)
            boxes = results[0].boxes
            if len(boxes) == 0:
                print("No objects detected.")
                return None
            
            # Filter out persons (assuming class ID 0 is person)
            non_person_boxes = []
            for i, box in enumerate(boxes):
                cls_id = int(results[0].boxes.cls[i])
                if cls_id != 0:
                    non_person_boxes.append(box)
            if not non_person_boxes:
                print("Only persons detected; ignoring.")
                return None
            
            # Select the highest-confidence non-person detection
            best_box = max(non_person_boxes, key=lambda b: b.conf)
            x_min, y_min, x_max, y_max = best_box.xyxy[0].int().tolist()
            center_x = int((x_min + x_max) / 2)
            center_y = int((y_min + y_max) / 2)
            
            # Retrieve depth at the center pixel
            depth = depth_frame.get_distance(center_x, center_y)
            if depth <= 0 or depth > 2.0:
                print(f"Invalid depth: {depth:.2f}m")
                return None
            
            # Convert pixel to 3D coordinates (in meters) using camera intrinsics
            intr = depth_frame.profile.as_video_stream_profile().intrinsics
            X_cam = (center_x - intr.ppx) / intr.fx * depth  # original camera X (rightward)
            Y_cam = (center_y - intr.ppy) / intr.fy * depth  # original camera Y (downward)
            Z_cam = depth  # original camera Z (forward)
            
            print(f"Detected object (camera frame): X={X_cam*1000:.1f}mm, Y={Y_cam*1000:.1f}mm, Z={Z_cam*1000:.1f}mm")
            return [X_cam, Y_cam, Z_cam], color_image, (x_min, y_min, x_max, y_max)
        
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
            time.sleep(2)
            return True
        except Exception as e:
            print(f"Movement failed: {e}")
            return False

    def pick_and_return(self, point):
        """
        Execute the sequence:
          1. Move above the target (hover)
          2. Lower to pick the object
          3. Simulate a pick (gripper control could be added)
          4. Return to hover, then to the initial position.
        
        With the initial position as the origin, we map:
          - Robot forward (X_robot) = initial_x - (detected Z_cam * 1000)
          - Robot lateral (Y_robot)  = initial_y + (detected X_cam * 1000) + camera_offset_y
          - Robot vertical (Z_robot) = initial_z - (detected Y_cam * 1000)
        """
        if not point:
            print("No valid point for picking.")
            return
        
        # Unpack detected camera coordinates (in meters)
        X_cam, Y_cam, Z_cam = point
        
        # Compute displacements (in mm)
        forward_disp = Z_cam * 1000       # forward displacement (robot X)
        lateral_disp = X_cam * 1000       # lateral displacement (robot Y)
        vertical_disp = Y_cam * 1000      # vertical displacement (Y_cam is downward)
        
        # Compute target robot coordinates relative to initial_pos (origin)
        target_x = self.initial_pos[0] - forward_disp
        target_y = self.initial_pos[1] + lateral_disp + self.camera_offset_y
        base_z = self.initial_pos[2] + vertical_disp
        
        target_x = target_x + 220
        target_y = target_y - 20
        # Define hover and pick heights
        target_z_hover = base_z + self.hover_height
        target_z_pick = base_z + self.pick_height
        
        hover_pos = [target_x, target_y, target_z_hover, *self.initial_pos[3:]]
        pick_pos = [target_x, target_y, target_z_pick, *self.initial_pos[3:]]
        
        print("\n--- Starting Pick Sequence ---")
        if not self.execute_movement(hover_pos):
            return
        if not self.execute_movement(pick_pos):
            return
        print("Picking object...")
        self.gripper_open()
        time.sleep(0.5)
        self.gripper_close()
        self.gripper_stop()
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
                time.sleep(0.1)
                
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
