import pyrealsense2 as rs
import numpy as np
import cv2

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()    
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start pipeline
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

# Wait for frames and align them
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
color_frame = aligned_frames.get_color_frame()
depth_frame = aligned_frames.get_depth_frame()
if not color_frame or not depth_frame:
    print("No valid frames received")
    pipeline.stop()
    exit()
color_image = np.asanyarray(color_frame.get_data())
cv2.imwrite('captured_color.jpg', color_image)

# Get intrinsics
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

# Function to handle mouse clicks
def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_value = depth_frame.get_distance(x, y)
        if depth_value > 0:
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_value)
            print(f"Clicked at (X,Y) pixel: ({x}, {y})")
            print(f"3D world coordinates (X,Y,Z in meters): {point_3d}")
        else:
            print("No valid depth at this pixel.")

# Show the image and set mouse callback
cv2.namedWindow('Captured Image')
cv2.setMouseCallback('Captured Image', mouse_click)
cv2.imshow('Captured Image', color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

pipeline.stop()
