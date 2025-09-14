import pyrealsense2 as rs
import numpy as np
import cv2

# Configure color stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start video streaming
pipeline.start(config)

try:
    print("Press 'q' in the video window to quit.")
    while True:
        # Wait for the next frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue 

        # Convert frame to numpy array for OpenCV
        color_image = np.asanyarray(color_frame.get_data())

        # Display the RGB video
        cv2.imshow('Depth Camera Live Video', color_image)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
