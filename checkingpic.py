import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import os
import time
import shutil

def get_detected_object_center():
    img_dir = 'images'
    predict_dir = os.path.join('runs', 'segment', 'predict')
    os.makedirs(img_dir, exist_ok=True)
    if os.path.exists(predict_dir):
        shutil.rmtree(predict_dir)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            print("No valid frames received")
            return None
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        timestamp = int(time.time())
        img_path = os.path.join(img_dir, f'captured_color_{timestamp}.jpg')
        cv2.imwrite(img_path, color_image)
        print(f"Saved new RGB image: {img_path}")

        model_path = os.path.join('runs', 'segment', 'train', 'weights', 'best.pt')
        model = YOLO(model_path)
        results = model(img_path, save=True)

        # Show the result image in a non-blocking way
        if os.path.exists(predict_dir):
            predicted_files = [f for f in os.listdir(predict_dir) if f.endswith('.jpg') or f.endswith('.png')]
            predicted_files.sort(key=lambda x: os.path.getmtime(os.path.join(predict_dir, x)), reverse=True)
            if predicted_files:
                result_img_path = os.path.join(predict_dir, predicted_files[0])
                result_img = cv2.imread(result_img_path)
                cv2.imshow("Detected Image", result_img)
                # Non-blocking: keep window open but do not wait
                cv2.waitKey(1)

        # Extract quadrilateral endpoints and get 3D center
        for result in results:
            if result.masks is not None: 
                for mask_coords in result.masks.xy:
                    four_points = mask_coords[:4] if len(mask_coords) >= 4 else mask_coords
                    center_x = int(np.mean(four_points[:, 0]))
                    center_y = int(np.mean(four_points[:, 1]))
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth_value = depth_frame.get_distance(center_x, center_y)
                    if depth_value > 0:
                        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_value)
                        pipeline.stop()
                        return point_3d
                    else:
                        print("No valid depth at center pixel!")
        pipeline.stop()
        return None
    except Exception as e:
        print("Error:", e)
        pipeline.stop()
        return None

if __name__ == "__main__":
    center_3d = get_detected_object_center()
    if center_3d is not None:
        print("3D Center Coordinates (x, y, z):", center_3d)
    else:
        print("No valid 3D center detected.")
    # Optional: Keep window open until user closes it
    # while True:
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    cv2.destroyAllWindows()
