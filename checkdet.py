import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO


def adaptive_epsilon_approximation(contour, method='percentage'):
    """
    Calculate adaptive epsilon for contour approximation
    
    Args:
        contour: Input contour
        method: 'percentage', 'adaptive', or 'iterative'
    
    Returns:
        Optimal epsilon value
    """
    peri = cv2.arcLength(contour, True)
    
    if method == 'percentage':
        # Standard approach: 1-3% of perimeter
        return 0.02 * peri
    elif method == 'adaptive':
        # Adaptive based on contour complexity
        area = cv2.contourArea(contour)
        complexity = peri / (2 * np.sqrt(np.pi * area))  # Circularity metric
        epsilon_factor = 0.01 + (complexity - 1) * 0.01  # Range: 0.01-0.03
        return max(0.01, min(0.05, epsilon_factor)) * peri
    elif method == 'iterative':
        # Iterative refinement to find best epsilon for 4 points
        for eps_factor in np.linspace(0.01, 0.1, 20):
            epsilon = eps_factor * peri
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4:
                return epsilon
        return 0.02 * peri  # Fallback


def convex_hull_quadrilateral(contour):
    """
    Method 1: Use convex hull + Douglas-Peucker approximation
    Most robust for general shapes
    """
    # Get convex hull
    hull = cv2.convexHull(contour)
    
    # Calculate adaptive epsilon
    epsilon = adaptive_epsilon_approximation(hull, 'iterative')
    
    # Approximate to polygon
    approx = cv2.approxPolyDP(hull, epsilon, True)
    
    # If not 4 points, try different epsilon values
    if len(approx) != 4:
        peri = cv2.arcLength(hull, True)
        for eps_factor in [0.015, 0.025, 0.03, 0.04, 0.05]:
            epsilon = eps_factor * peri
            approx = cv2.approxPolyDP(hull, epsilon, True)
            if len(approx) == 4:
                break
    
    return approx if len(approx) == 4 else None


def corner_based_quadrilateral(contour):
    """
    Method 2: Find extreme points and create quadrilateral
    Good for rectangular/box-like shapes
    """
    hull = cv2.convexHull(contour)
    hull = hull.reshape(-1, 2)
    
    # Find extreme points
    leftmost = tuple(hull[hull[:, 0].argmin()])
    rightmost = tuple(hull[hull[:, 0].argmax()])
    topmost = tuple(hull[hull[:, 1].argmin()])
    bottommost = tuple(hull[hull[:, 1].argmax()])
    
    # Find corner candidates by combining extreme directions
    top_left = hull[np.argmin(hull[:, 0] + hull[:, 1])]
    top_right = hull[np.argmax(hull[:, 0] - hull[:, 1])]
    bottom_left = hull[np.argmax(-hull[:, 0] + hull[:, 1])]
    bottom_right = hull[np.argmax(hull[:, 0] + hull[:, 1])]
    
    corners = np.array([top_left, top_right, bottom_right, bottom_left])
    
    return corners.reshape(-1, 1, 2)


def minimum_area_quadrilateral(contour):
    """
    Method 3: Start with minimum area rectangle and refine
    Best for near-rectangular shapes
    """
    # Get minimum area rectangle
    rect = cv2.minAreaRect(contour)
    box_points = cv2.boxPoints(rect)
    box_points = np.intp(box_points)
    
    # Try to fit the box to the actual contour better
    hull = cv2.convexHull(contour)
    
    # For each box corner, find the closest hull point
    refined_corners = []
    for box_corner in box_points:
        distances = np.sum((hull.reshape(-1, 2) - box_corner) ** 2, axis=1)
        closest_idx = np.argmin(distances)
        refined_corners.append(hull[closest_idx][0])
    
    return np.array(refined_corners).reshape(-1, 1, 2)


def iterative_douglas_peucker(contour, target_vertices=4):
    """
    Method 4: Iterative Douglas-Peucker with binary search
    Most accurate for complex shapes
    """
    peri = cv2.arcLength(contour, True)
    
    # Binary search for optimal epsilon
    low, high = 0.005 * peri, 0.15 * peri
    best_approx = None
    
    for _ in range(20):  # Max iterations
        mid = (low + high) / 2
        approx = cv2.approxPolyDP(contour, mid, True)
        
        if len(approx) == target_vertices:
            best_approx = approx
            break
        elif len(approx) > target_vertices:
            low = mid
        else:
            high = mid
    
    return best_approx


def robust_quadrilateral_detection(contour, mask_shape):
    """
    Main function combining multiple methods for robust detection
    """
    methods = [
        ("Convex Hull + Adaptive", convex_hull_quadrilateral),
        ("Corner-based", corner_based_quadrilateral),
        ("Minimum Area Refined", minimum_area_quadrilateral),
        ("Iterative Douglas-Peucker", iterative_douglas_peucker)
    ]
    
    best_quad = None
    best_score = -1
    best_method = ""
    
    for method_name, method_func in methods:
        try:
            quad = method_func(contour)
            
            if quad is not None and len(quad) == 4:
                # Score based on area coverage and shape regularity
                quad_area = cv2.contourArea(quad)
                contour_area = cv2.contourArea(contour)
                
                # Create mask for intersection calculation
                quad_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.fillPoly(quad_mask, [quad], 255)
                
                contour_mask = np.zeros(mask_shape, dtype=np.uint8)
                cv2.fillPoly(contour_mask, [contour], 255)
                
                intersection = cv2.bitwise_and(quad_mask, contour_mask)
                intersection_area = cv2.countNonZero(intersection)
                
                # Score combines area ratio and intersection coverage
                area_ratio = min(quad_area, contour_area) / max(quad_area, contour_area)
                coverage = intersection_area / cv2.countNonZero(contour_mask)
                score = area_ratio * 0.3 + coverage * 0.7
                
                if score > best_score:
                    best_score = score
                    best_quad = quad
                    best_method = method_name
                    
        except Exception as e:
            print(f"Method {method_name} failed: {e}")
            continue
    
    return best_quad, best_method, best_score


# Load YOLOv8 segmentation model
model = YOLO("colordet.pt")

# Configure RealSense streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Capture one frame
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()

if not color_frame:
    print("❌ No color frame captured")
    pipeline.stop()
    exit()

# Convert to numpy array
color_image = np.asanyarray(color_frame.get_data())

# Run YOLOv8 inference
results = model(color_image)
boxes = results[0].boxes
masks = results[0].masks
names = model.names

if masks is not None and boxes is not None:
    num_objects = masks.data.shape[0]
    print(f"Objects detected: {num_objects}")

    mask_img = results[0].plot(labels=True, boxes=False)

    for i, (box, mask) in enumerate(zip(boxes, masks)):
        cls_id = int(box.cls.item())
        label = names[cls_id]

        # Convert mask tensor to numpy
        mask_np = mask.data.squeeze().cpu().numpy().astype("uint8") * 255
        
        print(f"Processing object {i+1}: {label}")
        print(f"Mask shape: {mask_np.shape}, dtype: {mask_np.dtype}")

        # Find contours
        contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            print(f"No contours found for {label}")
            continue
            
        # Use the largest contour
        cnt = max(contours, key=cv2.contourArea)
        
        # Apply robust quadrilateral detection
        quad_pts, method_used, confidence = robust_quadrilateral_detection(cnt, mask_np.shape)
        
        if quad_pts is not None:
            quad_pts = np.intp(quad_pts)
            quad = quad_pts.reshape(-1, 2).tolist()
            
            print(f"ID: {cls_id} ({label}) | Method: {method_used}")
            print(f"Confidence: {confidence:.3f} | Quadrilateral points: {quad}")

            # Draw quadrilateral
            cv2.polylines(mask_img, [quad_pts], True, (0, 255, 0), 2)
            
            # Add method info to label
            label_text = f"{label} ({method_used[:8]})"
            cv2.putText(mask_img, label_text, tuple(quad_pts[0][0]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw corner points
            for corner in quad_pts:
                cv2.circle(mask_img, tuple(corner[0]), 3, (255, 0, 0), -1)
        else:
            print(f"Failed to detect quadrilateral for {label}")
else:
    print("No objects detected")
    mask_img = color_image

# Save and display result
cv2.imwrite("realsense_segmentation_quads.jpg", mask_img)
print("✅ Saved as realsense_segmentation_quads.jpg")

cv2.imshow("YOLOv8 Quadrilaterals", mask_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
pipeline.stop()
