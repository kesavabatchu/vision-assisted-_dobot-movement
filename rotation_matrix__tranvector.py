import numpy as np

def compute_transformation(points_camera, points_base):
    """
    Computes rotation matrix (R) and translation vector (t) that transforms
    points from the camera frame to the robotic arm base frame.
    Only supports exactly three points for minimal cases.

    Args:
        points_camera (list or np.array): 3x3 array of 3D points in camera frame.
        points_base (list or np.array): 3x3 array of corresponding 3D points in base frame.

    Returns:
        R (np.array): 3x3 rotation matrix.
        t (np.array): 3x1 translation vector.
    """
    points_camera = np.array(points_camera, dtype=np.float64)
    points_base = np.array(points_base, dtype=np.float64)

    if len(points_camera) != 3 or len(points_base) != 3:
        raise ValueError("Must provide exactly three points for each set.")

    centroid_camera = np.mean(points_camera, axis=0)
    centroid_base = np.mean(points_base, axis=0)

    centered_camera = points_camera - centroid_camera
    centered_base = points_base - centroid_base

    H = centered_camera.T @ centered_base
    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_base - R @ centroid_camera

    return R, t

def main():
    # Example usage with three points
    points_camera = [
        [-0.0714,  0.20509,     0.6600],
        [-0.08709,   0.206186,    0.8060],
        [-0.346973,  0.2025307,   1.0290]
    ]

    points_base = [
        [0,      -0.354,     0],
        [0,      -0.204,    0],
        [  -0.204,  0,      0]
    ]

    R, t = compute_transformation(points_camera, points_base)
    
    # Set numpy print options to always show decimal, not scientific notation
    np.set_printoptions(suppress=True, precision=6)
    
    print("Rotation matrix (R):")
    print(R)
    print("\nTranslation vector (t):")
    print(t)

if __name__ == "__main__":
    main()
