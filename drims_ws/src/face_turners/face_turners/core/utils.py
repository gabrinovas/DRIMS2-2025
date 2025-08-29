import cv2 as cv
import numpy as np

def draw_axis(img: np.ndarray, pose: np.ndarray, K: np.ndarray, length: float = 0.01) -> np.ndarray:
    axis_points = np.float32(
        [[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]]
    ).reshape(-1, 3)

    # Project the 3D axis points to 2D image points
    tvec = pose[:3, 3]
    rvec, _ = cv.Rodrigues(pose[:3, :3])
    image_points, _ = cv.projectPoints(axis_points, rvec, tvec, K, distCoeffs=None)

    # Draw the axis lines on the image
    image_points = np.int32(image_points).reshape(-1, 2)
    output_image = img.copy()
    cv.line(
        output_image,
        tuple(image_points[0]),
        tuple(image_points[1]),
        (0, 0, 255),
        5,
    )  # X-axis in red
    cv.line(
        output_image,
        tuple(image_points[0]),
        tuple(image_points[2]),
        (0, 255, 0),
        5,
    )  # Y-axis in green
    cv.line(
        output_image,
        tuple(image_points[0]),
        tuple(image_points[3]),
        (255, 0, 0),
        5,
    )  # Z-axis in blue

    return output_image
