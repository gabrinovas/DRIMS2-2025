import cv2 as cv
import numpy as np

from sklearn import cluster
from typing import List, Tuple
from dataclasses import dataclass

from scipy.spatial.transform import Rotation as R
from .utils import draw_axis


@dataclass
class DieDetection:
    face_number: int        # Number of points detected on the die face
    centroid: np.ndarray    # 2D centroid of the die in the image plane
    T_d2c: np.ndarray         # Pose of the die w.r.t. camera frame

class DiceDetector():
    def __init__(
            self, 
            blob_detector: cv.SimpleBlobDetector_Params,
            dbscan_eps: float,
            lower_hsv: Tuple[int, int, int],
            upper_hsv: Tuple[int, int, int],
            K: np.ndarray,
            d: np.ndarray,
            T_c2w: np.ndarray,
            die_size: float,
            roi_size: int = 100,
            kernel_size: int = 5
        ):
        self.blob_detector_ = cv.SimpleBlobDetector_create(blob_detector)
        self.dbscan = cluster.DBSCAN(eps=dbscan_eps, min_samples=1)
        
        self.lower_hsv_ = lower_hsv
        self.upper_hsv_ = upper_hsv

        self.K_ = K
        self.d_ = d
        self.T_c2w_ = T_c2w

        self.die_size_ = die_size

        self.roi_size_ = roi_size
        self.kernel_size_ = kernel_size
        
    def get_dice_from_blobs(self, img: np.ndarray) -> List[DieDetection]:
        img = cv.undistort(img, self.K_, self.d_)

        blobs = self.blob_detector_.detect(img)

        # Get centroids of all blobs
        X = np.asarray([
            b.pt for b in blobs if b.pt is not None
        ])

        if len(X) <= 0:
            return [], [], []

        # Cluster the detected points by proximity to detect all dice in the scene
        clustering = self.dbscan.fit(X)
       
        # Total number of detected dice in the scene
        num_dice = max(clustering.labels_) + 1

        # 2D centroids of each die
        dice, dice_imgs, dice_masks = [], [], []
        for die_id in range(num_dice):
            die_points = X[clustering.labels_ == die_id]
            die_centroid = np.mean(die_points, axis=0) # Points are symmetrically distributed over the die face
            success, die_pose, die_mask, die_img = self.detect_die(img, die_centroid)

            if not success:
                continue

            dice_imgs.append(die_img)
            dice_masks.append(die_mask)

            dice.append(DieDetection(
                face_number=len(die_points),
                centroid=die_centroid,
                T_d2c=die_pose
            ))

        return dice, dice_imgs, dice_masks

    def detect_die(self, img: np.ndarray, die_centroid: np.ndarray) -> Tuple[bool, np.ndarray, np.ndarray, np.ndarray]:
        # Crop a region of interest around the die centroid
        h, w = img.shape[:2]
        x, y = die_centroid.astype(int)
        x1, y1 = max(0, x - self.roi_size_), max(0, y - self.roi_size_)
        x2, y2 = min(w, x + self.roi_size_), min(h, y + self.roi_size_)
        roi = img[y1:y2, x1:x2]

        # Mask the die color (yellow)
        blurred = cv.GaussianBlur(roi, (self.kernel_size_, self.kernel_size_), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, self.lower_hsv_, self.upper_hsv_)
        kernel = cv.getStructuringElement(cv.MORPH_CROSS, (self.kernel_size_, self.kernel_size_))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        # Detect the die orientation from the masked region contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False, None, None, None

        cnt = max(contours, key=cv.contourArea)
        rect = cv.minAreaRect(cnt)   # (center(x,y), (w,h), angle)
        (w, h), angle = rect[1], rect[2]

        # Normalize OpenCV angle (It is in the range [-90, 0))
        yaw = angle if (w < h) else angle + 90
        yaw = yaw % 180

        # Get pose from the estimate yaw
        pose = self.get_die_pose(die_centroid, yaw)

        # Get outputs images (mask and bbox)
        mask_full = np.zeros(img.shape[:2], dtype=np.uint8)
        mask_full[y1:y2, x1:x2] = mask

        box = cv.boxPoints(rect).astype(int)
        box = box + np.array([x1, y1]) # Offset to original image coordinates
        img = cv.drawContours(img, [box], 0, (0, 255, 0), 2)

        img = draw_axis(img, pose, self.K_)

        return True, pose, mask_full, img

    def get_die_pose(self, die_centroid: np.ndarray, yaw: float) -> np.ndarray:
        # Distance of the die top face to the camera plane
        z_d = np.abs(self.T_c2w_[2, 3]) - self.die_size_  # Distance is always positive

        fx = self.K_[0, 0]
        fy = self.K_[1, 1]

        cx = self.K_[0, 2]
        cy = self.K_[1, 2]

        u, v = die_centroid
    
        # Get three tensors containing the [x, y, z] positions of all points with respect to the camera frame
        x = (u - cx) * z_d / fx
        y = (v - cy) * z_d / fy
        z = z_d

        R_yaw = R.from_euler('z', yaw, degrees=True).as_matrix()

        T_die = np.eye(4)
        T_die[:3, :3] = R_yaw
        T_die[:3, 3] = [x, y, z]

        return T_die # W.r.t camera frame
