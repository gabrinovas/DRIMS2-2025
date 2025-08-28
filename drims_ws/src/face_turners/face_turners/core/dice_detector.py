import cv2 as cv
import numpy as np

from sklearn import cluster
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class DieDetection:
    face_number: int        # Number of points detected on the die face
    centroid: np.ndarray    # 2D centroid of the die in the image plane
    yaw: np.ndarray         # Yaw angle of the die w.r.t. camera frame (Z axis assuming die is on plane)

class DiceDetector():
    def __init__(self, blob_detector: cv.SimpleBlobDetector_Params, dbscan_eps: float, lower_hsv: Tuple[int, int, int], upper_hsv: Tuple[int, int, int], roi_size: int = 100, k: int = 5):
        self.blob_detector_ = cv.SimpleBlobDetector_create(blob_detector)
        self.dbscan = cluster.DBSCAN(eps=dbscan_eps, min_samples=1)
        
        self.lower_hsv_ = lower_hsv
        self.upper_hsv_ = upper_hsv

        self.roi_size_ = roi_size
        self.k_ = k
        
    def get_dice_from_blobs(self, img: np.ndarray) -> List[DieDetection]:
        blobs = self.blob_detector_.detect(img)

        # Get centroids of all blobs
        X = np.asarray([
            b.pt for b in blobs if b.pt is not None
        ])

        if len(X) <= 0:
            return []
        
        # Cluster the detected points by proximity to detect all dice in the scene
        clustering = self.dbscan.fit(X)
       
        # Total number of detected dice in the scene
        num_dice = max(clustering.labels_) + 1

        # 2D centroids of each die
        dice = []
        for die_id in range(num_dice):
            die_points = X[clustering.labels_ == die_id]
            die_centroid = np.mean(die_points, axis=0) # Points are symmetrically distributed over the die face
            die_yaw = self.get_die_orientation(img, die_centroid)
            
            dice.append(DieDetection(
                face_number=len(die_points),
                centroid=die_centroid,
                yaw=die_yaw
            ))

        return dice

    def get_die_orientation(self, img: np.ndarray, die_centrodid: np.ndarray) -> float:
        # Crop a region of interest around the die centroid
        h, w = img.shape[:2]
        x, y = die_centrodid.astype(int)
        x1, y1 = max(0, x - self.roi_size_), max(0, y - self.roi_size_)
        x2, y2 = min(w, x + self.roi_size_), min(h, y + self.roi_size_)
        roi = img[y1:y2, x1:x2]

        # Mask the die color (yellow)
        blurred = cv.GaussianBlur(roi, (self.k_, self.k_), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, self.lower_hsv_, self.upper_hsv_)
        kernel = cv.getStructuringElement(cv.MORPH_CROSS, (self.k_, self.k_))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        
        # Detect the die orientation from the masked region contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnt = max(contours, key=cv.contourArea)
        _, (w, h), angle = cv.minAreaRect(cnt)   # (center(x,y), (w,h), angle)

        # Normalize OpenCV angle (It is in the range [-90, 0))
        yaw = angle if (w < h) else angle + 90
        yaw = yaw % 180

        return yaw
