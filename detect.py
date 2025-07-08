from __future__ import division
import time
import cv2
import numpy as np
class LaneDetector:
    def __init__(self, road_horizon, prob_hough=True):
        self.prob_hough = prob_hough
        self.vote = 50
        self.roi_theta = 0.3
        self.road_horizon = road_horizon
    @staticmethod
    def _standard_hough(img, init_vote):
        """Hough transform wrapper to return a list of points like PHough does
        """
        lines = cv2.HoughLines(img, 1, np.pi/180, init_vote)
        points = [[]]
        if lines is not None:
            for l in lines:
                for rho, theta in l:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*a)
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*a)
                    points[0].append((x1, y1, x2, y2))
        return points
    @staticmethod
    def _base_distance(x1, y1, x2, y2, width):
        if x2 == x1:
            return (width*0.5) - x1
        m = (y2-y1)/(x2-x1)
        c = y1 - m*x1
        base_cross = -c/m
        return (width*0.5) - base_cross

    def _scale_line(self, x1, y1, x2, y2, frame_height):
        if x1 == x2:
            if y1 < y2:
                y1 = self.road_horizon
                y2 = frame_height
                return x1, y1, x2, y2
            else:
                y2 = self.road_horizon
                y1 = frame_height
                return x1, y1, x2, y2
        if y1 < y2:
            m = (y1-y2)/(x1-x2)
            x1 = ((self.road_horizon-y1)/m) + x1
            y1 = self.road_horizon
            x2 = ((frame_height-y2)/m) + x2
            y2 = frame_height
        else:
            m = (y2-y1)/(x2-x1)
            x2 = ((self.road_horizon-y2)/m) + x2
            y2 = self.road_horizon
            x1 = ((frame_height-y1)/m) + x1
            y1 = frame_height
        return x1, y1, x2, y2

    def detect(self, frame):
        # Focus ROI on lower half of the frame for better lane detection
        height, width = frame.shape[:2]
        roi_top = max(self.road_horizon, int(height * 0.5))
        roi = frame[roi_top:height, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        # Adaptive Canny thresholds
        v = np.median(blur)
        lower = int(max(0, 0.66 * v))
        upper = int(min(255, 1.33 * v))
        edges = cv2.Canny(blur, lower, upper)

        if self.prob_hough:
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.vote,
                                    minLineLength=50, maxLineGap=50)
        else:
            lines = self._standard_hough(edges, self.vote)

        left_bound = None
        right_bound = None
        left_dist = -np.inf
        right_dist = np.inf
        if lines is not None:
            # Adjust coordinates to full frame
            lines = lines + np.array([0, roi_top, 0, roi_top]).reshape((1, 1, 4))
            for l in lines:
                for x1, y1, x2, y2 in l:
                    # Filter out short lines
                    if np.hypot(x2-x1, y2-y1) < 60:
                        continue
                    # Filter out nearly horizontal lines
                    theta = np.abs(np.arctan2((y2-y1), (x2-x1)))
                    if theta < self.roi_theta:
                        continue
                    dist = self._base_distance(x1, y1, x2, y2, width)
                    if dist < 0 and dist > left_dist:
                        left_bound = (x1, y1, x2, y2)
                        left_dist = dist
                    elif dist > 0 and dist < right_dist:
                        right_bound = (x1, y1, x2, y2)
                        right_dist = dist
        # Scale lines to horizon and bottom
        if left_bound is not None:
            left_bound = self._scale_line(*left_bound, frame.shape[0])
        if right_bound is not None:
            right_bound = self._scale_line(*right_bound, frame.shape[0])
        # Return None for missing bounds for robustness
        return [left_bound, right_bound]
