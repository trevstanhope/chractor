"""
Computer-Vision Motion estimation
Estimate motion of camera system

"""

import cv2, cv
import numpy as np
import time
import tools

# DEFAULT CONSTANTS


class CLORB:
    def __init__(self,
                cam=None,
                threshold=250,
                dist_coef=None,
                cam_matrix=None,
                crop=False,
                use_clahe=True,
                show_video=True,
                draw_lines=False,
                neighbors=1,
                use_crosscheck=True,
                CROP_WIDTH = 200,
                CROP_HEIGHT = 200,
                DIST_COEF = np.array([-3.20678032e+01, -6.02849983e-03, -3.21918860e-03, -7.12706263e-02, 2.41369510e-07]),
                CAM_MATRIX = np.array([[8.84126845e+03, 0.00000000e+00, 3.20129093e+02],
                                       [0.00000000e+00, 8.73308727e+03, 2.40511239e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
                CAM_WIDTH = 160,
                CAM_HEIGHT = 120,
                DEG_TOLERANCE = 2, # the tolerance for finding vectors of similar direction (was 2)
                FPS = 25,
                ZOOM = 0.975,
                CLIP_LIMIT = 2.0,
                TILE_GRID_SIZE = (8,8)):
        
        # Keyword Args
        if cam is None:
            self.cam = cv2.VideoCapture(0)
        else:
            self.cam = cam
        self.show_video = show_video
        self.draw_lines = draw_lines
        self.crop = crop
        self.cam.set(3, CAM_WIDTH)
        self.cam.set(4, CAM_HEIGHT)

        # Constants
        self.CAM_MATRIX = CAM_MATRIX
        self.DIST_COEF = DIST_COEF
        self.CAM_WIDTH = CAM_WIDTH
        self.CAM_HEIGHT = CAM_HEIGHT
        self.DEG_TOLERANCE = DEG_TOLERANCE
        self.CROP_WIDTH = CROP_WIDTH
        self.CROP_HEIGHT = CROP_HEIGHT
        self.FPS = FPS
        self.ZOOM = ZOOM
        self.CLIP_LIMIT = CLIP_LIMIT
        self.TILE_GRID_SIZE = TILE_GRID_SIZE
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.CAM_MATRIX,
                                                           self.DIST_COEF,
                                                           None,
                                                           self.CAM_MATRIX,
                                                           (self.CAM_WIDTH, self.CAM_HEIGHT),
                                                           5)
        
        ## Feature-Detector
        self.use_crosscheck = use_crosscheck
        self.NEIGHBORS = neighbors
        self.feature_descriptor = cv2.ORB(threshold)
        self.matcher = cv2.BFMatcher(crossCheck=True)

        ## Equalization
        self.use_clahe = use_clahe
        self.clahe = cv2.createCLAHE(clipLimit=self.CLIP_LIMIT, tileGridSize=self.TILE_GRID_SIZE)
            
        ## Empty Variables
        self.pts1, self.desc1 = None, None
        self.pts2, self.desc2 = None, None

    def find_matches(self):
        if self.use_clahe:
            self.gray = self.clahe.apply(self.gray)
        dst = self.undistort(self.gray) # apply undistortion remap
        self.pts2, self.desc2 = self.pts1, self.desc1 # copy previous keypoints
        (self.pts1, self.desc1) = self.feature_descriptor.detectAndCompute(dst, None) # Find key-points between set1 and set2
        if self.use_crosscheck:
            self.matches = self.matcher.knnMatch(self.desc1, self.desc2, k=1) # knn-Match descriptor sets
        else:
            self.matches = self.matcher.knnMatch(self.desc1, self.desc2, k=self.NEIGHBORS) # knn-Match descriptor sets
        m = len(self.matches)
        return m # returns total matches found

    def calculate_vector(self):
        """
        """

        # Grab matches
        pairs = []
        for m in self.matches:
            if len(m) != 0:
                pt1 = self.pts1[m[0].queryIdx]
                pt2 = self.pts2[m[0].trainIdx]
                xy1 = (pt1.pt[0], pt1.pt[1])
                xy2 = (pt2.pt[0], pt2.pt[1])
                pairs.append((xy1, xy2))
        vectorized = [self.vectorize(pt1, pt2) for (pt1, pt2) in pairs]
        V,T = map(list,zip(*vectorized)) # [v for (v,t) in vectorized]
        v_all = np.array(V)
        t_all = np.array(T)

        if len(v_all) != 0:
            v_best, t_best = self.hist_filter(v_all, t_all) # Filter for best matches
            t = np.median(t_best)
            v = np.median(v_best) #!TODO: estimation for speed, axiom: middle of pack is most likely
            n = len(v_all)

            if self.draw_lines:
                V_r = np.array(np.round(v_best*10), np.uint8)
                T_r = np.array(np.round(t_best), np.uint8) 
                mask = np.zeros((200, 360, 3), np.uint8)
                cv2.imshow('', self.gray)
                mask[:, T_r, 2] = 255
                mask[V_r[V_r < 200], :, 1] = 255
                height, width = self.gray.shape[:2]
                res = cv2.resize(mask,(width, height), interpolation = cv2.INTER_CUBIC)
                output = np.hstack((res, cv2.cvtColor(self.gray,cv2.COLOR_GRAY2RGB)))
                cv2.imshow('', output)
                if cv2.waitKey(5) == 0:
                    pass
        else:
            t = 'NaN'
            v = 'NaN'
            n = 0
        p = len(pairs)
        return v, t, n, p # returns speed, direction, number of valid matches, and number of vector-pairs

    def hist_filter(self, v, t):
        t_rounded = np.around(t, 0).astype(np.int32)
        t_bins = [tools.maprange(i, (-180,180), (0,360)) for i in t_rounded]
        t_counts = np.bincount(t_bins)
        t_mode = np.argmax(t_counts)
        t_best = np.isclose(t_bins, t_mode, atol=self.DEG_TOLERANCE)
        v = v[t_best]
        t = t[t_best]
        return v, t

    def undistort(self, gray):
        """ Apply undistort remap to current image """
        return cv2.remap(gray, self.mapx, self.mapy, cv2.INTER_LINEAR) # use linear interpolation

    def vectorize(self, pt1, pt2):
        """ Calculate vectors of good matches """    
        (x1, y1) = pt1
        (x2, y2) = pt2
        d = self.ZOOM * np.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
        p = np.arctan2((x2 - x1), (y2 - y1))
        t = np.rad2deg(p) # converted to degrees
        v = (3.6 / 1000.0) * (d * float(self.FPS))# convert from mm/s to km/hr
        return (v,t)

    def get_latest(self):
        """
        """
        s = False
        while not s:
            s, bgr = self.cam.read()
        self.gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.show_video:
            cv2.imshow('', self.gray)
            if cv2.waitKey(5) == 0:
                pass
