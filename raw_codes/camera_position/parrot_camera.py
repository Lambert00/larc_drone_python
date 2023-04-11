import cv2
import numpy as np

class ParrotCamera():
    def __init__(self, real_dimx, real_dimy):
        
        # From https://github.com/AutonomyLab/bebop_autonomy/blob/indigo-devel/bebop_driver/data/bebop2_camera_calib.yaml
        # Real_dimx and real_dimy in meters
        
        # self.camera_matrix = np.array([[539.403503, 0, 429.275072],
        #                                [0, 529.838562, 238.941372],
        #                                [0, 0, 1]], dtype=np.float32)

        # self.camera_matrix = np.array([[540, 0, 856/2],
        #                                [0, 530, 480/2],
        #                                [0, 0, 1]])
        
        self.camera_matrix = np.array([[530.42901611, 0, 434.85937575],
                                       [0, 520.19354248, 231.65746157],
                                       [0, 0, 1]], np.float32)
        
        self.dist_coeffs = np.zeros((5,1))
        # self.dist_coeffs = np.array([0.004974, -0.000130, -0.001212, 0.002192, 0.00000], np.float32)
        # self.dist_coeffs = np.array([0.00945204, -0.01249052, -0.00477354, 0.00279677, 0.00451873], np.float32)
    
        # Clockwise start from up-left
        points_3D = np.array([[0, 0, 0],
                              [1, 0, 0],
                              [1, 1, 0],
                              [0, 1, 0]], dtype=np.float32)

        dimensions = [real_dimx, real_dimy, 0]
        self.points_3D = points_3D*dimensions
        
        self.axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
        
    def draw(self, img, imgpts, centroid):
        corner = tuple(map(round,centroid))
        try:
            img = cv2.line(img, corner, tuple(map(round,imgpts[0].ravel())), (255,0,0), 5)
            img = cv2.line(img, corner, tuple(map(round,imgpts[1].ravel())), (0,255,0), 5)
            img = cv2.line(img, corner, tuple(map(round,imgpts[2].ravel())), (0,0,255), 5)
        except: pass
        return img
        
    def get_pose(self, points_2D):
        _, rvec, tvec = cv2.solvePnP(self.points_3D, points_2D, self.camera_matrix, self.dist_coeffs)
        return rvec, tvec
    
    def project_3D(self, rvec, tvec, points_2D, frame):
        imgpts, jac = cv2.projectPoints(self.axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        frame = self.draw(frame,imgpts, points_2D)
        