import numpy as np

class KalmanFilter:
    def __init__(self, Q: float, R: float, delta_t: float):
        # Q: Process Noise Covariance, R: Measurement Noise Covariance
        self.Q = np.array([[Q, 0],
                           [0,10*Q]])

        self.B = delta_t
        self.R = np.array([[R, 0],
                           [0, R]])
  
        self.delta_t = delta_t
        self.F = np.array([[1, self.delta_t],[0, 1]])
        self.P = np.array([[0.1,0],[0.1,0]])
        self.H = np.array([[1,0],[0,1]])

        self.fin = np.array([[1], [0]])
        self.pos_hat = 0

        self.z_hat = np.array([[0], [0]])
        self.x_hat = np.array([[0], [0]])
        self.speed_hat = 0

    def update_filter(self, imu: float) -> float:
        vector = np.array([[self.speed_hat],[imu]])

        self.x_hat_minus = np.dot(self.F, self.x_hat) + np.dot(self.B, vector)
        self.P_minus = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        S = np.dot(np.dot(self.H, self.P_minus), self.H.T) + self.R  
        self.K = np.dot(np.dot(self.P_minus, self.H.T), np.linalg.inv(S)) 
        z = np.array([[self.pos_hat + self.speed_hat * self.delta_t],[self.speed_hat + imu* self.delta_t]])
        self.v_k = z - np.dot(self.H, self.x_hat_minus)
        self.x_hat = self.x_hat_minus + np.dot(self.K, self.v_k) 
        Y = np.eye(2) - np.dot(self.K, self.H)
        self.P = np.dot(Y, self.P_minus)
        #self.P = np.dot(np.dot(Y, self.P_minus), Y.T)  + np.dot(np.dot(self.K, self.R), self.K.T)
        self.z_hat = np.dot(self.H, self.x_hat)  
        
        self.speed_hat = self.x_hat[1,0]
        self.pos_hat = self.x_hat[0,0]
        print(self.x_hat)
        
        return self.pos_hat
