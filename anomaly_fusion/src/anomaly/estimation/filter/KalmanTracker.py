from pykalman import KalmanFilter
import numpy as np

##### parameters ########
# process noise: the variability in how fast the Hexbug is speeding up (stdv of acceleration: meters/sec^2)
HexAccel_noise_mag = .01
# Error in measurement.
Ez = np.array([[0.1, 0], [0, 0.1]])


#########################3



#### Kalman model ############
## A: function that creates the transition matrix based on
f_transition_matrix = lambda dt: np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
## B: offset function based on dt.
f_offset = lambda dt: np.array([[dt, 0], [0, dt], [1, 0], [0, 1]])
## C: constant measurement matrix.
measurement_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

# Error in transition.
f_Ex = lambda dt: np.array([[dt ** 4 / 4, 0, dt ** 3 / 2, 0],
                            [0, dt ** 4 / 4, 0, dt ** 3 / 2],
                            [dt ** 3 / 2, 0, dt ** 2, 0],
                            [0, dt ** 3 / 2, 0, dt ** 2]]) * HexAccel_noise_mag ** 2

dt1 = 1

### Initial values
# initial estate



class KalmanTracker:
    def __init__(self, initial_x=[0, 0, 0, 0], initial_cov=f_Ex(10000)):
        self.estimated_x = initial_x
        self.estimated_cov = initial_cov

        # Initialize the Kalman Filter
        self.kf = KalmanFilter(
            transition_matrices=f_transition_matrix(dt1),  # Transition_matrix
            observation_matrices=measurement_matrix,  # Observation_matrix
            transition_covariance=f_Ex(dt1),  # Initial_transition_covariance,
            observation_covariance=Ez,  # Initial_observation_covariance,
            initial_state_mean=self.estimated_x,  # initial_state_mean,
            initial_state_covariance=self.estimated_cov,  # initial_state_covariance,
            random_state=0
        )

        # self.kf.filter()
        # self.kf.filter_update()
        pass


    def actual_state(self, dt):
        return [self.estimated_x[0] + self.estimated_x[2] * dt,  # x + Vx * dt
                self.estimated_x[1] + self.estimated_x[2] * dt,  # y + Vy * dt
                self.estimated_x[2],  # Vx
                self.estimated_x[3]]  # Vy

        # TODO return covariance, test with this
        # self.estimated_x, self.estimated_cov = self.kf.filter_update(
        # self.estimated_x,
        #     self.estimated_cov,
        #     # transition_offset=tran_offset
        #     transition_matrix=f_transition_matrix(dt)
        # )

    def estimate_state_with_measurement(self, dt, measurement):
        ## time is constant


        self.estimated_x, self.estimated_cov = self.kf.filter_update(
            self.estimated_x,
            self.estimated_cov,
            measurement,
            # transition_offset=tran_offset
            # transition_matrix=f_transition_matrix(dt)
        )

        return self.estimated_x, self.estimated_cov