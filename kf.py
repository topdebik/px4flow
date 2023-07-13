from pykalman import KalmanFilter


class kf:
    def __init__(self, transition_matrices=[1], observation_matrices=[1], initial_state_mean=0, initial_state_covariance=1, observation_covariance=1, transition_covariance=0.03, initial_state=0, initial_covariance=1):
        self.filter = KalmanFilter(transition_matrices,
            observation_matrices,
            initial_state_mean,
            initial_state_covariance,
            observation_covariance,
            transition_covariance
        )
        self.initial_state = initial_state
        self.initial_covariance = initial_covariance

    def filter_value(self, value):
        self.initial_state, self.initial_covariance=self.filter.filter_update(
            filtered_state_mean=self.initial_state,
            filtered_state_covariance=self.initial_covariance,
            observation=value
        )
        return self.initial_state.flatten()[0]