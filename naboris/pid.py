
class PID:

    def __init__(self, kp, kd, ki):
        self.reset()

        self.kp = kp
        self.kd = kd
        self.ki = ki

    def reset(self):
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        p_term = self.kp * error
        i_term = self.ki * self.error_sum * dt
        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.error_sum += error

        return p_term + i_term + d_term
