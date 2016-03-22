import time


class PID:

    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.accumulated_error = 0.0
        self.acceptable_error = 0.0
        self.accumulated_max = 0.0
        self.prev_time = 0
        self.prev_error = None

    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_kd(self, kd):
        self.kd = kd

    def set_acceptable_error(self, error):
        self.acceptable_error = error

    def set_accumulated_error_max(self, error):
        self.accumulated_max = error

    def out(self, error):
        # P
        p = self.kp * error

        # D
        current_time = time.time()
        if self.prev_error is not None and self.prev_time != 0:
            error_delta = error - self.prev_error
            print(error_delta)
            time_delta = self.prev_time - current_time
            d = (error_delta/time_delta * self.kd)
        else:
            d = 0
        self.prev_time = current_time

        # I factor
        self.accumulated_error += error
        if error <= self.acceptable_error:
            self.accumulated_error = 0
        if self.accumulated_error > self.accumulated_max:
            self.accumulated_error = self.accumulated_max
        i = self.accumulated_error * self.ki

        self.prev_error = error

        return p + i + d
