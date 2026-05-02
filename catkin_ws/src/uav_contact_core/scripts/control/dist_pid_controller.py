class DistPIDController:
    def __init__(self, kp, ki, kd, dt, v_max):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError("dt must be > 0")
        self.v_max = float(v_max)

        self.integral = 0.0
        self.prev_error = None

    def compute(self, distance, target_distance, phase_enabled):
        if not phase_enabled:
            return 0.0

        error = float(distance) - float(target_distance)

        p_term = self.kp * error
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        if self.prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self.prev_error) / self.dt

        output = p_term + i_term + d_term
        output = max(min(output, self.v_max), -self.v_max)

        self.prev_error = error
        return output
