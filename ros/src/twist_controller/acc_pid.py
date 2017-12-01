
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class AccumulatingPID(object):
    def __init__(self, kp, kp_neg, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.kp_neg = kp_neg
        self.kd = kd
        self.min = mn
        self.max = mx

        self.last_error = 0.0;

        self.last_val = 0.0;

    def reset(self):
        self.last_error = 0.0;
        self.last_val = 0.0

    def step(self, error, sample_time):
        derivative = (error - self.last_error) / sample_time;

        self.last_val += (self.kp if error > 0 else self.kp_neg) * error + self.kd * derivative;
        self.last_val = max(self.min, min(self.last_val, self.max))

        self.last_error = error

        return self.last_val
