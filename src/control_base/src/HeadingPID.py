import time
from utils import shortestAngleDiff

class HeadingPID:
    def __init__(self):
        self.kp, self.ki, self.kd = 3, 0, 0
        self.heading = 0
        self.target_heading = 0
        self.prev_time = time.time()
        self.cum_error = 0
        self.rate_error = 0
        self.last_error = 0
        self.vel_theta = 0
        self.MAX_LIMIT, self.MIN_LIMIT = 10, -10

    def updateTarget(self, theta):
        self.target_heading = theta

    def update(self, theta):
        self.heading = theta
        curr_time = time.time()
        elapsed_time = curr_time - self.prev_time
        error = shortestAngleDiff(self.target_heading, self.heading)

        self.cum_error += error * elapsed_time
        rate_error = (error-self.last_error)/elapsed_time

        self.vel_theta = self.kp*error + self.ki*curr_time + self.kd*rate_error
        self.vel_theta = max(self.MIN_LIMIT, min(self.MAX_LIMIT, self.vel_theta))

        self.last_error = error
        self.prev_time = curr_time
        return self.vel_theta


    