import time

class PositionPID:
    def __init__(self):
        self.kp, self.ki, self.kd = 3, 0, 0
        self.posX, self.posY = 0,0
        self.targetX, self.targetY = 0,0

        self.prev_time = time.time()
        self.cum_errorX, self.cum_errorY = 0, 0
        self.rate_errorX, self.rate_errorY = 0, 0
        self.last_errorX, self.last_errorY = 0, 0
        self.velX, self.velY = 0, 0

        self.MAX_LIMIT, self.MIN_LIMIT = 100, -100
        self.first = False

    def updateTarget(self, x, y):
        self.targetX, self.targetY = x, y

    def update(self, posX, posY):
        self.posX, self.posY = posX, posY
        if self.first:
            self.targetX, self.targetY = posX, posY
            self.first = False
        curr_time = time.time()

        elapsed_time = curr_time - self.prev_time
        errorX = self.targetX - self.posX
        errorY = self.targetY - self.posY

        self.cum_errorX += errorX * elapsed_time
        self.cum_errorY += errorY * elapsed_time

        self.rate_errorX = (errorX-self.last_errorX)/elapsed_time
        self.rate_errorY = (errorY-self.last_errorY)/elapsed_time

        self.velX = self.kp*errorX + self.ki*curr_time + self.kd*self.rate_errorX
        self.velY = self.kp*errorY + self.ki*curr_time + self.kd*self.rate_errorY

        self.velX = max(self.MIN_LIMIT, min(self.MAX_LIMIT, self.velX))
        self.velY = max(self.MIN_LIMIT, min(self.MAX_LIMIT, self.velY))

        self.last_errorX, self.last_errorY = errorX, errorY
        self.prev_time = curr_time
        return self.velX, self.velY

    