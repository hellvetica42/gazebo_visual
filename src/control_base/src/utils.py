import math
def shortestAngleDiff(th1, th2):
    err = (th1+math.pi) - (th2+math.pi)
    diff = err

    if diff < 0.0:
        if abs(diff) > (2*math.pi + diff):
            diff = 2*math.pi + diff

    else:

        if diff > abs(diff - 2*math.pi):
            diff = diff - 2*math.pi

    return diff