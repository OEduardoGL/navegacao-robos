import math

def sat(x, m):
    return max(-m, min(m, x))

def heading_to(x, y, gx, gy):
    return math.atan2(gy - y, gx - x)

def dist(a, b, c, d):
    return math.hypot(a - c, b - d)

