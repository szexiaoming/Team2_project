import math

def norm2(dx, dy):
    return math.sqrt(dx*dx + dy*dy)

def normalize(dx, dy):
    n = norm2(dx, dy)
    if n < 1e-9: return 0.0, 0.0
    return dx/n, dy/n

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def get_heading(node):
    """从Webots节点获取朝向角"""
    o = node.getOrientation()
    return math.atan2(o[3], o[0])

def get_pos(node):
    """从Webots节点获取坐标(x, y)"""
    p = node.getPosition()
    return float(p[0]), float(p[1])

def get_axes(node):
    """获取坐标轴(用于检测摔倒)"""
    o = node.getOrientation()
    x_axis = (o[0], o[3], o[6])
    z_axis = (o[2], o[5], o[8])
    return x_axis, z_axis