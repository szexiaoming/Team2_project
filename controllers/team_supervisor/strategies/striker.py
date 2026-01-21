import math
from utils import norm2, normalize, wrap_pi
from movement import action_to_target

# === 前锋 (Striker) ===
def run_striker(my_x, my_y, my_theta, bx, by, goal_target_xy, obstacles):
    gx, gy = goal_target_xy 
    
    dx_shot, dy_shot = gx - bx, gy - by
    dir_x, dir_y = normalize(dx_shot, dy_shot)
    desired_theta = math.atan2(dir_y, dir_x)

    # 理想站位
    DIST_BEHIND = 0.35 
    OFFSET_RIGHT = 0.05 
    stand_x = bx - dir_x * DIST_BEHIND + dir_y * OFFSET_RIGHT
    stand_y = by - dir_y * DIST_BEHIND - dir_x * OFFSET_RIGHT

    # 1. 激进射门检测
    dist_to_ball = norm2(my_x - bx, my_y - by)
    heading_to_goal = math.atan2(gy - my_y, gx - my_x)
    heading_err_goal = abs(wrap_pi(heading_to_goal - my_theta))
    heading_to_ball = math.atan2(by - my_y, bx - my_x)
    heading_err_ball = abs(wrap_pi(heading_to_ball - my_theta))

    if dist_to_ball < 0.2 and heading_err_goal < 1.0 and heading_err_ball < 0.8:
            dist_me_goal = norm2(my_x - gx, my_y - gy)
            dist_ball_goal = norm2(bx - gx, by - gy)
            if dist_me_goal > dist_ball_goal: 
                return "KICK_L"

    # 2. 绕行逻辑
    vec_br_x, vec_br_y = my_x - bx, my_y - by
    dist_br = norm2(vec_br_x, vec_br_y)
    dot = vec_br_x * dir_x + vec_br_y * dir_y
    
    if dot > -0.15 and dist_br < 0.5:
        cross = vec_br_x * dir_y - vec_br_y * dir_x
        orbit_radius = 0.45
        if cross > 0:
            nav_x = bx + dir_y * orbit_radius 
            nav_y = by - dir_x * orbit_radius
        else:
            nav_x = bx - dir_y * orbit_radius
            nav_y = by + dir_x * orbit_radius
        face_ball = math.atan2(by - my_y, bx - my_x)
        return action_to_target(my_x, my_y, my_theta, nav_x, nav_y, face_ball, obstacles, True)

    # 3. 正常跑位
    return action_to_target(my_x, my_y, my_theta, stand_x, stand_y, desired_theta, obstacles, True)

# === 后卫 (Defender) ===
def run_defender(my_x, my_y, my_theta, bx, by, goal_own_xy, obstacles):
    hx, hy = goal_own_xy
    
    # 计算球到球门的向量
    dx = bx - hx
    dy = by - hy
    dist = norm2(dx, dy)
    
    # 1. 计算理想防守距离：不再锁死 1.8米
    # 策略：总是站在球和门之间，靠近球一点（比如距离球门 60% 的位置）进行拦截
    # 但如果球太远，就不要跟过去了，最远只防守到中场附近（距离球门 3.5米）
    target_dist = min(3.5, dist * 0.6) 
    
    nx, ny = normalize(dx, dy)
    target_x = hx + nx * target_dist
    target_y = hy + ny * target_dist * 0.8 # Y轴收缩逻辑保留，防止跑太偏
    
    # 2. 【关键】强制半场限制 (Hard Constraint)
    # 假设红队门在 -4.5，蓝队在 4.5
    # 如果我是红队(hx < 0)，我的 x 不能大于 -0.5 (留点余量给前锋)
    # 如果我是蓝队(hx > 0)，我的 x 不能小于 0.5
    if hx > 0: # 红队
        target_x = min(target_x, -1.5) # 最远跑到 x=-1.5 处
    else:      # 蓝队
        target_x = max(target_x, 1.5)  # 最远跑到 x=1.5 处

    face_angle = math.atan2(by - target_y, bx - target_x)
    
    # 增加避障，防止和门将撞
    return action_to_target(my_x, my_y, my_theta, target_x, target_y, face_angle, obstacles, True)

# === 支援 (Support) ===
def run_support(my_x, my_y, my_theta, bx, by, goal_target_xy, obstacles):
    gx, gy = goal_target_xy
    dx_att = gx - bx
    dy_att = gy - by
    nx, ny = normalize(dx_att, dy_att)
    target_x = bx - nx * 1.0 - ny * 0.5
    target_y = by - ny * 1.0 + nx * 0.5
    face_angle = math.atan2(by - target_y, bx - target_x)
    return action_to_target(my_x, my_y, my_theta, target_x, target_y, face_angle, obstacles, True)