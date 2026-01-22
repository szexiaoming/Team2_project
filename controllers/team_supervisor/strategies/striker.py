import math
from utils import norm2, normalize, wrap_pi
from movement import action_to_target

# === 辅助函数：检查射门路线上是否有障碍 ===
def is_shot_blocked(bx, by, tx, ty, obstacles):
    vx, vy = tx - bx, ty - by
    dist_shot = norm2(vx, vy)
    if dist_shot < 0.01: return False

    for (ox, oy) in obstacles:
        vox, voy = ox - bx, oy - by
        projection = (vox * vx + voy * vy) / (dist_shot * dist_shot)
        if 0.0 < projection < 1.0:
            closest_x = bx + projection * vx
            closest_y = by + projection * vy
            dist_vertical = norm2(ox - closest_x, oy - closest_y)
            if dist_vertical < 0.35:
                return True
    return False

# === 前锋 (Striker) ===
def run_striker(my_x, my_y, my_theta, bx, by, goal_target_xy, obstacles):
    gx, gy = goal_target_xy 
    
    # --- 0. 智能射门点选择 ---
    target_shoot_x, target_shoot_y = gx, gy
    if is_shot_blocked(bx, by, gx, gy, obstacles):
        target_left_y = gy + 0.6
        target_right_y = gy - 0.6
        blocked_left = is_shot_blocked(bx, by, gx, target_left_y, obstacles)
        blocked_right = is_shot_blocked(bx, by, gx, target_right_y, obstacles)
        
        if not blocked_left and blocked_right: target_shoot_y = target_left_y 
        elif blocked_left and not blocked_right: target_shoot_y = target_right_y
        elif not blocked_left and not blocked_right: target_shoot_y = target_left_y

    # --- 1. 计算站位 (左脚踢球逻辑) ---
    dx_shot, dy_shot = target_shoot_x - bx, target_shoot_y - by
    dir_x, dir_y = normalize(dx_shot, dy_shot)
    desired_theta = math.atan2(dir_y, dir_x)

    USING_LEFT_FOOT = True
    DIST_BEHIND = 0.25 
    
    if USING_LEFT_FOOT:
        OFFSET_SIDE = 0.05  # 球在左脚 -> 人在球右侧
        KICK_CMD = "KICK_L"
    else:
        OFFSET_SIDE = -0.05
        KICK_CMD = "KICK_R"

    stand_x = bx - dir_x * DIST_BEHIND + dir_y * OFFSET_SIDE
    stand_y = by - dir_y * DIST_BEHIND - dir_x * OFFSET_SIDE

    # --- 2. 控球决策 ---
    dist_to_ball = norm2(my_x - bx, my_y - by)
    heading_to_target = math.atan2(target_shoot_y - my_y, target_shoot_x - my_x)
    heading_err_target = abs(wrap_pi(heading_to_target - my_theta)) 
    heading_to_ball = math.atan2(by - my_y, bx - my_x)
    heading_err_ball = abs(wrap_pi(heading_to_ball - my_theta))

    if dist_to_ball < 0.30 and heading_err_target < 0.8 and heading_err_ball < 0.8:
        dist_ball_to_goal = norm2(bx - gx, by - gy)
        SHOOTING_RANGE = 1.2  
        
        if dist_ball_to_goal < SHOOTING_RANGE:
            dist_me_goal = norm2(my_x - gx, my_y - gy)
            if dist_me_goal > dist_ball_to_goal: 
                return KICK_CMD
        else:
            # 带球跑：开启 is_dribbling=True
            return action_to_target(my_x, my_y, my_theta, target_shoot_x, target_shoot_y, desired_theta, obstacles, 
                                    use_avoidance=True, is_dribbling=True, can_strafe=False) # 带球时不建议侧移，容易丢球

    # --- 3. 绕行逻辑 ---
    vec_br_x, vec_br_y = my_x - bx, my_y - by
    dist_br = norm2(vec_br_x, vec_br_y)
    dot = vec_br_x * dir_x + vec_br_y * dir_y
    
    if dot > -0.15 and dist_br < 0.5:
        cross = vec_br_x * dir_y - vec_br_y * dir_x
        orbit_radius = 0.35
        if cross > 0:
            nav_x = bx + dir_y * orbit_radius 
            nav_y = by - dir_x * orbit_radius
        else:
            nav_x = bx - dir_y * orbit_radius
            nav_y = by + dir_x * orbit_radius
        face_ball = math.atan2(by - my_y, bx - my_x)
        return action_to_target(my_x, my_y, my_theta, nav_x, nav_y, face_ball, obstacles, True)

    # --- 4. 正常跑位 (Approach) ---
    # 【关键修改】开启 can_strafe=True
    # 当机器人跑到射门点附近进行微调时，允许左右横移
    return action_to_target(my_x, my_y, my_theta, stand_x, stand_y, desired_theta, obstacles, 
                            use_avoidance=True, is_dribbling=False, can_strafe=True)


# === 后卫 (Defender) ===
def run_defender(my_x, my_y, my_theta, bx, by, goal_own_xy, obstacles):
    hx, hy = goal_own_xy
    dx = bx - hx
    dy = by - hy
    dist = norm2(dx, dy)
    
    target_dist = min(3, dist * 0.6) 
    nx, ny = normalize(dx, dy)
    target_x = hx + nx * target_dist
    target_y = hy + ny * target_dist * 0.8 
    
    if hx > 0: # 红队
        if target_x > -1: target_x = -1
    else:      # 蓝队
        if target_x < 1: target_x = 1

    face_angle = math.atan2(by - target_y, bx - target_x)
    
    # 【关键修改】开启 can_strafe=True
    # 后卫在封堵位置微调时，也允许侧移
    return action_to_target(my_x, my_y, my_theta, target_x, target_y, face_angle, obstacles, 
                            use_avoidance=True, is_dribbling=False, can_strafe=True)


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