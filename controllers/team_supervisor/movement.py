import math
from utils import norm2, normalize, wrap_pi

# === 避障向量计算 ===
def get_avoidance_heading(my_x, my_y, target_x, target_y, obstacles):
    # 1. 计算引力
    dx_goal = target_x - my_x
    dy_goal = target_y - my_y
    dist_goal = norm2(dx_goal, dy_goal)
    
    if dist_goal < 0.2: 
        return math.atan2(dy_goal, dx_goal)

    attr_x, attr_y = normalize(dx_goal, dy_goal)

    # 2. 计算斥力
    rep_x, rep_y = 0.0, 0.0
    
    AVOID_RADIUS = 0.5    
    REPULSION_FORCE = 0.8 

    for (ox, oy) in obstacles:
        dist_obs = norm2(my_x - ox, my_y - oy)
        if dist_obs < AVOID_RADIUS and dist_obs > 0.01:
            push_x = my_x - ox
            push_y = my_y - oy
            strength = REPULSION_FORCE * (1.0 - dist_obs / AVOID_RADIUS)
            px, py = normalize(push_x, push_y)
            rep_x += px * strength
            rep_y += py * strength

    final_x = attr_x + rep_x
    final_y = attr_y + rep_y
    return math.atan2(final_y, final_x)

# === 生成移动指令 ===
def action_to_target(my_x, my_y, my_theta, tx, ty, face_theta, obstacles, use_avoidance=True):
    dist = norm2(tx - my_x, ty - my_y)

    # 1. 计算目标航向
    if use_avoidance:
        target_heading = get_avoidance_heading(my_x, my_y, tx, ty, obstacles)
    else:
        target_heading = math.atan2(ty - my_y, tx - my_x)

    heading_err = wrap_pi(target_heading - my_theta)

    # 2. 移动逻辑
    if abs(heading_err) > 0.45: 
        return "TURN_L" if heading_err > 0 else "TURN_R"
    
    if dist > 0.15: 
        return "FWD" 
    
    # 3. 到达位置后的朝向调整
    face_err = wrap_pi(face_theta - my_theta)
    if abs(face_err) > 0.45: 
        return "TURN_L" if face_err > 0 else "TURN_R"
        
    return "STOP"