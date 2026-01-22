import math
from utils import norm2, normalize, wrap_pi

# === 避障向量计算 (保持不变) ===
def get_avoidance_heading(my_x, my_y, target_x, target_y, obstacles):
    dx_goal = target_x - my_x
    dy_goal = target_y - my_y
    dist_goal = norm2(dx_goal, dy_goal)
    
    if dist_goal < 0.2: 
        return math.atan2(dy_goal, dx_goal)

    attr_x, attr_y = normalize(dx_goal, dy_goal)

    # 计算斥力
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

# === 生成移动指令 (调整了优先级顺序) ===
def action_to_target(my_x, my_y, my_theta, tx, ty, face_theta, obstacles, use_avoidance=True, is_dribbling=False, can_strafe=False):
    dist = norm2(tx - my_x, ty - my_y)

    # 1. 计算目标航向
    if use_avoidance:
        target_heading = get_avoidance_heading(my_x, my_y, tx, ty, obstacles)
    else:
        target_heading = math.atan2(ty - my_y, tx - my_x)

    heading_err = wrap_pi(target_heading - my_theta)

    # === 计算局部坐标 (Local X, Local Y) ===
    dx_world = tx - my_x
    dy_world = ty - my_y
    
    sin_t = math.sin(my_theta)
    cos_t = math.cos(my_theta)
    
    # 旋转矩阵公式
    local_x = dx_world * cos_t + dy_world * sin_t
    local_y = -dx_world * sin_t + dy_world * cos_t
    
    angle_threshold = 0.20 if is_dribbling else 0.60

    # ==========================
    #      优先级调整区
    # ==========================

    # 优先级 1: 转向逻辑 (Heading)
    # 必须最先判断：如果脸都没对准，往前走或者侧移都没意义，容易走偏
    if abs(heading_err) > angle_threshold: 
        return "TURN_L" if heading_err > 0 else "TURN_R"
    
    # 优先级 2: 前进逻辑 (Forward)
    # 先大步流星走到目标附近。只要距离大于 15cm，就优先直走。
    if dist > 0.15: 
        return "FWD" 

    # 优先级 3: 侧移逻辑 (Strafing)
    # 代码走到这里，说明 dist <= 0.15 (已经停在目标附近了)
    # 或者是守门员那种一直在微调的情况
    STRAFE_LIMIT_DIST = 0.4 
    
    if can_strafe and dist < STRAFE_LIMIT_DIST:
        # 此时已经很近了，如果发现左右还没对准 (偏差 > 5cm)，再用侧移微调
        if local_y > 0.05:
            return "SIDE_L"
        if local_y < -0.05:
            return "SIDE_R"

    # 优先级 4: 原地调整朝向 (Facing)
    # 最后一步：位置都对准了，调整身体朝向（例如看向球）
    face_err = wrap_pi(face_theta - my_theta)
    if abs(face_err) > angle_threshold: 
        return "TURN_L" if face_err > 0 else "TURN_R"
        
    return "STOP"