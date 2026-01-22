import math
from utils import clamp
from movement import action_to_target

def calculate_predicted_y(bx, by, goal_x, ball_history):
    # 简单的线性预测：如果没有历史数据，就直接返回当前y
    if len(ball_history) < 5:
        return by
    # 这里简化处理，直接用当前位置，后续可以加卡尔曼滤波
    return by

def run_goalie(my_x, my_y, my_theta, bx, by, goal_own_xy, ball_history):
    hx, hy = goal_own_xy
    
    # 1. 计算理想防守位置 (预测球的 Y 轴落点)
    pred_y = calculate_predicted_y(bx, by, hx, ball_history)
    
    # 守门员站在球门线前方一点点 (0.35m)
    # 这里的 hx 是传进来的“自家球门”坐标
    # 如果 hx 是 -4.5，base_x 就是 -4.15
    # 如果 hx 是 +4.5，base_x 就是 +4.15
    base_x = hx + (0.35 if hx < 0 else -0.35)
    
    # 2. 限制活动范围 (Clamp)
    # 确保守门员不会跑出小禁区
    if hx > 0: # 如果守门的是左边球门 (-4.5)
        desired_gk_x = clamp(base_x, -4.5, -3.5) # 限制 X 在 [-4.5, -3.5]
        desired_gk_y = clamp(pred_y, -1.0, 1.0)  # 限制 Y 在门宽范围内
    else:      # 如果守门的是右边球门 (+4.5)
        desired_gk_x = clamp(base_x, 3.5, 4.5)   # 限制 X 在 [3.5, 4.5]
        desired_gk_y = clamp(pred_y, -1.0, 1.0)
    
    # 3. 始终面向球
    gk_face = math.atan2(by - desired_gk_y, bx - desired_gk_x)
    
    # 4. 移动指令 (开启侧移 can_strafe=True)
    return action_to_target(my_x, my_y, my_theta, desired_gk_x, desired_gk_y, gk_face, [], 
                            use_avoidance=False, is_dribbling=False, can_strafe=True)