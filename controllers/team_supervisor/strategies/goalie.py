import math
from utils import clamp
from movement import action_to_target

def calculate_predicted_y(bx, by, goal_x_line, ball_history):
    """球的轨迹预测"""
    current_pos = (bx, by)
    
    # 简单的历史记录维护
    if not ball_history or ball_history[-1] != current_pos:
        ball_history.append(current_pos)
        if len(ball_history) > 3:
            ball_history.pop(0)

    if len(ball_history) < 2: return by

    (x1, y1) = ball_history[0]
    (x2, y2) = ball_history[-1]
    dx_ball = x2 - x1
    dy_ball = y2 - y1

    moving_towards_goal = False
    if goal_x_line < 0: # 目标是负半场
        if dx_ball < -0.005: moving_towards_goal = True
    else: 
        if dx_ball > 0.005: moving_towards_goal = True

    if moving_towards_goal and abs(dx_ball) > 0.001:
        slope = dy_ball / dx_ball
        predicted_y = y2 + slope * (goal_x_line - x2)
        return max(-0.8, min(0.8, predicted_y))
    return by

def run_goalie(my_x, my_y, my_theta, bx, by, goal_own_xy, ball_history):
    """
    守门员主逻辑
    注意：守门员不使用 obstacles，因为它不需要避障，且在门前容易被门柱干扰
    """
    hx, hy = goal_own_xy
    pred_y = calculate_predicted_y(bx, by, hx, ball_history)
    base_x = hx + (0.35 if hx < 0 else -0.35)
    
    if hx > 0: # 红队
        desired_gk_x = clamp(base_x, -4.5, -3.5)
        desired_gk_y = clamp(pred_y, -1.5, 1.5)
    else: # 蓝队
        desired_gk_x = clamp(base_x, 3.5, 4.5)
        desired_gk_y = clamp(pred_y, -1.5, 1.5)
    
    gk_face = math.atan2(by - desired_gk_y, bx - desired_gk_x)
    
    # use_avoidance=False, obstacles=[]
    return action_to_target(my_x, my_y, my_theta, desired_gk_x, desired_gk_y, gk_face, [], False)