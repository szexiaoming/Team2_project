from controller import Supervisor
import socket, json, time, math, select

# ================= 配置区 =================
TIME_STEP = 32
SEND_DT = 0.05  # 50ms 发一次指令，保证动作流畅

# UDP 目标端口 (发给球员)
PORT = {
    "B1": 10001, "B2": 10002, "B3": 10003, "B4": 10004,
    "R1": 10011, "R2": 10012, "R3": 10013, "R4": 10014,
}

# Supervisor 监听端口 (收球员的 READY/DONE)
SUPERVISOR_PORT = 12000

# 场景 DEF 名称 (请与 Webots 场景树一致)
BALL_DEF = "BALL"
BLUE_DEFS = ["BLUE1", "BLUE2", "BLUE3", "BLUE4"]
RED_DEFS  = ["RED1", "RED2", "RED3", "RED4"]
GOAL_RED_DEF = "GOAL_BLUE_CENTER" # 蓝门中心 (x = 4.5)
GOAL_BLUE_DEF  = "GOAL_RED_CENTER"  # 红门中心 (x = -4.5)

# ================= 工具函数 =================
def norm2(dx, dy): return math.sqrt(dx*dx + dy*dy)
def clamp(v, lo, hi): return max(lo, min(hi, v))
def wrap_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a
def normalize(dx, dy):
    n = norm2(dx, dy)
    if n < 1e-9: return 0.0, 0.0
    return dx/n, dy/n

def heading_of_robot(node):
    o = node.getOrientation()
    return math.atan2(o[3], o[0])

def get_axes(node):
    o = node.getOrientation()
    x_axis = (o[0], o[3], o[6])
    z_axis = (o[2], o[5], o[8])
    return x_axis, z_axis

# ================= 主类 =================
class TeamSupervisor:
    def __init__(self):
        self.robot = Supervisor()

        # 发送 Socket
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_send = 0.0

        # 接收 Socket
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(("127.0.0.1", SUPERVISOR_PORT))
        self.sock_rx.setblocking(False)

        # 获取节点
        self.ball = self.robot.getFromDef(BALL_DEF)
        self.blue_nodes = [self.robot.getFromDef(d) for d in BLUE_DEFS]
        self.red_nodes  = [self.robot.getFromDef(d) for d in RED_DEFS]
        self.goal_blue = self.robot.getFromDef(GOAL_BLUE_DEF)
        self.goal_red  = self.robot.getFromDef(GOAL_RED_DEF)

        # 动态检测场上激活的机器人 ID
        self.blue_ids = []
        self.red_ids = []
        
        # 只要节点存在且在 PORT 表里，就加入名单
        for i, node in enumerate(self.blue_nodes):
            if node: self.blue_ids.append(f"B{i+1}")
        for i, node in enumerate(self.red_nodes):
            if node: self.red_ids.append(f"R{i+1}")

        print(f"Active Players: Blue={self.blue_ids}, Red={self.red_ids}")

        self.all_ids = self.blue_ids + self.red_ids
        self.busy = {rid: False for rid in self.all_ids}
        self.last_sent_cmd = {rid: "STOP" for rid in self.all_ids}

        # 摔倒检测参数
        self.fall_count = {rid: 0 for rid in self.all_ids}
        self.fall_needed = 6
        self.fall_up_th = 0.60
        self.recovering = {rid: False for rid in self.all_ids}

        # 握手状态
        self.ready = {rid: False for rid in self.all_ids}
        self.handshake_done = False

    def xy(self, node):
        p = node.getPosition()
        return float(p[0]), float(p[1])

    def send_cmd(self, rid, cmd):
        self.seq += 1
        msg = {"seq": self.seq, "id": rid, "cmd": cmd}
        self.sock_tx.sendto(json.dumps(msg).encode("utf-8"), ("127.0.0.1", PORT[rid]))
        self.last_sent_cmd[rid] = cmd
        self.busy[rid] = (cmd != "STOP")

    def poll_events(self):
        while True:
            r, _, _ = select.select([self.sock_rx], [], [], 0)
            if not r: break
            try:
                data, _ = self.sock_rx.recvfrom(4096)
                msg = json.loads(data.decode("utf-8"))
                rid = msg.get("id", "")
                event = msg.get("event", "")
                action = msg.get("action", "")
                
                if rid in self.busy:
                    if event == "DONE":
                        self.busy[rid] = False
                        if action in ("GETUP_FRONT", "GETUP_BACK"):
                            self.recovering[rid] = False
                    elif event == "READY":
                        self.busy[rid] = False
                        self.ready[rid] = True
            except Exception:
                break

    def do_handshake_if_needed(self):
        if self.handshake_done: return True
        if not all(self.ready.values()): return False
        
        # 广播 STOP 确保大家都收到
        for _ in range(2):
            for rid in self.all_ids:
                self.send_cmd(rid, "STOP")
            for _k in range(2):
                self.robot.step(TIME_STEP)
                self.poll_events()
        
        self.handshake_done = True
        print("HANDSHAKE DONE: All players ready.")
        return True

    def detect_fall_and_getup_cmd(self, node, rid):
        x_axis, z_axis = get_axes(node)
        up_z = z_axis[2]
        front_z = x_axis[2]

        if up_z < self.fall_up_th:
            self.fall_count[rid] += 1
        else:
            self.fall_count[rid] = 0

        if self.fall_count[rid] < self.fall_needed: return None
        if self.recovering[rid]: return None

        self.recovering[rid] = True
        self.fall_count[rid] = 0
        return "GETUP_FRONT" if front_z < 0 else "GETUP_BACK"

    def assign_roles(self, nodes, goal_own_xy, bx, by):
        """
        角色分配逻辑：
        1. 强制 B4/R4 (Index 3) 为守门员。
        2. 剩下的人里，离球最近的是前锋 Striker。
        """
        active_nodes = [n for n in nodes if n]
        if not active_nodes:
            return -1, -1, -1, -1

        # ========================================================
        # 修改点：指定 Index 3 (B4/R4) 为守门员
        # ========================================================
        target_gk_index = 3 
        
        # 检查 B4/R4 是否存在
        if len(nodes) > target_gk_index and nodes[target_gk_index]:
            gk = target_gk_index
        else:
            # 如果 B4/R4 不存在 (比如没启动)，回退到 B1/R1 (Index 0)
            if nodes[0]: gk = 0
            else: gk = -1 # 没人守门
        
        # 2. 前锋：在剩下的人里选离球最近的
        d_ball = []
        for i in range(len(nodes)): 
            if i == gk: continue # 跳过守门员
            if nodes[i]:
                x, y = self.xy(nodes[i])
                d_ball.append((norm2(x - bx, y - by), i))
        
        if d_ball:
            striker = sorted(d_ball)[0][1]
        else:
            striker = -1 # 没有其他队员了

        # 3. 其他人
        others = [i for i in range(len(nodes)) if i not in (gk, striker) and nodes[i]]
        while len(others) < 2: others.append(-1)

        return gk, striker, others[0], others[1]

    def action_to_target(self, node, tx, ty, face_theta):
        x, y = self.xy(node)
        th = heading_of_robot(node)

        dx, dy = tx - x, ty - y
        dist = norm2(dx, dy)
        ang_to = math.atan2(dy, dx)
        ang_err = wrap_pi(ang_to - th)

        # 简单的 P 控制
        if abs(ang_err) > 0.45: return "TURN_L" if ang_err > 0 else "TURN_R"
        if dist > 0.15: return "FWD" 
        
        face_err = wrap_pi(face_theta - th)
        if abs(face_err) > 0.45: return "TURN_L" if face_err > 0 else "TURN_R"

        return "STOP"

    def striker_logic_left_kick(self, striker_node, bx, by, goal_target_xy):
        gx, gy = goal_target_xy 
        sx, sy = self.xy(striker_node) 
        th = heading_of_robot(striker_node) 

        # 射门向量
        dx_shot, dy_shot = gx - bx, gy - by
        dist_shot = norm2(dx_shot, dy_shot)
        dir_x, dir_y = normalize(dx_shot, dy_shot)
        desired_theta = math.atan2(dir_y, dir_x)

        # 最佳站位 (球后 22cm, 右偏 5cm)
        DIST_BEHIND = 0.22  
        OFFSET_RIGHT = 0.05 
        stand_x = bx - dir_x * DIST_BEHIND + dir_y * OFFSET_RIGHT
        stand_y = by - dir_y * DIST_BEHIND - dir_x * OFFSET_RIGHT

        # === 智能绕行逻辑 (Orbit) ===
        vec_br_x, vec_br_y = sx - bx, sy - by
        dist_br = norm2(vec_br_x, vec_br_y)
        dot = vec_br_x * dir_x + vec_br_y * dir_y
        
        # 如果机器人在球前方或侧方，且距离较近，执行绕行
        if dot > -0.15 and dist_br < 0.6:
            cross = vec_br_x * dir_y - vec_br_y * dir_x
            orbit_radius = 0.45
            if cross > 0: # 在左侧，往左绕
                nav_x = bx + dir_y * orbit_radius 
                nav_y = by - dir_x * orbit_radius
            else: # 在右侧，往右绕
                nav_x = bx - dir_y * orbit_radius
                nav_y = by + dir_x * orbit_radius
            face_ball = math.atan2(by - sy, bx - sx)
            return self.action_to_target(striker_node, nav_x, nav_y, face_ball)

        # === 射门判定 ===
        dist_to_spot = norm2(sx - stand_x, sy - stand_y)
        angle_diff = abs(wrap_pi(desired_theta - th))
        KICK_DIST_THRESHOLD = 0.06 
        KICK_ANGLE_THRESHOLD = 0.15 

        if dist_to_spot < KICK_DIST_THRESHOLD and angle_diff < KICK_ANGLE_THRESHOLD:
            # 只有机器人在球后面时才踢
            if norm2(sx - gx, sy - gy) > dist_shot:
                return "KICK_L"

        return self.action_to_target(striker_node, stand_x, stand_y, desired_theta)

    def compute_desired_cmds(self, nodes, goal_own_xy, goal_target_xy, bx, by):
        hx, hy = goal_own_xy
        gx, gy = goal_target_xy
        gk, striker, support, defender = self.assign_roles(nodes, goal_own_xy, bx, by)
        cmds = ["STOP"] * len(nodes)

        # === 守门员逻辑 (限制范围) ===
        if gk != -1:
            # 1. 计算理想位置 (门前0.35米，Y跟随球)
            base_gk_x = hx + (0.35 if hx < 0 else -0.35)
            desired_gk_y = clamp(by, hy - 0.8, hy + 0.8)

            # 2. X 轴强制锁死：绝对不能出禁区 (4.0 ~ 4.5)
            if hx > 0: # 蓝门
                desired_gk_x = clamp(base_gk_x, 4.0, 4.5)
            else:      # 红门
                desired_gk_x = clamp(base_gk_x, -4.5, -4.0)

            gk_face = math.atan2(by - desired_gk_y, bx - desired_gk_x)
            
            cmds[gk] = self.action_to_target(nodes[gk], desired_gk_x, desired_gk_y, gk_face)

        if striker != -1:
            cmds[striker] = self.striker_logic_left_kick(nodes[striker], bx, by, goal_target_xy)

        if support != -1:
            dtx, dty = normalize(gx - bx, gy - by)
            perp_x, perp_y = -dty, dtx
            cmds[support] = self.action_to_target(nodes[support], bx-dtx+perp_x*0.6, by-dty+perp_y*0.6, math.atan2(dty, dtx))

        if defender != -1:
            t = 0.55
            cmds[defender] = self.action_to_target(nodes[defender], hx+(bx-hx)*t, hy+(by-hy)*t, math.atan2(by-(hy+(by-hy)*t), bx-(hx+(bx-hx)*t)))

        return cmds

    def run(self):
        # 开场表演逻辑的时间轴
        PHASE_1_STABILIZE = 50  
        PHASE_2_TRIGGER_KICK = PHASE_1_STABILIZE + 10 
        PHASE_3_WAIT_ANIMATION = PHASE_2_TRIGGER_KICK + 80 

        game_steps = 0

        while self.robot.step(TIME_STEP) != -1:
            self.poll_events()
            if not self.do_handshake_if_needed():
                continue

            game_steps += 1

            # ==================================================
            # 开场表演逻辑
            # ==================================================
            if game_steps < PHASE_1_STABILIZE:
                now = time.time()
                if now - self.last_send > 0.2:
                    for rid in self.all_ids:
                        self.send_cmd(rid, "STOP")
                    self.last_send = now
                continue 

            elif game_steps < PHASE_2_TRIGGER_KICK:
                for rid in self.all_ids:
                    self.send_cmd(rid, "KICK_L")
                continue 

            elif game_steps < PHASE_3_WAIT_ANIMATION:
                continue 

            # ==================================================
            # 阶段 4: 比赛正式开始
            # ==================================================
            
            now = time.time()
            if now - self.last_send < SEND_DT:
                continue
            self.last_send = now

            bx, by = self.xy(self.ball)
            blue_goal_xy = self.xy(self.goal_blue)
            red_goal_xy = self.xy(self.goal_red)

            # 计算指令
            blue_cmds = self.compute_desired_cmds(self.blue_nodes, blue_goal_xy, blue_goal_xy, bx, by)
            red_cmds  = self.compute_desired_cmds(self.red_nodes, red_goal_xy, red_goal_xy, bx, by)

            # 发送指令
            for i, rid in enumerate(self.blue_ids):
                getup = self.detect_fall_and_getup_cmd(self.blue_nodes[i], rid)
                if getup: 
                    self.send_cmd(rid, "INTERRUPT_" + getup)
                elif not self.busy[rid]:
                    self.send_cmd(rid, blue_cmds[i])

            for i, rid in enumerate(self.red_ids):
                getup = self.detect_fall_and_getup_cmd(self.red_nodes[i], rid)
                if getup:
                    self.send_cmd(rid, "INTERRUPT_" + getup)
                elif not self.busy[rid]:
                    self.send_cmd(rid, red_cmds[i])

if __name__ == "__main__":
    TeamSupervisor().run()