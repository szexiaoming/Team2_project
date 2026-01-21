from controller import Supervisor, Display
import socket, json, time, select

# 引入我们的模块
import utils
from strategies import goalie, striker

# ================= 配置区 =================
TIME_STEP = 32
PORT = {
    "B1": 10001, "B2": 10002, "B3": 10003, "B4": 10004,
    "R1": 10011, "R2": 10012, "R3": 10013, "R4": 10014,
}
SUPERVISOR_PORT = 12000

BALL_DEF = "BALL"
BLUE_DEFS = ["BLUE1", "BLUE2", "BLUE3", "BLUE4"]
RED_DEFS  = ["RED1", "RED2", "RED3", "RED4"]

GOAL_RED_DEF  = "GOAL_RED_CENTER"
GOAL_BLUE_DEF = "GOAL_BLUE_CENTER"

# ================= 主类 =================
class TeamSupervisor:
    def __init__(self):
        self.robot = Supervisor()
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_send = 0.0

        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(("127.0.0.1", SUPERVISOR_PORT))
        self.sock_rx.setblocking(False)

        self.ball = self.robot.getFromDef(BALL_DEF)
        self.blue_nodes = [self.robot.getFromDef(d) for d in BLUE_DEFS]
        self.red_nodes  = [self.robot.getFromDef(d) for d in RED_DEFS]
        self.goal_blue = self.robot.getFromDef(GOAL_BLUE_DEF)
        self.goal_red  = self.robot.getFromDef(GOAL_RED_DEF)

        self.blue_ids = [f"B{i+1}" for i, n in enumerate(self.blue_nodes) if n]
        self.red_ids  = [f"R{i+1}" for i, n in enumerate(self.red_nodes) if n]
        print(f"Active Players: Blue={self.blue_ids}, Red={self.red_ids}")

        self.all_ids = self.blue_ids + self.red_ids
        self.busy = {rid: False for rid in self.all_ids}
        self.last_sent_cmd = {rid: "STOP" for rid in self.all_ids}
        self.fall_count = {rid: 0 for rid in self.all_ids}
        self.recovering = {rid: False for rid in self.all_ids}
        self.ready = {rid: False for rid in self.all_ids}
        self.handshake_done = False
        
        self.ball_pos_history = [] 

        # 小地图
        self.display = self.robot.getDevice("minimap")
        self.FIELD_LENGTH = 9.0
        self.FIELD_WIDTH = 6.0
        self.score_blue = 0
        self.score_red = 0
        if self.display:
            self.d_width = self.display.getWidth()
            self.d_height = self.display.getHeight()
            self.display.setFont("Arial", 12, True)

    def send_cmd(self, rid, cmd):
        is_move = cmd in ["FWD", "TURN_L", "TURN_R", "STOP"]
        was_move = self.last_sent_cmd.get(rid, "STOP") in ["FWD", "TURN_L", "TURN_R", "STOP"]
        
        # 如果正忙，且不是“移动切移动”，则跳过
        if self.busy[rid] and not (is_move and was_move): return

        self.seq += 1
        msg = {"seq": self.seq, "id": rid, "cmd": cmd}
        self.sock_tx.sendto(json.dumps(msg).encode("utf-8"), ("127.0.0.1", PORT[rid]))
        self.last_sent_cmd[rid] = cmd
        
        # 【关键修复】把 INTERRUPT 指令也加入忙碌列表
        # 只要是这些指令，就锁住机器人，直到收到 DONE，防止被 STOP 打断
        busy_cmds = [
            "KICK_L", 
            "GETUP_FRONT", "GETUP_BACK", 
            "INTERRUPT_GETUP_FRONT", "INTERRUPT_GETUP_BACK"
        ]
        
        if cmd in busy_cmds:
            self.busy[rid] = True
        else:
            self.busy[rid] = False

    def poll_events(self):
        while True:
            r, _, _ = select.select([self.sock_rx], [], [], 0)
            if not r: break
            try:
                data, _ = self.sock_rx.recvfrom(4096)
                msg = json.loads(data.decode("utf-8"))
                rid = msg.get("id", "")
                if msg.get("event") == "DONE":
                    self.busy[rid] = False
                    self.recovering[rid] = False
                elif msg.get("event") == "READY":
                    self.ready[rid] = True
            except: break

    def do_handshake_if_needed(self):
        if self.handshake_done: return True
        if not all(self.ready.values()): return False
        
        # 握手逻辑：先全部发STOP，确保大家都停下
        for _ in range(2):
            for rid in self.all_ids: self.send_cmd(rid, "STOP")
            for _k in range(2):
                self.robot.step(TIME_STEP)
                self.poll_events()
                
        self.handshake_done = True
        print("HANDSHAKE DONE")
        return True

    def check_fall(self, node, rid):
        _, z_axis = utils.get_axes(node)
        _, front_x_axis = utils.get_axes(node) 
        
        if z_axis[2] < 0.60: self.fall_count[rid] += 1
        else: self.fall_count[rid] = 0

        # 如果还没有确定摔倒(count < 6) 或者 已经在起身过程中(recovering=True)，就返回 None
        if self.fall_count[rid] < 6 or self.recovering[rid]: return None
        
        self.recovering[rid] = True
        self.fall_count[rid] = 0
        return "GETUP_BACK" if front_x_axis[2] < 0 else "GETUP_FRONT"

    def get_all_positions(self):
        obstacles = []
        for n in self.blue_nodes + self.red_nodes:
            if n: obstacles.append(utils.get_pos(n))
        return obstacles

    def assign_roles_and_compute(self, nodes, goal_own, goal_target, bx, by, obstacles):
        active_nodes = [n for n in nodes if n]
        if not active_nodes: return []
        
        cmds = ["STOP"] * len(nodes)
        
        # 1. 强制分配守门员
        gk_idx = 3 if len(nodes) > 3 else 0
        
        # 2. 其他人按距离分配
        d_ball = []
        for i, node in enumerate(nodes):
            if i == gk_idx or not node: continue
            x, y = utils.get_pos(node)
            d_ball.append((utils.norm2(x-bx, y-by), i))
        d_ball.sort()

        striker_idx = d_ball[0][1] if len(d_ball) > 0 else -1
        support_idx = d_ball[1][1] if len(d_ball) > 1 else -1
        defender_idx = d_ball[2][1] if len(d_ball) > 2 else -1

        # 3. 计算指令
        for i, node in enumerate(nodes):
            if not node: continue
            
            my_x, my_y = utils.get_pos(node)
            my_theta = utils.get_heading(node)
            
            # 排除自己
            my_obstacles = [o for o in obstacles if utils.norm2(o[0]-my_x, o[1]-my_y) > 0.01]

            if i == gk_idx:
                cmds[i] = goalie.run_goalie(my_x, my_y, my_theta, bx, by, goal_own, self.ball_pos_history)
            elif i == striker_idx:
                cmds[i] = striker.run_striker(my_x, my_y, my_theta, bx, by, goal_target, my_obstacles)
            elif i == defender_idx:
                cmds[i] = striker.run_defender(my_x, my_y, my_theta, bx, by, goal_own, my_obstacles)
            elif i == support_idx:
                cmds[i] = striker.run_support(my_x, my_y, my_theta, bx, by, goal_target, my_obstacles)
                
        return cmds

    def update_minimap(self, bx, by):
        if not self.display: return

        self.display.setColor(0x006600)
        self.display.fillRectangle(0, 0, self.d_width, self.d_height)
        self.display.setColor(0xFFFFFF)
        self.display.drawLine(self.d_width//2, 0, self.d_width//2, self.d_height) 
        self.display.drawOval(self.d_width//2, self.d_height//2, 10, 10) 

        BOX_DEPTH = 0.6; BOX_WIDTH = 2.2
        x1, y1 = self.world_to_screen(-self.FIELD_LENGTH/2, BOX_WIDTH/2)
        x2, y2 = self.world_to_screen(-self.FIELD_LENGTH/2 + BOX_DEPTH, -BOX_WIDTH/2)
        self.display.drawRectangle(x1, y1, abs(x2-x1), abs(y2-y1))
        x1, y1 = self.world_to_screen(self.FIELD_LENGTH/2 - BOX_DEPTH, BOX_WIDTH/2)
        x2, y2 = self.world_to_screen(self.FIELD_LENGTH/2, -BOX_WIDTH/2)
        self.display.drawRectangle(x1, y1, abs(x2-x1), abs(y2-y1))

        self.display.setColor(0x0000FF)
        for node in self.blue_nodes:
            if node:
                x, y = utils.get_pos(node)
                sx, sy = self.world_to_screen(x, y)
                self.display.fillOval(sx, sy, 4, 4)

        self.display.setColor(0xFF0000)
        for node in self.red_nodes:
            if node:
                x, y = utils.get_pos(node)
                sx, sy = self.world_to_screen(x, y)
                self.display.fillOval(sx, sy, 4, 4)

        self.display.setColor(0xFFFF00)
        bsx, bsy = self.world_to_screen(bx, by)
        self.display.fillOval(bsx, bsy, 3, 3)

        self.display.setColor(0x000000) 
        self.display.fillRectangle(0, 0, self.d_width, 20)
        current_time = int(self.robot.getTime())
        score_str = f"Blue {self.score_blue} : {self.score_red} Red"
        time_str = f"Time: {current_time}s"
        self.display.setColor(0xFFFFFF) 
        self.display.drawText(score_str, 10, 3) 
        self.display.drawText(time_str, self.d_width - 80, 3)

    def world_to_screen(self, wx, wy):
        sx = int((wx + self.FIELD_LENGTH/2) / self.FIELD_LENGTH * self.d_width)
        sy = int((-wy + self.FIELD_WIDTH/2) / self.FIELD_WIDTH * self.d_height)
        return max(0, min(self.d_width, sx)), max(0, min(self.d_height, sy))

    def run(self):
        PHASE_1_STABILIZE = 50 
        PHASE_2_TRIGGER_KICK = PHASE_1_STABILIZE + 10 
        PHASE_3_WAIT_ANIMATION = PHASE_2_TRIGGER_KICK + 80 
        game_steps = 0

        while self.robot.step(TIME_STEP) != -1:
            self.poll_events()
            if not self.do_handshake_if_needed(): continue

            game_steps += 1
            
            # === 开场表演 (可选) ===
            if game_steps < PHASE_1_STABILIZE:
                now = time.time()
                if now - self.last_send > 0.2:
                    for rid in self.all_ids: self.send_cmd(rid, "STOP")
                    self.last_send = now
                bx, by = utils.get_pos(self.ball)
                self.update_minimap(bx, by)
                continue 

            elif game_steps < PHASE_2_TRIGGER_KICK:
                for rid in self.all_ids: self.send_cmd(rid, "KICK_L")
                bx, by = utils.get_pos(self.ball)
                self.update_minimap(bx, by)
                continue 

            elif game_steps < PHASE_3_WAIT_ANIMATION:
                bx, by = utils.get_pos(self.ball)
                self.update_minimap(bx, by)
                continue 
            # ==========================================
            
            now = time.time()
            if now - self.last_send < 0.02: continue 
            self.last_send = now

            bx, by = utils.get_pos(self.ball)
            
            if bx > 4.5:
                self.score_red += 1
                self.ball.getField("translation").setSFVec3f([0, 0, 0.1])
                self.ball.resetPhysics()
                bx, by = 0, 0
            elif bx < -4.5:
                self.score_blue += 1
                self.ball.getField("translation").setSFVec3f([0, 0, 0.1])
                self.ball.resetPhysics()
                bx, by = 0, 0

            blue_goal = utils.get_pos(self.goal_blue)
            red_goal = utils.get_pos(self.goal_red)
            all_obstacles = self.get_all_positions()

            blue_cmds = self.assign_roles_and_compute(self.blue_nodes, blue_goal, blue_goal, bx, by, all_obstacles)
            red_cmds = self.assign_roles_and_compute(self.red_nodes, red_goal, red_goal, bx, by, all_obstacles)

            # === 蓝队发送循环 ===
            for i, rid in enumerate(self.blue_ids):
                # 1. 检查摔倒
                fall_cmd = self.check_fall(self.blue_nodes[i], rid)
                
                if fall_cmd: 
                    self.send_cmd(rid, "INTERRUPT_" + fall_cmd)
                elif self.recovering[rid]:
                    # 【关键修复】如果正在起身，直接跳过，不要发送 STOP 或其他战术指令
                    continue
                else: 
                    self.send_cmd(rid, blue_cmds[i])

            # === 红队发送循环 ===
            for i, rid in enumerate(self.red_ids):
                fall_cmd = self.check_fall(self.red_nodes[i], rid)
                
                if fall_cmd: 
                    self.send_cmd(rid, "INTERRUPT_" + fall_cmd)
                elif self.recovering[rid]:
                    continue # 同样跳过
                else: 
                    self.send_cmd(rid, red_cmds[i])
            
            self.update_minimap(bx, by)

if __name__ == "__main__":
    TeamSupervisor().run()