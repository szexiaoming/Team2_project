from controller import Supervisor, Display
import socket, json, select

# 引入我们的模块
import utils
from strategies import goalie, striker

# ================= 配置区 =================
TIME_STEP = 32

# 端口映射要和 Player 端一致
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
        
        # 接收端口 (用于接收 DONE/READY 信号)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(("127.0.0.1", SUPERVISOR_PORT))
        self.sock_rx.setblocking(False)

        # 获取 Webots 节点
        self.ball = self.robot.getFromDef(BALL_DEF)
        self.blue_nodes = [self.robot.getFromDef(d) for d in BLUE_DEFS]
        self.red_nodes  = [self.robot.getFromDef(d) for d in RED_DEFS]
        self.goal_blue = self.robot.getFromDef(GOAL_BLUE_DEF)
        self.goal_red  = self.robot.getFromDef(GOAL_RED_DEF)

        # 生成 ID 列表
        self.blue_ids = [f"B{i+1}" for i, n in enumerate(self.blue_nodes) if n]
        self.red_ids  = [f"R{i+1}" for i, n in enumerate(self.red_nodes) if n]
        print(f"Active Players: Blue={self.blue_ids}, Red={self.red_ids}")

        self.all_ids = self.blue_ids + self.red_ids
        
        # 状态管理
        self.busy = {rid: False for rid in self.all_ids}
        self.last_sent_cmd = {rid: "STOP" for rid in self.all_ids}
        self.fall_count = {rid: 0 for rid in self.all_ids}
        self.recovering = {rid: False for rid in self.all_ids}
        self.ready = {rid: False for rid in self.all_ids}
        self.handshake_done = False
        
        self.ball_pos_history = [] 

        # 小地图初始化
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
        """发送指令给机器人，处理忙碌锁逻辑"""
        # 判断是否是移动类指令 (可以被覆盖)
        is_move = cmd in ["FWD", "TURN_L", "TURN_R", "STOP", "SIDE_L", "SIDE_R"]
        was_move = self.last_sent_cmd.get(rid, "STOP") in ["FWD", "TURN_L", "TURN_R", "STOP", "SIDE_L", "SIDE_R"]
        
        # 如果机器人正忙(Busy=True)，且不是连续的移动指令，则不发送
        if self.busy[rid] and not (is_move and was_move): return

        self.seq += 1
        msg = {"seq": self.seq, "id": rid, "cmd": cmd}
        self.sock_tx.sendto(json.dumps(msg).encode("utf-8"), ("127.0.0.1", PORT[rid]))
        self.last_sent_cmd[rid] = cmd
        
        # 定义哪些指令会触发忙碌锁 (直到收到 DONE)
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
        """处理机器人返回的事件 (DONE, READY)"""
        while True:
            r, _, _ = select.select([self.sock_rx], [], [], 0)
            if not r: break
            try:
                data, _ = self.sock_rx.recvfrom(4096)
                msg = json.loads(data.decode("utf-8"))
                rid = msg.get("id", "")
                event = msg.get("event")
                
                if event == "DONE":
                    self.busy[rid] = False
                    self.recovering[rid] = False
                elif event == "READY":
                    self.ready[rid] = True
            except: break

    def do_handshake_if_needed(self):
        """等待所有机器人上线 (READY)"""
        if self.handshake_done: return True
        if not all(self.ready.values()): return False
        
        # 握手成功，先发两轮 STOP 确保状态同步
        for _ in range(2):
            for rid in self.all_ids: self.send_cmd(rid, "STOP")
            for _k in range(2):
                self.robot.step(TIME_STEP)
                self.poll_events()
                
        self.handshake_done = True
        print("HANDSHAKE DONE")
        return True

    def check_fall(self, node, rid):
        """检测摔倒方向"""
        _, z_axis = utils.get_axes(node)
        _, front_x_axis = utils.get_axes(node) 
        
        # Z轴过低认为摔倒
        if z_axis[2] < 0.60: self.fall_count[rid] += 1
        else: self.fall_count[rid] = 0

        # 滤波 & 状态检查
        if self.fall_count[rid] < 6 or self.recovering[rid]: return None
        
        self.recovering[rid] = True
        self.fall_count[rid] = 0
        
        # 根据 X 轴判断前后 (如果你的动作反了，在这里交换返回值即可)
        return "GETUP_BACK" if front_x_axis[2] < 0 else "GETUP_FRONT"

    def get_all_positions(self):
        """获取场上所有机器人的位置 (用于避障)"""
        obstacles = []
        for n in self.blue_nodes + self.red_nodes:
            if n: obstacles.append(utils.get_pos(n))
        return obstacles

    def assign_roles_and_compute(self, nodes, goal_own, goal_target, bx, by, obstacles, is_red):
        """分配角色并计算指令 (固定角色分配)"""
        active_nodes = [n for n in nodes if n]
        if not active_nodes: return []
        
        cmds = ["STOP"] * len(nodes)
        
        # === 固定角色配置 ===
        # 索引对应: 0->Player1, 1->Player2, 2->Player3, 3->Player4
        if is_red:
            # 红队: R1=Defender, R2=Support, R3=Striker, R4=Goalie
            idx_def = 0 
            idx_sup = 1 
            idx_str = 2 
            idx_gk  = 3 
        else:
            # 蓝队: B1=Striker, B2=Defender, B3=Support, B4=Goalie
            idx_str = 0 
            idx_def = 1 
            idx_sup = 2 
            idx_gk  = 3 

        # 计算指令
        for i, node in enumerate(nodes):
            if not node: continue
            
            my_x, my_y = utils.get_pos(node)
            my_theta = utils.get_heading(node)
            
            # 障碍物列表排除自己
            my_obstacles = [o for o in obstacles if utils.norm2(o[0]-my_x, o[1]-my_y) > 0.01]

            if i == idx_gk:
                cmds[i] = goalie.run_goalie(my_x, my_y, my_theta, bx, by, goal_own, self.ball_pos_history)
            elif i == idx_str:
                cmds[i] = striker.run_striker(my_x, my_y, my_theta, bx, by, goal_target, my_obstacles)
            elif i == idx_def:
                cmds[i] = striker.run_defender(my_x, my_y, my_theta, bx, by, goal_own, my_obstacles)
            elif i == idx_sup:
                cmds[i] = striker.run_support(my_x, my_y, my_theta, bx, by, goal_target, my_obstacles)
                
        return cmds

    def update_minimap(self, bx, by):
        if not self.display: return

        # 背景
        self.display.setColor(0x006600)
        self.display.fillRectangle(0, 0, self.d_width, self.d_height)
        
        # 线条
        self.display.setColor(0xFFFFFF)
        self.display.drawLine(self.d_width//2, 0, self.d_width//2, self.d_height) 
        self.display.drawOval(self.d_width//2, self.d_height//2, 10, 10) 

        # 禁区
        BOX_DEPTH = 0.6; BOX_WIDTH = 2.2
        x1, y1 = self.world_to_screen(-self.FIELD_LENGTH/2, BOX_WIDTH/2)
        x2, y2 = self.world_to_screen(-self.FIELD_LENGTH/2 + BOX_DEPTH, -BOX_WIDTH/2)
        self.display.drawRectangle(x1, y1, abs(x2-x1), abs(y2-y1))
        x1, y1 = self.world_to_screen(self.FIELD_LENGTH/2 - BOX_DEPTH, BOX_WIDTH/2)
        x2, y2 = self.world_to_screen(self.FIELD_LENGTH/2, -BOX_WIDTH/2)
        self.display.drawRectangle(x1, y1, abs(x2-x1), abs(y2-y1))

        # 蓝队点
        self.display.setColor(0x0000FF)
        for node in self.blue_nodes:
            if node:
                x, y = utils.get_pos(node)
                sx, sy = self.world_to_screen(x, y)
                self.display.fillOval(sx, sy, 4, 4)

        # 红队点
        self.display.setColor(0xFF0000)
        for node in self.red_nodes:
            if node:
                x, y = utils.get_pos(node)
                sx, sy = self.world_to_screen(x, y)
                self.display.fillOval(sx, sy, 4, 4)

        # 足球点
        self.display.setColor(0xFFFF00)
        bsx, bsy = self.world_to_screen(bx, by)
        self.display.fillOval(bsx, bsy, 3, 3)

        # 文字
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

        # === 主循环：去掉了 time.time()，完全依赖 physics step ===
        while self.robot.step(TIME_STEP) != -1:
            self.poll_events()
            if not self.do_handshake_if_needed(): continue

            game_steps += 1
            bx, by = utils.get_pos(self.ball)

            # === 阶段 1: 开场表演 (可注释) ===
            if game_steps < PHASE_1_STABILIZE:
                for rid in self.all_ids: self.send_cmd(rid, "STOP")
                self.update_minimap(bx, by)
                continue 

            elif game_steps < PHASE_2_TRIGGER_KICK:
                for rid in self.all_ids: self.send_cmd(rid, "KICK_L")
                self.update_minimap(bx, by)
                continue 

            elif game_steps < PHASE_3_WAIT_ANIMATION:
                self.update_minimap(bx, by)
                continue 
            # ==============================
            
            # === 阶段 2: 比赛逻辑 ===
            
            # 计分板重置
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

            # 获取位置信息
            blue_goal = utils.get_pos(self.goal_blue)
            red_goal = utils.get_pos(self.goal_red)
            all_obstacles = self.get_all_positions()

            # 计算指令 (注意 goal_target 的传参)
            # 蓝队: Own=blue_goal(+4.5), Target=red_goal(-4.5)
            blue_cmds = self.assign_roles_and_compute(
                self.blue_nodes, blue_goal, red_goal, bx, by, all_obstacles, is_red=False)
            
            # 红队: Own=red_goal(-4.5), Target=blue_goal(+4.5)
            red_cmds = self.assign_roles_and_compute(
                self.red_nodes, red_goal, blue_goal, bx, by, all_obstacles, is_red=True)

            # === 发送蓝队指令 ===
            for i, rid in enumerate(self.blue_ids):
                # 1. 检测摔倒
                fall_cmd = self.check_fall(self.blue_nodes[i], rid)
                if fall_cmd: 
                    self.send_cmd(rid, "INTERRUPT_" + fall_cmd)
                elif self.recovering[rid]:
                    continue # 正在起身，跳过
                else: 
                    self.send_cmd(rid, blue_cmds[i])

            # === 发送红队指令 ===
            for i, rid in enumerate(self.red_ids):
                fall_cmd = self.check_fall(self.red_nodes[i], rid)
                if fall_cmd: 
                    self.send_cmd(rid, "INTERRUPT_" + fall_cmd)
                elif self.recovering[rid]:
                    continue
                else: 
                    self.send_cmd(rid, red_cmds[i])
            
            self.update_minimap(bx, by)

if __name__ == "__main__":
    TeamSupervisor().run()