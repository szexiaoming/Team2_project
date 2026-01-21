from controller import Robot, Motion
import socket, json, select, sys

# ================= 配置区 =================
TIME_STEP = 32

# 必须与 TeamSupervisor 中的端口对应
PORT_MAP = {
    "B1": 10001, "B2": 10002, "B3": 10003, "B4": 10004,
    "R1": 10011, "R2": 10012, "R3": 10013, "R4": 10014,
}

SUPERVISOR_HOST = "127.0.0.1"
SUPERVISOR_PORT = 12000

# 动作文件路径 (根据你之前的代码，你确认有这些文件)
MOTION_PATH_PREFIX = "motions/" 

def safe_get_duration(m: Motion, default_sec: float) -> float:
    try:
        d = m.getDuration()
        if d and d > 0.01: return float(d)
    except: pass
    return float(default_sec)

class GoalkeeperClient:
    def __init__(self):
        # 1. 初始化 Robot (不再是 Supervisor)
        self.robot = Robot()
        
        # 2. 获取 ID (通过 controllerArgs，例如 "B4" 或 "R4")
        self.rid = sys.argv[1] if len(sys.argv) > 1 else "B4"
        if self.rid not in PORT_MAP:
            print(f"Warning: Unknown ID {self.rid}, fallback to B4")
            self.rid = "B4"

        # 3. UDP 网络初始化 (监听裁判指令)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(("127.0.0.1", PORT_MAP[self.rid]))
        self.sock_rx.setblocking(False)
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"[{self.rid}] Goalkeeper Client listening on port {PORT_MAP[self.rid]}")

        # 4. 加载动作 (只加载你确认有的)
        # 即使 Supervisor 发了 DIVE，如果我们没有文件，这里不加载就不会报错(只是不执行)
        self.motion = {}
        self.load_motion_safe("FWD", "Forwards50.motion") # 对应 Supervisor 的 "FWD"
        self.load_motion_safe("forward", "Forwards50.motion") # 兼容旧名
        
        self.load_motion_safe("BWD", "Backwards.motion")
        self.load_motion_safe("backward", "Backwards.motion")

        self.load_motion_safe("TURN_L", "TurnLeftSmall.motion")
        self.load_motion_safe("turn_left", "TurnLeftSmall.motion")

        self.load_motion_safe("TURN_R", "TurnRightSmall.motion")
        self.load_motion_safe("turn_right", "TurnRightSmall.motion")

        self.load_motion_safe("SIDE_L", "SidestepLeft.motion")
        self.load_motion_safe("strafe_left", "SidestepLeft.motion")

        self.load_motion_safe("SIDE_R", "SidestepRight.motion")
        self.load_motion_safe("strafe_right", "SidestepRight.motion")
        
        # 删除了 Stand, DiveLeft, DiveRight, GetUp 等缺失的动作
        
        # 定义哪些动作可以循环播放 (移动类)
        self.continuous_motions = ["FWD", "BWD", "TURN_L", "TURN_R", "SIDE_L", "SIDE_R", 
                                   "forward", "backward", "turn_left", "turn_right", "strafe_left", "strafe_right"]

        self.latest_seq = -1
        self.pending_cmd = None
        self.current_action = None
        self.action_end_time = 0.0

        # 告诉 Supervisor 我准备好了
        self.send_event("READY", action="")

    def load_motion_safe(self, key, filename):
        try:
            # 尝试加载
            self.motion[key] = Motion(MOTION_PATH_PREFIX + filename)
        except Exception:
            # 如果加载失败(文件不存在)，静默跳过，不报错
            pass

    def send_event(self, event, action=""):
        msg = {"id": self.rid, "event": event, "action": action, "t": self.robot.getTime()}
        try: self.sock_tx.sendto(json.dumps(msg).encode(), (SUPERVISOR_HOST, SUPERVISOR_PORT))
        except: pass

    def poll_cmd(self):
        """接收 UDP 指令"""
        r, _, _ = select.select([self.sock_rx], [], [], 0)
        if not r: return None
        try:
            data, _ = self.sock_rx.recvfrom(4096)
            msg = json.loads(data.decode())
            if msg.get("seq", -1) <= self.latest_seq: return None
            self.latest_seq = msg["seq"]
            return msg
        except: return None

    def start_action(self, cmd):
        now = self.robot.getTime()

        # 1. 连续动作续命逻辑
        if cmd in self.continuous_motions and self.current_action == cmd:
            if cmd in self.motion: self.motion[cmd].setLoop(True)
            self.action_end_time = now + 0.5 
            return

        # 2. 停止当前动作
        if self.current_action and self.current_action in self.motion:
            self.motion[self.current_action].stop()

        # 3. 处理 STOP 指令
        if cmd == "STOP":
            self.current_action = None
            self.action_end_time = now + 0.5
            self.send_event("DONE", action="STOP")
            return

        # 4. 检查是否有该动作
        if cmd not in self.motion:
            # 如果 Supervisor 发了 "DIVE_L" 但我们没有加载到 motion，就当 STOP 处理
            # print(f"Ignored unknown command: {cmd}") 
            self.current_action = None
            self.send_event("DONE", action="UNKNOWN")
            return

        # 5. 执行新动作
        self.current_action = cmd
        m = self.motion[cmd]
        
        loop = (cmd in self.continuous_motions)
        m.setLoop(loop)
        m.play()

        dur = safe_get_duration(m, 1.0)
        self.action_end_time = now + (0.5 if loop else dur + 0.05)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            # 1. 读取指令
            msg = self.poll_cmd()
            if msg:
                cmd = msg.get("cmd", "STOP")
                # 处理打断指令
                if cmd.startswith("INTERRUPT_"):
                    real_cmd = cmd.replace("INTERRUPT_", "")
                    self.pending_cmd = None
                    if self.current_action and self.current_action in self.motion:
                        self.motion[self.current_action].stop()
                    self.start_action(real_cmd)
                    continue
                self.pending_cmd = cmd

            # 2. 检查动作是否结束
            if self.current_action:
                if self.current_action in self.continuous_motions:
                    if self.robot.getTime() > self.action_end_time:
                        self.current_action = None 
                elif self.robot.getTime() >= self.action_end_time:
                    finished = self.current_action
                    self.current_action = None
                    self.send_event("DONE", action=finished)

            # 3. 执行新指令
            is_busy_atomic = (self.current_action and self.current_action not in self.continuous_motions)
            if not is_busy_atomic and self.pending_cmd is not None:
                cmd = self.pending_cmd
                self.pending_cmd = None
                self.start_action(cmd)

if __name__ == "__main__":
    GoalkeeperClient().run()