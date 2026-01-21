from controller import Robot, Motion
import socket, json, select, sys

TIME_STEP = 32
PORT_MAP = {
    "B1": 10001, "B2": 10002, "B3": 10003, "B4": 10004,
    "R1": 10011, "R2": 10012, "R3": 10013, "R4": 10014,
}
SUPERVISOR_HOST = "127.0.0.1"
SUPERVISOR_PORT = 12000
MOTION_PATH_PREFIX = "motions/" # 请确保路径正确

def safe_get_duration(m: Motion, default_sec: float) -> float:
    try:
        d = m.getDuration()
        if d and d > 0.01: return float(d)
    except: pass
    return float(default_sec)

class NaoClient:
    def __init__(self):
        self.robot = Robot()
        self.rid = sys.argv[1] if len(sys.argv) > 1 else "B2"
        if self.rid not in PORT_MAP: self.rid = "B2"
        
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(("127.0.0.1", PORT_MAP[self.rid]))
        self.sock_rx.setblocking(False)
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"[{self.rid}] Player listening on {PORT_MAP[self.rid]}")

        # 动作定义
        self.motion = {
            "FWD": Motion(MOTION_PATH_PREFIX + "Forwards50.motion"),
            "BWD": Motion(MOTION_PATH_PREFIX + "Backwards.motion"),
            "TURN_L": Motion(MOTION_PATH_PREFIX + "TurnLeftSmall.motion"),
            "TURN_R": Motion(MOTION_PATH_PREFIX + "TurnRightSmall.motion"),
            "SIDE_L": Motion(MOTION_PATH_PREFIX + "SidestepLeft.motion"),
            "SIDE_R": Motion(MOTION_PATH_PREFIX + "SidestepRight.motion"),
            "KICK_L": Motion(MOTION_PATH_PREFIX + "Shoot.motion"),
            "STAND": Motion(MOTION_PATH_PREFIX + "Stand.motion"),
            "GETUP_FRONT": Motion(MOTION_PATH_PREFIX + "GetUpFront.motion"),
            "GETUP_BACK": Motion(MOTION_PATH_PREFIX + "GetUpBack.motion"),
        }
        
        # 连续动作 (不需要汇报DONE，循环播放)
        self.continuous_motions = ["FWD", "BWD", "TURN_L", "TURN_R", "SIDE_L", "SIDE_R", "STAND"]

        self.latest_seq = -1
        self.pending_cmd = None
        self.current_action = None
        self.action_end_time = 0.0

        # 启动握手
        self.send_event("READY", action="")

    def send_event(self, event, action=""):
        msg = {"id": self.rid, "event": event, "action": action}
        try: self.sock_tx.sendto(json.dumps(msg).encode(), (SUPERVISOR_HOST, SUPERVISOR_PORT))
        except: pass

    def poll_cmd(self):
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

        # 优化：如果是连续动作且正在做，只刷新超时，不打断
        if cmd in self.continuous_motions and self.current_action == cmd:
            if cmd in self.motion: self.motion[cmd].setLoop(True)
            self.action_end_time = now + 0.5 # 续命 0.5s
            return

        # 停止旧动作 (STAND除外，为了平滑可以保留一瞬间)
        if self.current_action and self.current_action in self.motion:
            if self.current_action != "STAND":
                self.motion[self.current_action].stop()

        if cmd == "STOP":
            self.current_action = "STAND"
            self.motion["STAND"].setLoop(True)
            self.motion["STAND"].play()
            self.action_end_time = now + 0.5
            return

        if cmd not in self.motion: return

        self.current_action = cmd
        m = self.motion[cmd]
        
        # 设置循环模式
        loop = (cmd in self.continuous_motions)
        m.setLoop(loop)
        m.play()

        dur = safe_get_duration(m, 1.0)
        # 连续动作设置个短时间，依赖 start_action 不断刷新来维持
        # 原子动作 (KICK) 设置真实时长
        self.action_end_time = now + (0.5 if loop else dur + 0.05)

    def run(self):
        # 初始站立
        if "STAND" in self.motion:
            self.motion["STAND"].setLoop(True)
            self.motion["STAND"].play()
            self.current_action = "STAND"

        while self.robot.step(TIME_STEP) != -1:
            msg = self.poll_cmd()
            if msg:
                cmd = msg.get("cmd", "STOP")
                # 紧急打断
                if cmd.startswith("INTERRUPT_"):
                    real_cmd = cmd.replace("INTERRUPT_", "")
                    self.pending_cmd = None
                    # 强行停止当前
                    if self.current_action and self.current_action in self.motion:
                        self.motion[self.current_action].stop()
                    self.start_action(real_cmd)
                    continue
                self.pending_cmd = cmd

            # 动作结束判定
            if self.current_action:
                # 连续动作超时才算停
                if self.current_action in self.continuous_motions:
                    if self.robot.getTime() > self.action_end_time:
                        self.current_action = None # 超时视为停止
                # 原子动作 (踢球/起立) 结束汇报 DONE
                elif self.robot.getTime() >= self.action_end_time:
                    finished = self.current_action
                    self.current_action = None
                    self.send_event("DONE", action=finished)

            # 执行新命令
            if self.pending_cmd:
                # 只有当前是连续动作 或者 空闲时，才允许切换 (Kick时不能被打断，除非是Interrupt)
                is_busy_atomic = (self.current_action and self.current_action not in self.continuous_motions)
                if not is_busy_atomic:
                    c = self.pending_cmd
                    self.pending_cmd = None
                    self.start_action(c)

if __name__ == "__main__":
    NaoClient().run()