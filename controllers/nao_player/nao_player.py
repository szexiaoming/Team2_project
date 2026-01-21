from controller import Robot, Motion
import socket, json, select, sys

TIME_STEP = 32

# 每个 Player 自己监听的端口（收 Supervisor 命令）
PORT_MAP = {
    "B1": 10001, "B2": 10002, "B3": 10003, "B4": 10004,
    "R1": 10011, "R2": 10012, "R3": 10013, "R4": 10014,
}

# Supervisor 用来收 DONE 的端口
SUPERVISOR_HOST = "127.0.0.1"
SUPERVISOR_PORT = 12000

def safe_get_duration(m: Motion, default_sec: float) -> float:
    try:
        d = m.getDuration()
        if d and d > 0.01:
            return float(d)
    except Exception:
        pass
    return float(default_sec)

class NaoPlayer:
    """
    极简执行器：
    - 收到 cmd 后：若空闲 -> 播放对应 motion（完整播完）
    - 忙则缓存 1 条 pending（只保留最新）
    - 动作播完：向 Supervisor 回报 DONE
    - 支持 INTERRUPT_*：立即打断当前动作，马上执行指定动作（用于摔倒起立）
    """

    def __init__(self):
        self.robot = Robot()

        # robot id from controllerArgs
        self.rid = sys.argv[1] if len(sys.argv) > 1 else "B1"
        if self.rid not in PORT_MAP:
            print("Unknown robot id:", self.rid, "fallback to B1")
            self.rid = "B1"

        # UDP recv (commands)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(("127.0.0.1", PORT_MAP[self.rid]))
        self.sock_rx.setblocking(False)

        # UDP tx (DONE -> supervisor)
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"[{self.rid}] listening UDP on port {PORT_MAP[self.rid]}")

        # motions
        self.motion = {
            "FWD": Motion("motions/Forwards50.motion"),
            "BWD": Motion("motions/Backwards.motion"),
            "TURN_L": Motion("motions/TurnLeftSmall.motion"),
            "TURN_R": Motion("motions/TurnRightSmall.motion"),
            "KICK_L": Motion("motions/Shoot.motion"),
            "GETUP_FRONT": Motion("motions/GetUpFront.motion"),
            "GETUP_BACK": Motion("motions/GetUpBack.motion"),
        }

        # 如果 getDuration 拿不到，就用这些默认值（可按你们文件实际时长微调）
        self.default_duration = {
            "FWD": 0.55,
            "BWD": 0.55,
            "TURN_L": 0.45,
            "TURN_R": 0.45,
            "KICK_L": 1.20,
            "GETUP_FRONT": 3.00,
            "GETUP_BACK": 3.00,
        }

        # 安全保险丝：任何动作最长不超过这个秒数（避免 getDuration 异常卡死）
        self.max_action_sec = 6.0

        self.latest_seq = -1
        self.pending_cmd = None

        self.current_action = None
        self.action_end_time = 0.0  # robot.getTime()

        # 启动时告诉 supervisor 我在线
        self.send_event("READY", action="")

    def send_event(self, event: str, action: str):
        msg = {"id": self.rid, "event": event, "action": action, "t": self.robot.getTime()}
        self.sock_tx.sendto(json.dumps(msg).encode("utf-8"), (SUPERVISOR_HOST, SUPERVISOR_PORT))

    def poll_cmd(self):
        r, _, _ = select.select([self.sock_rx], [], [], 0)
        if not r:
            return None
        try:
            data, _ = self.sock_rx.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))
            seq = msg.get("seq", -1)
            if seq <= self.latest_seq:
                return None
            self.latest_seq = seq
            return msg
        except Exception:
            return None

    def start_action(self, cmd: str):
        now = self.robot.getTime()

        if cmd == "STOP":
            # STOP 不播放 motion，立刻 DONE
            self.current_action = None
            self.action_end_time = now
            self.send_event("DONE", action="STOP")
            return

        # 只支持左脚：KICK_R 当 KICK_L（避免 supervisor 误发）
        if cmd == "KICK_R":
            cmd = "KICK_L"

        if cmd not in self.motion:
            # 未知命令：当 STOP
            self.current_action = None
            self.action_end_time = now
            self.send_event("DONE", action="STOP")
            return

        self.current_action = cmd
        m = self.motion[cmd]
        m.play()

        dur = safe_get_duration(m, self.default_duration.get(cmd, 1.0))
        # safety cap（避免 duration 异常）
        dur = min(float(dur), self.max_action_sec)
        self.action_end_time = now + dur + 0.05

    def interrupt_action(self, cmd: str):
        """
        立即打断当前动作，开始执行 cmd（用于摔倒起立等紧急动作）。
        """
        # 清掉队列，避免打断后又马上执行旧命令
        self.pending_cmd = None

        # 尝试 stop 当前 motion（不是必须，很多版本没有 stop 也没事）
        try:
            if self.current_action and self.current_action in self.motion:
                self.motion[self.current_action].stop()
        except Exception:
            pass

        self.current_action = None
        self.action_end_time = self.robot.getTime()

        self.start_action(cmd)

    def update_action(self):
        if self.current_action is None:
            return
        now = self.robot.getTime()
        if now >= self.action_end_time:
            finished = self.current_action
            self.current_action = None
            self.send_event("DONE", action=finished)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            # 1) 接收命令（即使忙，也只缓存最新一条）
            msg = self.poll_cmd()
            if msg:
                cmd = msg.get("cmd", "STOP")
                cmd = cmd if cmd else "STOP"

                # ===== 新增：支持紧急打断 =====
                # Supervisor 会发：INTERRUPT_GETUP_FRONT / INTERRUPT_GETUP_BACK
                if cmd.startswith("INTERRUPT_"):
                    cmd2 = cmd.replace("INTERRUPT_", "")
                    # 只允许打断成已知动作，否则当 STOP
                    if cmd2 in self.motion or cmd2 == "STOP":
                        self.interrupt_action(cmd2)
                    else:
                        self.interrupt_action("STOP")
                    # 本帧结束（避免下面 pending 再触发一次）
                    continue

                self.pending_cmd = cmd

            # 2) 更新当前动作是否结束
            self.update_action()

            # 3) 如果空闲且有 pending -> 开始下一动作（严格串行）
            if self.current_action is None and self.pending_cmd is not None:
                cmd = self.pending_cmd
                self.pending_cmd = None
                self.start_action(cmd)

if __name__ == "__main__":
    NaoPlayer().run()