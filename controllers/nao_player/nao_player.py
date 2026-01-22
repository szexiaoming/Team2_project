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
    """安全获取动作时长，防止读取失败"""
    try:
        d = m.getDuration()
        if d and d > 0.01:
            return float(d)
    except Exception:
        pass
    return float(default_sec)

class NaoPlayer:
    def __init__(self):
        self.robot = Robot()

        # 1. 确定 Robot ID
        # 优先从命令行参数获取 (Webots controllerArgs)
        self.rid = sys.argv[1] if len(sys.argv) > 1 else "B1"
        
        # 如果参数不对，尝试使用机器人名称 (DEF name)
        if self.rid not in PORT_MAP:
            name = self.robot.getName()
            if name in PORT_MAP:
                self.rid = name
            else:
                print(f"Unknown robot id: {self.rid}, fallback to B1")
                self.rid = "B1"

        # 2. 初始化 UDP 接收端 (收指令)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(("127.0.0.1", PORT_MAP[self.rid]))
        self.sock_rx.setblocking(False)

        # 3. 初始化 UDP 发送端 (发 DONE/READY)
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"[{self.rid}] listening UDP on port {PORT_MAP[self.rid]}")

        # 4. 加载动作文件 (确保 motions 文件夹下有这些文件)
        self.motion = {
            "FWD": Motion("motions/Forwards50.motion"),
            "BWD": Motion("motions/Backwards.motion"),
            "TURN_L": Motion("motions/TurnLeftSmall.motion"),
            "TURN_R": Motion("motions/TurnRightSmall.motion"),
            
            # === 新增：左右横移 ===
            "SIDE_L": Motion("motions/SideStepLeft.motion"),
            "SIDE_R": Motion("motions/SideStepRight.motion"),
            
            "KICK_L": Motion("motions/Shoot.motion"),
            "GETUP_FRONT": Motion("motions/GetUpFront.motion"),
            "GETUP_BACK": Motion("motions/GetUpBack.motion"),
        }

        # 5. 设置默认时长 (用于 getDuration 失败时的兜底)
        self.default_duration = {
            "FWD": 0.55,
            "BWD": 0.55,
            "TURN_L": 0.45,
            "TURN_R": 0.45,
            "SIDE_L": 0.60, # 侧移通常较慢
            "SIDE_R": 0.60,
            "KICK_L": 1.20,
            "GETUP_FRONT": 4.00, # 爬起动作较长
            "GETUP_BACK": 4.00,
        }

        # 安全保险丝：任何动作最长不超过这个秒数 (防止死锁)
        self.max_action_sec = 6.0

        # 状态变量
        self.latest_seq = -1
        self.pending_cmd = None
        self.current_action = None
        self.action_end_time = 0.0  

        # 启动时发送 READY 信号
        self.send_event("READY", action="")

    def send_event(self, event: str, action: str):
        """发送状态给 Supervisor"""
        msg = {"id": self.rid, "event": event, "action": action, "t": self.robot.getTime()}
        try:
            self.sock_tx.sendto(json.dumps(msg).encode("utf-8"), (SUPERVISOR_HOST, SUPERVISOR_PORT))
        except Exception as e:
            print(f"Socket send error: {e}")

    def poll_cmd(self):
        """
        【关键修复】从 UDP 缓冲区读取所有积压的命令，只返回最新的一条 (seq 最大的)。
        这能彻底解决高频发送下的动作延迟问题。
        """
        latest_msg = None
        
        while True:
            # 检查是否有数据可读
            r, _, _ = select.select([self.sock_rx], [], [], 0)
            if not r:
                break # 缓冲区已空
            
            try:
                data, _ = self.sock_rx.recvfrom(4096)
                msg = json.loads(data.decode("utf-8"))
                seq = msg.get("seq", -1)
                
                # 只有序列号更新的指令才有效
                if seq > self.latest_seq:
                    self.latest_seq = seq
                    latest_msg = msg
            except Exception:
                pass
        
        return latest_msg

    def start_action(self, cmd: str):
        """开始执行一个动作"""
        now = self.robot.getTime()

        # 处理 STOP
        if cmd == "STOP":
            self.current_action = None
            self.action_end_time = now
            self.send_event("DONE", action="STOP")
            return

        # 容错处理：如果没有右脚动作，强制转为左脚
        if cmd == "KICK_R":
            cmd = "KICK_L"

        # 未知动作处理
        if cmd not in self.motion:
            print(f"[{self.rid}] Unknown motion: {cmd}")
            self.current_action = None
            self.action_end_time = now
            self.send_event("DONE", action="STOP")
            return

        # 播放动作
        self.current_action = cmd
        m = self.motion[cmd]
        
        # 这里的 play() 是非阻塞的，Webots 会在后台播放
        m.play()

        # 计算结束时间
        dur = safe_get_duration(m, self.default_duration.get(cmd, 1.0))
        dur = min(float(dur), self.max_action_sec) # 限制最大时长
        
        # +0.05 是为了留一点缓冲时间
        self.action_end_time = now + dur + 0.05

    def interrupt_action(self, cmd: str):
        """
        立即打断当前动作，开始执行 cmd（用于摔倒起立）。
        """
        # 清空等待队列
        self.pending_cmd = None

        # 尝试停止当前动作
        try:
            if self.current_action and self.current_action in self.motion:
                self.motion[self.current_action].stop()
        except Exception:
            pass

        self.current_action = None
        self.action_end_time = self.robot.getTime()
        
        # 立即开始新动作
        print(f"[{self.rid}] Interrupting -> {cmd}")
        self.start_action(cmd)

    def update_action(self):
        """检查动作是否播放完毕"""
        if self.current_action is None:
            return
            
        now = self.robot.getTime()
        if now >= self.action_end_time:
            finished = self.current_action
            self.current_action = None
            # 告诉 Supervisor 我做完了
            self.send_event("DONE", action=finished)

    def run(self):
        """主循环"""
        while self.robot.step(TIME_STEP) != -1:
            # 1. 接收命令 (只取最新)
            msg = self.poll_cmd()
            if msg:
                cmd = msg.get("cmd", "STOP")
                cmd = cmd if cmd else "STOP"

                # === 紧急打断逻辑 ===
                if cmd.startswith("INTERRUPT_"):
                    cmd2 = cmd.replace("INTERRUPT_", "")
                    # 如果是已知动作或STOP，立即执行
                    if cmd2 in self.motion or cmd2 == "STOP":
                        self.interrupt_action(cmd2)
                    else:
                        self.interrupt_action("STOP")
                    # 打断后，本帧不再处理 Pending 逻辑
                    continue

                # 普通指令存入 Pending
                self.pending_cmd = cmd

            # 2. 更新当前动作状态 (检查是否结束)
            self.update_action()

            # 3. 如果当前空闲，且有等待执行的指令 -> 开始执行
            # 这保证了动作是串行的，不会还没走完就踢球
            if self.current_action is None and self.pending_cmd is not None:
                cmd = self.pending_cmd
                self.pending_cmd = None
                self.start_action(cmd)

if __name__ == "__main__":
    NaoPlayer().run()