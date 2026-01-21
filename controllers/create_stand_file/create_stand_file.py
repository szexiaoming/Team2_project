import os

def generate_motion_file():
    # 1. 定义 Webots NAO 机器人所有关键关节的标准顺序
    # (这个顺序必须严格对应，不能乱)
    joint_names = [
        "HeadYaw", "HeadPitch",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"
    ]

    # 2. 这是你代码里那个“绝对不倒”的姿势数据
    # 我补全了头部和手肘的默认值 0.0，确保数据完整
    target_poses = {
        "HeadYaw": 0.0,        "HeadPitch": 0.0,
        "LShoulderPitch": 1.5, "LShoulderRoll": 0.1, "LElbowYaw": 0.0, "LElbowRoll": 0.0,
        "LHipYawPitch": 0.0,   "LHipRoll": 0.0,      "LHipPitch": -0.4,
        "LKneePitch": 0.85,    "LAnklePitch": -0.45, "LAnkleRoll": 0.0,
        "RHipYawPitch": 0.0,   "RHipRoll": 0.0,      "RHipPitch": -0.4,
        "RKneePitch": 0.85,    "RAnklePitch": -0.45, "RAnkleRoll": 0.0,
        "RShoulderPitch": 1.5, "RShoulderRoll": -0.1, "RElbowYaw": 0.0, "RElbowRoll": 0.0
    }

    # 3. 构建文件内容
    # 第一行：固定表头
    header = "#WEBOTS_MOTION,V1.0," + ",".join(joint_names)
    
    # 构建数据行
    values = []
    for name in joint_names:
        # 如果字典里有这个关节，就用字典的值，否则用 0.0
        val = target_poses.get(name, 0.0)
        values.append(str(val))
    
    data_str = ",".join(values)

    # 生成两帧数据（第0秒和第1秒保持动作不变）
    line1 = "00:00:000," + data_str
    line2 = "00:01:000," + data_str

    content = f"{header}\n{line1}\n{line2}"

    # 4. 写入文件
    file_path = "motions/Stand.motion"
    
    # 确保文件夹存在
    if not os.path.exists("motions"):
        os.makedirs("motions")

    with open(file_path, "w", encoding="utf-8", newline='\n') as f:
        f.write(content)
    
    print(f"✅ 成功生成文件: {os.path.abspath(file_path)}")
    print("现在你可以在 Webots 中直接使用 Stand.motion 了！")

if __name__ == "__main__":
    generate_motion_file()