import time
import serial.tools.list_ports as lp
from pymycobot import MyCobot320

BAUD = 115200


def find_robot_port():
    for p in lp.comports():
        if 'CH9102' in p.description or 'USB-Enhanced-SERIAL' in p.description:
            return p.device
    return None


PORT = find_robot_port() or 'COM3'
print(f'使用串口: {PORT}')
mc = MyCobot320(PORT, BAUD, timeout=0.5, debug=True)
print(BAUD)
print(f'已连接 {PORT} @ {BAUD}，开始读取关节角度...')

# 刚上电时主控可能还没就绪，重试多次
angles = -1
for i in range(10):
    angles = mc.get_angles()
    if angles != -1:
        print('✔ 关节角度:', angles)
        break
    time.sleep(0.5)

if angles == -1:
    print('✘ 机械臂 10 次均无应答（-1），请先确认 M5 屏幕显示 "Atom: ok"：')
    print('  1) 将机械臂摆成舒展姿态(不要蜷缩)，断开电源等 5~10 秒后重新上电')
    print('  2) 等待 M5 屏幕出现 "Atom: ok" 再运行本脚本')
    print('  3) 若仍显示 "Atom: no"，用 myStudio 重新烧录 Atom 固件后再重启')
    mc.close()
    raise SystemExit(1)

# 能读到角度说明 Atom 已就绪；若舵机供电未开则先上电
if mc.is_power_on() != 1:
    print('舵机供电未开启，调用 power_on() ...')
    mc.power_on()
    time.sleep(1)

# 再确认一次角度后测试运动指令
angles = mc.get_angles()
print('上电后关节角度:', angles)
home = list(angles)   # ★ 记录起始姿态：测试完会回到这里
mc.send_angle(1, 40, 20)

print('等待 6 秒让机械臂到达目标位置 ...')
time.sleep(6)

now = mc.get_angles()
print('运动后关节角度:', now)


def wait_reach(target, timeout=20, tol=2.0):
    """轮询直到 6 个关节都接近目标(误差<tol度)，超时则返回当前角度"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        cur = mc.get_angles()
        if cur != -1 and all(abs(c - t) <= tol for c, t in zip(cur, target)):
            return cur
        time.sleep(0.5)
    return mc.get_angles()


# ---- 测试完成，回到起始姿态 ----
print('测试完成，现在回到起始姿态 ...')
mc.send_angles(home, 20)          # 6 关节同步缓慢回位
cur = wait_reach(home)
print('回到原位后关节角度:', cur)
mc.close()
print('✔ 测试完成，机械臂已回到起始姿态')
