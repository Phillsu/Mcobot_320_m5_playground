import sys, time
import serial.tools.list_ports as lp
from pymycobot import MyCobot320


def find_robot_port():
    for p in lp.comports():
        if 'CH9102' in p.description or 'USB-Enhanced-SERIAL' in p.description:
            return p.device
    return None


port = find_robot_port() or 'COM3'
print(f'使用串口: {port}', flush=True)
mc = MyCobot320(port, 115200, timeout=0.5, debug=False)
print('连接成功，开始分层体检...\n', flush=True)


def probe(name, fn):
    r = fn()
    print(f'{name:32s} -> {r}', flush=True)
    time.sleep(0.3)
    return r


basic = probe('get_basic_version (主控 Basic 固件)', mc.get_basic_version)
atom_conn = probe('is_controller_connected (Atom 连接, 1=通 0=断 -1=错误)', mc.is_controller_connected)
atom_ver = probe('get_atom_version (Atom 固件版本)', mc.get_atom_version)
power = probe('is_power_on (舵机供电, 1=开 0=关 -1=错误)', mc.is_power_on)
mc.close()

print('\n================ 诊断结论 ================', flush=True)
ok = True
if basic == -1:
    ok = False
    print('✘ 主控 Basic 无应答 -> USB 线/串口/主控固件问题，换线直连、重刷 Basic 固件')
if atom_conn in (0, -1) or atom_ver == -1:
    ok = False
    print('✘ Atom 无响应 -> 屏幕停留在 "Atom: ok" 才算就绪；若显示 "Atom: no"：')
    print('   1) 确认急停按钮已旋转弹出(释放)，按下状态会切断舵机供电')
    print('   2) 机械臂摆成舒展姿态(不要蜷缩)，断开电源等 5 秒重新上电')
    print('   3) 上电后等待 M5 屏幕出现 "Atom: ok" 再运行')
    print('   4) 仍不行则用 myStudio 重刷 Atom 固件')
if power == 0:
    print('⚠ 舵机供电关闭：若 Atom 已 ok，可先调用 mc.power_on() 上电；'
          '若急停被按下则先释放急停')
if ok:
    print('✔ 一切正常，可执行运动指令')
else:
    print('==========================================', flush=True)
    sys.exit(1)
