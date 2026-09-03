"""Xbox 手把遙控 MyCobot320（Windows XInput + pymycobot，零第三方依賴）

控制映射（都可在下方 MAP 區修改）:
  左搖桿 X → J1(底座旋轉)    左搖桿 Y → J2(肩)
  右搖桿 Y → J3(肘)          右搖桿 X → J4(手腕旋轉1)
  LB / RB  → J5 反轉/正轉(按住)
  LT / RT  → J6 反轉/正轉(按住, 推越多越快)

按鍵:
  X  = 切換速度檔(慢/中/快)
  A  = 回到起始位置(home)
  Y  = 把當前位置設為新 home
  BACK = 退出(倒數 EXIT_DELAY 秒後所有關節回 0°(歸零)再關閉;
              倒數中 Ctrl+C 可跳過歸零直接關閉)
  Ctrl+C = 退出(歸零後關閉; 歸零中再按一次可跳過)

安全: 搖桿死區 / 關節軟限位(官方範圍) / 手把斷線 3 秒自動退出(同樣歸零)
"""
import ctypes
import ctypes.wintypes as wt
import sys
import time

import serial.tools.list_ports as lp
from pymycobot import MyCobot320

# ================= 機械臂參數 =================
BAUD = 115200
TICK = 0.1                  # 控制週期(秒) = 10Hz
SEND_SPEED = 80             # send_angle 速度參數 (deg/s)，決定單軸硬上限
CALIBRATE_EVERY = 2.0       # 每 N 秒用實際讀數校正一次目標, 防漂移
LOST_SECONDS = 3.0          # 手把斷線超過 N 秒 → 自動退出
WAIT_SECONDS = 20           # 等待到位(A 回家)上限

# 關閉程序（退出時收尾：延遲 → 歸零 → 關閉）
EXIT_DELAY = 3.0            # 退出後倒數幾秒才開始歸零(倒數中 Ctrl+C 可跳過歸零)
ZERO_SPEED = 40             # 歸零 send_angles 速度
ZERO_WAIT_S = 25.0          # 等待歸零到位的上限(秒)，超過仍關閉

# J1~J5 ±165, J6 ±175 (官方 myCobot320 M5 規格)
JOINT_LIMITS = [(-165.0, 165.0)] * 5 + [(-175.0, 175.0)]

# 三檔速度: 搖桿每 tick 最大步進(度)   (X 鍵循環切換)
# 實際角速度 ≈ 步進 / TICK: 8°/s、16°/s、50°/s
AXIS_STEP = {0: 0.8, 1: 1.6, 2: 5.0}
# 扳機每 tick 最大步進
TRIG_STEP = {0: 1.0, 1: 2.0, 2: 6.0}
GEAR_NAME = {0: '慢', 1: '中', 2: '快'}

# ================= 手把→關節映射 =================
# 搖桿: (名稱, 關節1~6, 方向 ±1)  —— 方向若反了, 把 +1 改 -1
STICK_MAP = [
    ('LX', 1, +1),
    ('LY', 2, +1),
    ('RY', 3, +1),
    ('RX', 4, +1),
]
# 扳機: (名稱, 關節, 方向)   LT=J6反轉, RT=J6正轉
TRIG_MAP = [('LT', 6, -1), ('RT', 6, +1)]
# 肩鍵: (名稱, 關節, 方向)   LB=J5反轉, RB=J5正轉
BUMP_MAP = [('LB', 5, -1), ('RB', 5, +1)]

# ================= XInput 底層 =================
ERROR_SUCCESS = 0
BTN = {'A': 0x1000, 'B': 0x2000, 'X': 0x4000, 'Y': 0x8000,
       'LB': 0x0100, 'RB': 0x0200,
       'START': 0x0010, 'BACK': 0x0020}
AXIS_DEAD = {'LX': 7849, 'LY': 7849, 'RX': 8689, 'RY': 8689}
TRIG_DEAD = 30
STICK_FULL = 32767.0


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [('wButtons', wt.WORD), ('bLeftTrigger', wt.BYTE),
                ('bRightTrigger', wt.BYTE), ('sThumbLX', ctypes.c_short),
                ('sThumbLY', ctypes.c_short), ('sThumbRX', ctypes.c_short),
                ('sThumbRY', ctypes.c_short)]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [('dwPacketNumber', wt.DWORD), ('Gamepad', XINPUT_GAMEPAD)]


def find_robot_port():
    for p in lp.comports():
        if 'CH9102' in p.description or 'USB-Enhanced-SERIAL' in p.description:
            return p.device
    return None


def axis_norm(v, dead):
    """搖桿 -32768..32767 → -1..1, 死區內為 0"""
    if abs(v) < dead:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dead) / (STICK_FULL - dead)


def trig_norm(v):
    """扳機 0..255 → 0..1, 小於死區為 0"""
    if v < TRIG_DEAD:
        return 0.0
    return (v - TRIG_DEAD) / (255.0 - TRIG_DEAD)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def zero_joints(mc):
    """關閉程序：所有關節送回 0°(J1~J6 = 0) 並輪詢等到位。
    回傳 True=已歸零到位, False=發送失敗或等待超時(仍關閉)。"""
    print('關節歸零中 (J1~J6 → 0°) ...')
    try:
        mc.send_angles([0.0] * 6, ZERO_SPEED)
    except Exception as e:
        print('歸零指令發送失敗:', e)
        return False
    t0 = time.time()
    while time.time() - t0 < ZERO_WAIT_S:
        try:
            a = mc.get_angles()
        except Exception:
            a = -1
        if a != -1 and a is not None and len(a) == 6 \
                and all(abs(float(x)) <= 2.0 for x in a):
            print('✔ 已歸零 (J1~J6 = 0°)')
            return True
        time.sleep(0.2)
    print('⚠ 等待歸零超時(未完全到位)，仍關閉程式')
    return False


def main():
    # ---------- 1. 連接機械臂 ----------
    port = find_robot_port()
    if not port:
        print('找不到機械臂串口(CH9102), 請確認 USB 已接好')
        return 1
    print(f'連接機械臂 {port} ...')
    mc = MyCobot320(port, BAUD, timeout=0.5, debug=False)

    angles = -1
    for _ in range(10):
        angles = mc.get_angles()
        if angles != -1:
            break
        time.sleep(0.5)
    if angles == -1:
        print('機械臂無應答, 請確認 M5 螢幕為 "Atom: ok"')
        mc.close()
        return 1
    if mc.is_power_on() != 1:
        print('舵機供電未開, power_on() ...')
        mc.power_on()
        time.sleep(1)
    target = [float(a) for a in mc.get_angles()]
    home = list(target)
    print('起始角度:', ['%.1f' % a for a in target])

    # ---------- 2. 找手把 ----------
    xi = None
    for name in ('xinput1_4.dll', 'xinput1_3.dll', 'xinput9_1_0.dll', 'xinput.dll'):
        try:
            xi = ctypes.WinDLL(name)
            break
        except OSError:
            continue
    if xi is None:
        print('找不到 XInput DLL, 請確認是 Windows')
        mc.close()
        return 1
    xi.XInputGetState.restype = wt.DWORD
    xi.XInputGetState.argtypes = [wt.DWORD, ctypes.POINTER(XINPUT_STATE)]
    pad = None
    for i in range(4):
        st = XINPUT_STATE()
        if xi.XInputGetState(i, ctypes.byref(st)) == ERROR_SUCCESS:
            pad = i
            break
    if pad is None:
        print('找不到 Xbox 手把! 請插上 USB 或開啟無線並配對後重跑')
        mc.close()
        return 1
    print(f'手把 UserIndex={pad}, 就緒! 按 BACK 退出')

    gear = 1
    prev_btn = 0
    moving_home = False
    last_cal = time.time()
    last_print = 0.0
    lost_at = None
    btn_name = {v: k for k, v in BTN.items()}
    ctrl_c = False          # 是否經 Ctrl+C 退出(ctrl+C 路徑不再等倒數, 直接歸零)

    try:
        while True:
            st = XINPUT_STATE()
            ok = xi.XInputGetState(pad, ctypes.byref(st)) == ERROR_SUCCESS
            if not ok:
                if lost_at is None:
                    lost_at = time.time()
                    print('\n⚠ 手把斷線! 機械臂已停住, 等待重新連上 ...')
                elif time.time() - lost_at > LOST_SECONDS:
                    print('\n手把斷線超過 %ds, 準備歸零退出 (%ds 後所有關節回 0°)'
                          % (LOST_SECONDS, EXIT_DELAY))
                    break
                time.sleep(0.2)
                continue
            lost_at = None
            g = st.Gamepad

            # ---- 按鍵(邊緣觸發) ----
            cur = g.wButtons
            def edge(b):
                return (cur & b) and not (prev_btn & b)
            if edge(BTN['BACK']):
                print('\nBACK: 準備歸零退出 (%d 秒後所有關節回 0°; '
                      '倒數中 Ctrl+C 可跳過歸零直接關閉)' % EXIT_DELAY)
                break
            if edge(BTN['X']):
                gear = (gear + 1) % 3
                print(f'\n>> 速度檔: {GEAR_NAME[gear]}')
            if edge(BTN['Y']):
                home = list(target)
                print('\n>> 已將當前位置設為新 home:', ['%.1f' % a for a in home])
            if edge(BTN['A']):
                moving_home = True
                print('\n>> 回 home ...')
            prev_btn = cur

            # ---- 回家模式: 忽略搖桿直到到位 ----
            if moving_home:
                mc.send_angles(home, SEND_SPEED)
                now = mc.get_angles()
                if now != -1 and all(abs(a - h) <= 2.0 for a, h in zip(now, home)):
                    target = [float(a) for a in now]
                    moving_home = False
                    print('✔ 已回到 home')
                elif now != -1:
                    target = [float(a) for a in now]
                time.sleep(0.2)
            else:
                # ---- 增量角度 ----
                axes = {'LX': axis_norm(g.sThumbLX, AXIS_DEAD['LX']),
                        'LY': axis_norm(g.sThumbLY, AXIS_DEAD['LY']),
                        'RX': axis_norm(g.sThumbRX, AXIS_DEAD['RX']),
                        'RY': axis_norm(g.sThumbRY, AXIS_DEAD['RY'])}
                trig = {'LT': trig_norm(g.bLeftTrigger),
                        'RT': trig_norm(g.bRightTrigger)}

                changed = {}
                for name, j, d in STICK_MAP:
                    step = axes[name] * AXIS_STEP[gear] * d
                    if step != 0.0:
                        changed[j - 1] = target[j - 1] + step
                for name, j, d in BUMP_MAP:          # LB/RB 按住即動
                    if cur & BTN[name]:
                        changed[j - 1] = target[j - 1] + AXIS_STEP[gear] * d
                for name, j, d in TRIG_MAP:
                    step = trig[name] * TRIG_STEP[gear] * d
                    if step != 0.0:
                        changed[j - 1] = target[j - 1] + step

                for i, a in changed.items():
                    target[i] = clamp(a, *JOINT_LIMITS[i])
                    mc.send_angle(i + 1, round(target[i], 1), SEND_SPEED)

                # ---- 定期用實際讀數校正目標, 防累積漂移 ----
                # 只在「本 tick 沒有任何軸在動」時校正:
                # get_angles() 串口來回會阻塞迴圈數百 ms,
                # 若在持續推桿中執行, 指令停送 → 手臂頓一下。
                # 推桿中不校正(也不需要: 目標是自己累加的);
                # 停止操作 2 秒後才靜靜校正一次。
                if not changed and time.time() - last_cal > CALIBRATE_EVERY:
                    real = mc.get_angles()
                    if real != -1:
                        target = [float(a) for a in real]
                    last_cal = time.time()

            # ---- 狀態列(4Hz) ----
            if time.time() - last_print > 0.25:
                last_print = time.time()
                sys.stdout.write('\r[%s] J1~J6: %s    '
                                 % (GEAR_NAME[gear],
                                    ' '.join('%6.1f' % a for a in target)))
                sys.stdout.flush()

            time.sleep(TICK)
    except KeyboardInterrupt:
        ctrl_c = True
        print('\nCtrl+C: 準備關閉(關節歸零後退出; 歸零中再按一次可跳過)')
    finally:
        # ---- 統一收尾: 倒數(僅 BACK/斷線等正常退出) → 關節全回 0° → 關閉連線 ----
        skip_zero = False
        if not ctrl_c and EXIT_DELAY > 0:
            # 正常退出: 給 EXIT_DELAY 秒倒數, 讓人有時間離開機械臂
            print('\n%d 秒後所有關節歸零並退出 (Ctrl+C 可跳過歸零直接關閉) ...'
                  % EXIT_DELAY)
            try:
                t_end = time.time() + EXIT_DELAY
                while time.time() < t_end:
                    time.sleep(0.2)
            except KeyboardInterrupt:
                skip_zero = True
                print('\n已跳過歸零, 直接關閉 (機械臂原地停住)')
        if not skip_zero:
            # Ctrl+C 路徑不等倒數直接歸零; 歸零中再按一次 Ctrl+C 可跳過
            try:
                zero_joints(mc)
            except KeyboardInterrupt:
                print('\n歸零被中斷, 直接關閉')
        mc.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
