"""Xbox 手柄偵測 (Windows XInput, 零依賴)

掃描 UserIndex 0~3, 找出已連接的 Xbox 手柄,
接著即時顯示 6 秒按鍵 / 搖桿 / 扳機狀態。
"""
import ctypes
import ctypes.wintypes as wt
import time

ERROR_SUCCESS = 0

BTN = {
    'DPAD_UP': 0x0001, 'DPAD_DOWN': 0x0002, 'DPAD_LEFT': 0x0004, 'DPAD_RIGHT': 0x0008,
    'START': 0x0010, 'BACK': 0x0020,
    'LEFT_THUMB': 0x0040, 'RIGHT_THUMB': 0x0080,
    'LEFT_SHOULDER': 0x0100, 'RIGHT_SHOULDER': 0x0200,
    'A': 0x1000, 'B': 0x2000, 'X': 0x4000, 'Y': 0x8000,
}


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ('wButtons', wt.WORD),
        ('bLeftTrigger', wt.BYTE),
        ('bRightTrigger', wt.BYTE),
        ('sThumbLX', ctypes.c_short),
        ('sThumbLY', ctypes.c_short),
        ('sThumbRX', ctypes.c_short),
        ('sThumbRY', ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [('dwPacketNumber', wt.DWORD), ('Gamepad', XINPUT_GAMEPAD)]


def load_xinput():
    for name in ('xinput1_4.dll', 'xinput1_3.dll', 'xinput9_1_0.dll', 'xinput.dll'):
        try:
            return ctypes.WinDLL(name)
        except OSError:
            continue
    return None


def main():
    xi = load_xinput()
    if xi is None:
        print('找不到 XInput DLL，請確認是 Windows')
        return 1
    xi.XInputGetState.restype = wt.DWORD
    xi.XInputGetState.argtypes = [wt.DWORD, ctypes.POINTER(XINPUT_STATE)]

    connected = []
    for i in range(4):
        st = XINPUT_STATE()
        r = xi.XInputGetState(i, ctypes.byref(st))
        if r == ERROR_SUCCESS:
            connected.append(i)
            print(f'UserIndex {i}: >>> 已連接 (Xbox 手柄) <<<')
        else:
            print(f'UserIndex {i}: 未連接 (err={r})')

    if not connected:
        print()
        print('沒有偵測到任何 Xbox 手柄。請檢查：')
        print('  1) 有線：USB 線是否插好')
        print('  2) 無線：手柄電源是否開啟、是否已配對 (按一下 Xbox 鍵)')
        print('  3) 其他手把若走 DirectInput 不走 XInput，本腳本偵測不到')
        return 1

    idx = connected[0]
    print(f'\n讀取 UserIndex {idx} 狀態 6 秒，請隨意動搖桿 / 按按鈕 ...\n')
    t0 = time.time()
    while time.time() - t0 < 6:
        st = XINPUT_STATE()
        xi.XInputGetState(idx, ctypes.byref(st))
        g = st.Gamepad
        pressed = [n for n, b in BTN.items() if g.wButtons & b]
        print(f'BTN={str(pressed):30s} '
              f'LX={g.sThumbLX:6d} LY={g.sThumbLY:6d} '
              f'RX={g.sThumbRX:6d} RY={g.sThumbRY:6d} '
              f'LT={g.bLeftTrigger:3d} RT={g.bRightTrigger:3d}', flush=True)
        time.sleep(0.05)
    print('\n偵測完成')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
