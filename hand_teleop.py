# -*- coding: utf-8 -*-
"""
hand_teleop.py — 用筆電攝影機「手勢遙操」MyCobot320 (M5)

控制概念：絕對游標式（掌心 = 游標）
  - 畫面正中央 = 中性點(neutral)。掌心對上中心圓 → 自動校準並啟動。
  - 掌心相對中心的偏移，直接映射成機械臂末端「相對零位的絕對目標」：
      掌心左/右  -> 末端左/右平移 (Y)     掌心上/下 -> 末端上/下平移 (Z)
      掌心往鏡頭推/拉 -> 末端前進/後退 (X)   掌心滾轉(轉方向盤) -> 末端滾轉 (RZ)
  - 掌心回到中心 → 末端回到零位；全程不回讀積分、無漂移校正 → 動作即時且線性。

啟動流程：
  1) 對準鏡頭張開手掌
  2) 把掌心移進畫面中央的圓圈內並停留 ~0.6s → 自動校準(快照零位與深度/滾轉基準)
  3) 開始移動手掌遙操；掌心回中心即停止/回零

鍵盤：
  R       重設零位 = 機械臂「現在位置」變中性點（掌心在圓內會一併校準基準）
  1~4     翻轉 左右/上下/前後/滾轉 方向（鏡射校正）
  0       回到「啟動時的位置」(home)，並把零位移到 home
  Q/ESC   退出：倒數 EXIT_DELAY 秒(畫面顯示)後「所有關節回 0°(歸零)」再關閉
          倒數中：再按一次 Q/ESC = 立即歸零退出；按其他鍵 = 取消
          Ctrl+C 同為「歸零後關閉」(不再等倒數)

用法：
  python hand_teleop.py --preview        # 只預覽(不連機械臂)，校準方向與手感
  python hand_teleop.py --cam 0          # 正式搖操
  python hand_teleop.py --selftest       # 只測攝影機能不能開(無視窗)
"""
import argparse
import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

# ---------- 參數設定 ----------
CAM_ID = 0
DISPLAY_W = 960                     # 顯示寬度(等比例縮小)
MAX_HAND_LOST = 1.5                 # 手消失超過此秒數 → 解除啟動，需回中心重新校準

# 映射參數 (掌心偏移 → 末端位移，線性)
BOX_MM = 120.0                      # 左右/上下全行程(±)mm — 掌心到畫面邊緣 = 120mm
BOX_X_MM = 100.0                    # 前後(深度)全行程(±)mm
BOX_RZ_DEG = 110.0                  # 滾轉全行程(±)度
K_XY = BOX_MM / 0.5                 # mm per 畫面半寬
K_FB = BOX_X_MM / 0.55              # mm per log-ratio 0.55
K_RZ = BOX_RZ_DEG / 90.0            # 滾轉每 90° → 全行程

DEAD_XY = 0.045                     # 左右/上下死區(畫面比例 ±4.5%)
DEAD_FB_LN = 0.06                   # 深度死區(log size 差)
DEAD_RZ_DEG = 6.0                   # 滾轉死區(度)

ENGAGE_R = 0.13                     # 掌心要進入中心圓半徑(畫面比例)才算「對準」
ENGAGE_T = 0.6                      # 在圓內停留多久自動校準啟動(秒)

FEAT_ALPHA = 0.5                    # 手勢特徵平滑(越小越平滑/越遲鈍)
TGT_ALPHA = 0.5                     # 目標位置平滑(0~1，越小越滑順)
CTRL_DT = 0.05                      # 發送週期(秒) → ~20Hz
ARM_SPEED = 60                      # send_coords speed 1~100
MAX_STEP_MM = 12.0                  # 每 tick 單軸最大目標位移(mm) → ~240mm/s 上限，防暴衝
MAX_STEP_RZ = 10.0                  # 每 tick 滾轉最大目標變化(度)

# 關閉程序（退出時收尾：延遲 → 歸零 → 關閉）
EXIT_DELAY = 3.0                    # 按 Q/ESC 後倒數幾秒才開始歸零(畫面倒數可取消)
ZERO_SPEED = 40                     # 歸零 send_angles 速度 1~100
ZERO_WAIT_S = 25.0                  # 等待歸零到位的上限(秒)，超過仍關閉程式

# 軸向方向 (預設：鏡像畫面內「手往哪邊動，末端就往同方向」)
FLIP_LR = 1     # 手往右(畫面右) -> Y+
FLIP_UD = 1     # 手往上         -> Z+
FLIP_FB = 1     # 手往鏡頭推     -> X+ (往機械臂前方)
FLIP_RZ = 1     # 順時針滾轉(掌面朝鏡頭) -> RZ+


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_deg(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


# MyCobot320 的韌體/API 工作空間 (與 pymycobot RobotLimit 一致；單位 mm 與 deg)
WS_FALLBACK_LO = [-350.0, -350.0, -41.0, -180.0, -180.0, -180.0]
WS_FALLBACK_HI = [350.0, 350.0, 523.9, 180.0, 180.0, 180.0]


def load_robot_ws():
    """從 pymycobot 讀取 MyCobot320 官方座標上下限；讀不到就用內建值"""
    try:
        from pymycobot.error import RobotLimit
        d = RobotLimit.robot_limit.get('MyCobot320')
        if d and 'coords_min' in d and 'coords_max' in d:
            return ([float(x) for x in d['coords_min']],
                    [float(x) for x in d['coords_max']])
    except Exception:
        pass
    return (list(WS_FALLBACK_LO), list(WS_FALLBACK_HI))


def clamp_target(tgt, base, ws_lo, ws_hi,
                 box_mm=BOX_MM, box_rz_deg=BOX_RZ_DEG):
    """目標座標軟限位 = 「以零位(base)為中心的 ±box」 ∩ 「韌體真實工作空間」。
    缺一不可：只框 base 盒子會放寬到韌體不允許的區域(例如 Z>523.9 直接被拒)。"""
    t = list(tgt)
    for i in (0, 1, 2):                       # X / Y / Z
        t[i] = clamp(t[i],
                     max(base[i] - box_mm, ws_lo[i]),
                     min(base[i] + box_mm, ws_hi[i]))
    t[5] = clamp(t[5],                        # RZ(滾轉)
                 max(base[5] - box_rz_deg, ws_lo[5]),
                 min(base[5] + box_rz_deg, ws_hi[5]))
    return t


def feat_offsets(cx, cy, ln_size, roll_deg, ref_ln, ref_roll):
    """掌心特徵(正規化座標/對數大小/滾轉度) → 末端偏移量(mm, deg)。
    死區內回 0；超過死區後線性放大到全行程。回傳 (off_lr, off_ud, off_fb, off_rz)。"""
    def zd(off, dead):
        return 0.0 if abs(off) < dead else off

    dx = zd(cx - 0.5, DEAD_XY)                # 畫面右為正(比例)
    dy = zd(0.5 - cy, DEAD_XY)                # 畫面上為正
    off_lr = clamp(dx * K_XY, -BOX_MM, BOX_MM)
    off_ud = clamp(dy * K_XY, -BOX_MM, BOX_MM)

    dln = zd(ln_size - ref_ln, DEAD_FB_LN)    # 比校準時大 = 更靠近鏡頭
    off_fb = clamp(dln * K_FB, -BOX_X_MM, BOX_X_MM)

    rd = wrap_deg(roll_deg - ref_roll)
    rd = 0.0 if abs(rd) < DEAD_RZ_DEG else rd
    off_rz = clamp(rd * K_RZ, -BOX_RZ_DEG, BOX_RZ_DEG)
    return off_lr, off_ud, off_fb, off_rz


def target_from_offs(base, offs, flips, ws_lo, ws_hi):
    """零位 + 偏移 → 絕對目標座標(軟限位後)。offs = (off_lr, off_ud, off_fb, off_rz)"""
    t = [base[0] + flips[2] * offs[2],        # 深度 -> X
         base[1] + flips[0] * offs[0],        # 左右  -> Y
         base[2] + flips[1] * offs[1],        # 上下  -> Z
         base[3],
         base[4],
         base[5] + flips[3] * offs[3]]        # 滾轉  -> RZ
    return clamp_target(t, base, ws_lo, ws_hi)


def step_limit(cur, desired, max_step):
    """每 tick 最多移動 max_step，避免目標突跳造成暴衝"""
    out = list(cur)
    for i in range(6):
        d = desired[i] - cur[i]
        out[i] = cur[i] + clamp(d, -max_step, max_step)
    return out


# ---------- MediaPipe 手勢 ----------
class HandFeat:
    __slots__ = ('cx', 'cy', 'ln_size', 'roll_deg', 'landmarks', 'w', 'h')


MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
             'hand_landmarker/float16/1/hand_landmarker.task')
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'models', 'hand_landmarker.task')


def ensure_model():
    """回傳手部模型路徑；不存在就自動下載(約 7MB)"""
    if os.path.exists(MODEL_PATH):
        return MODEL_PATH
    os.makedirs(os.path.dirname(MODEL_PATH), exist_ok=True)
    print('首次使用：下載手部模型 hand_landmarker.task (約 7MB) ...', flush=True)
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    except Exception:
        if os.path.exists(MODEL_PATH):
            os.remove(MODEL_PATH)
        raise RuntimeError('手部模型下載失敗，請自行下載並放到:\n  ' + MODEL_PATH
                           + '\n下載網址: ' + MODEL_URL)
    print('模型已就緒', flush=True)
    return MODEL_PATH


class HandTracker:
    """把一幀畫面轉成手掌特徵；所有座標都基於「鏡像後」的畫面。

    使用新版 tasks API (HandLandmarker)，mediapipe 0.10.30+/1.x 通用，
    不再依賴已被移除的 mp.solutions.hands 舊 API。
    """

    def __init__(self):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision
        self._mp = mp
        base = mp_python.BaseOptions(model_asset_path=ensure_model())
        opts = vision.HandLandmarkerOptions(
            base_options=base,
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.hands = vision.HandLandmarker.create_from_options(opts)
        self._ts_ms = 0          # VIDEO 模式需要嚴格遞增的時間戳

    def process(self, frame_mirrored):
        """frame_mirrored: 已水平鏡像的 BGR 幀。回傳 HandFeat 或 None"""
        rgb = cv2.cvtColor(frame_mirrored, cv2.COLOR_BGR2RGB)
        img = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                             data=np.ascontiguousarray(rgb))
        self._ts_ms += 1
        res = self.hands.detect_for_video(img, self._ts_ms)
        h, w = frame_mirrored.shape[:2]
        if not res.hand_landmarks:
            return None
        lm = res.hand_landmarks[0]      # 21 個點，x/y 為正規化座標
        # 掌心 = 手腕(0) + 4 根手指根部 MCP(5,9,13,17) 的平均
        idx = [0, 5, 9, 13, 17]
        pts = [(lm[i].x, lm[i].y) for i in idx]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        # 深度代理：手腕→中指根部 的像素長度 (越大=越靠近鏡頭)
        dx_px = (lm[9].x - lm[0].x) * w
        dy_px = (lm[9].y - lm[0].y) * h
        size_px = math.hypot(dx_px, dy_px) + 1e-6
        # 滾轉：手腕→中指根部向量與「畫面正上方」的夾角
        roll = math.degrees(math.atan2(lm[9].x - lm[0].x, -(lm[9].y - lm[0].y)))
        f = HandFeat()
        f.cx, f.cy = cx, cy
        f.ln_size = math.log(size_px)
        f.roll_deg = roll
        f.landmarks = lm
        f.w, f.h = w, h
        return f

    def close(self):
        try:
            self.hands.close()
        except Exception:
            pass


# ---------- 機械臂 ----------
class RobotArm:
    def __init__(self, port=None):
        import serial.tools.list_ports as lp
        from pymycobot import MyCobot320

        if port is None:
            for p in lp.comports():
                if 'CH9102' in p.description or 'USB-Enhanced-SERIAL' in p.description:
                    port = p.device
                    break
        if port is None:
            raise RuntimeError('找不到 CH9102 串口(機械臂未插 USB？)')
        print(f'連接 {port} @ 115200 ...', flush=True)
        self.mc = MyCobot320(port, 115200, timeout=0.5, debug=False)
        # 只執行最新指令，避免遙操指令堆積
        try:
            self.mc.set_fresh_mode(1)
        except Exception:
            pass
        time.sleep(0.3)
        # 舵機上電
        try:
            if self.mc.is_power_on() == 0:
                print('舵機未上電，power_on() ...', flush=True)
                self.mc.power_on()
                time.sleep(1.0)
        except Exception as e:
            print('power_on 檢查失敗:', e, flush=True)
        self.home = self.read_coords()
        if self.home is None:
            raise RuntimeError('讀不到機械臂當前座標，請確認螢幕 Atom: ok')
        print(f'啟動位置(home): {[round(v, 1) for v in self.home]}', flush=True)

        # 韌體真實工作空間 (與 pymycobot 發送前檢查同一來源)
        self.ws_lo, self.ws_hi = load_robot_ws()

        # 絕對游標式控制：base = 中性點對應的機械臂零位
        # 掌心在畫面中心 → 目標 = base；掌心偏移 → 目標 = base + 映射偏移
        # 全程不回讀、不積分 → 無漂移、動作即時
        self.base = list(self.home)
        self._last_sent = list(self.home)

        # 錯誤節流狀態
        self._err_count = 0
        self._last_err_t = 0.0

        # 若 home 貼近工作空間邊界 → 警告 (否則該方向幾乎不能動)
        near = []
        for i, name in ((0, 'X'), (1, 'Y'), (2, 'Z')):
            if self.home[i] < self.ws_lo[i] + 25.0:
                near.append(f'{name} 已貼近下限 {self.ws_lo[i]:.0f} ({self.home[i]:.0f})')
            if self.home[i] > self.ws_hi[i] - 25.0:
                near.append(f'{name} 已貼近上限 {self.ws_hi[i]:.0f} ({self.home[i]:.0f})')
        if abs(self.home[5]) > 170:
            near.append('RZ 已貼近 ±180° 邊界')
        if near:
            print('⚠ 注意：' + '；'.join(near) + '。'
                  '該方向可用行程很少，建議先把機械臂移到工作空間中段再啟動。', flush=True)

    def read_coords(self):
        try:
            c = self.mc.get_coords()
        except Exception:
            return None
        if not c or len(c) != 6 or c[0] == -1:
            return None
        return [float(v) for v in c]

    def set_base(self, coords=None):
        """把「現在位置」設為中性點零位。回傳新 base 或 None。"""
        c = coords or self.read_coords()
        if c is None:
            return None
        self.base = list(c)
        self._last_sent = list(c)
        print(f'✔ 零位已重設 = {[round(v, 1) for v in self.base]} '
              '(掌心對上中心圓即為此位置)', flush=True)
        return self.base

    def send_target(self, tgt):
        """發送絕對目標(mode=0 非阻塞 + fresh mode)。目標幾乎沒變就略過。"""
        t = [round(v, 2) for v in tgt]
        if all(abs(t[i] - self._last_sent[i]) < 0.3 for i in (0, 1, 2, 5)):
            return False
        try:
            self.mc.send_coords(t, ARM_SPEED, mode=0)
        except Exception as e:
            # 節流：同樣的錯誤最多約每 2 秒印一次，避免刷屏
            self._err_count += 1
            now_t = time.time()
            if now_t - self._last_err_t > 2.0:
                print(f'send_coords 失敗 x{self._err_count} (每2秒顯示一次): {e}',
                      flush=True)
                self._last_err_t = now_t
            return False
        self._err_count = 0
        self._last_sent = t
        return True

    def goto_home(self):
        print('回到啟動位置 ...', flush=True)
        try:
            self.mc.send_coords([round(v, 2) for v in self.home], ARM_SPEED)
        except Exception as e:
            print('回位失敗:', e, flush=True)
        self.base = list(self.home)
        self._last_sent = list(self.home)

    def zero_all(self):
        """關閉程序：所有關節送回 0°(J1~J6 = 0) 並輪詢等到位。
        回傳 True=已歸零到位, False=發送失敗或等待超時(仍關閉)。"""
        print('關節歸零中 (J1~J6 → 0°) ...', flush=True)
        try:
            self.mc.send_angles([0.0] * 6, ZERO_SPEED)
        except Exception as e:
            print('歸零指令發送失敗:', e, flush=True)
            return False
        t0 = time.time()
        while time.time() - t0 < ZERO_WAIT_S:
            try:
                a = self.mc.get_angles()
            except Exception:
                a = -1
            if a not in (-1, None) and len(a) == 6 \
                    and all(abs(float(x)) <= 2.0 for x in a):
                print('✔ 已歸零 (J1~J6 = 0°)', flush=True)
                return True
            time.sleep(0.2)
        print('⚠ 等待歸零超時(未完全到位)，仍關閉程式', flush=True)
        return False

    def close(self):
        try:
            self.mc.close()
        except Exception:
            pass


# ---------- 主程式 ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--preview', action='store_true', help='只做視覺預覽，不連機械臂')
    ap.add_argument('--cam', type=int, default=CAM_ID, help='攝影機 index')
    ap.add_argument('--port', default=None, help='機械臂 COM 口(預設自動偵測)')
    ap.add_argument('--selftest', action='store_true', help='無視窗測試攝影機')
    args = ap.parse_args()

    # ---- 開攝影機 ----
    cap = None
    tried = []
    for idx in [args.cam] + [i for i in range(5) if i != args.cam]:
        c = cv2.VideoCapture(idx)
        if c.isOpened():
            cap = c
            print(f'使用攝影機 index {idx}', flush=True)
            break
        c.release()
        tried.append(idx)
    if cap is None:
        print(f'攝影機打不開(試過 index {tried})', flush=True)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if args.selftest:
        ok = 0
        for _ in range(15):
            r, frame = cap.read()
            if r and frame is not None:
                ok += 1
        print(f'selftest: 讀到 {ok}/15 幀 -> {"OK" if ok > 10 else "FAIL"}')
        cap.release()
        sys.exit(0 if ok > 10 else 1)

    tracker = HandTracker()
    arm = None
    if not args.preview:
        try:
            arm = RobotArm(args.port)
        except Exception as e:
            print('機械臂初始化失敗:', e, flush=True)
            tracker.close()
            cap.release()
            sys.exit(1)

    # ---- 狀態 ----
    ws_lo, ws_hi = (arm.ws_lo, arm.ws_hi) if arm else load_robot_ws()
    base_coords = list(arm.base) if arm else [0, 0, 0, 0, 0, 0]

    engaged = False              # 已校準啟動(掌心在中心圓停留足夠久)
    engage_t0 = None             # 開始對準中心圓的時間
    ref_ln = None                # 校準時的手掌 log-size(深度基準)
    ref_roll = None              # 校準時的滾轉基準
    feats = None                 # EMA 平滑後特徵 [cx, cy, ln_size, roll]
    tgt_ema = list(base_coords)  # 目標位置平滑值
    last_lost_t = None
    need_recenter = False          # 手追丟>0.3s → 需回中心圓才恢復動作
    show_help = True
    flip = [FLIP_LR, FLIP_UD, FLIP_FB, FLIP_RZ]
    t_prev = time.time()
    last_cmd_t = 0.0
    fps = 0.0
    last_offs = (0.0, 0.0, 0.0, 0.0)
    exiting = False                  # Q/ESC 後進入關閉倒數(不再送指令)
    exit_deadline = 0.0              # 倒數截止時間(now)

    print('\n===== 手勢搖操 (絕對游標式) =====', flush=True)
    print('將手掌張開對準鏡頭，把掌心移進畫面中央的圓圈內停留一下即自動啟動。',
          flush=True)
    print('鍵盤: R=重設零位  1~4=翻轉方向  0=回啟動位置  '
          f'Q/ESC=退出({EXIT_DELAY:.0f}秒後歸零,倒數中可取消)', flush=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            # 鏡像顯示 → 直覺(像照鏡子)
            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            disp = cv2.resize(frame, (DISPLAY_W, int(DISPLAY_W * h / w)))

            now = time.time()
            dt = max(0.02, min(0.5, now - t_prev))
            t_prev = now

            f = tracker.process(frame)   # 用原始尺寸算，避免縮放誤差
            hand_here = f is not None

            # ---- 特徵平滑 ----
            if hand_here:
                vec = [f.cx, f.cy, f.ln_size, f.roll_deg]
                if feats is None:
                    feats = vec[:]
                else:
                    for i in range(4):
                        feats[i] = FEAT_ALPHA * vec[i] + (1 - FEAT_ALPHA) * feats[i]
                last_lost_t = None
            elif feats is not None:
                if last_lost_t is None:
                    last_lost_t = now
                if now - last_lost_t > 0.3:
                    need_recenter = True        # 追丟>0.3s：回到中心圓才恢復，防暴衝
                if now - last_lost_t > MAX_HAND_LOST:
                    feats = None
                    engaged = False
                    engage_t0 = None
                    need_recenter = True

            # ---- 啟動狀態機：掌心對上中心圓才開始 ----
            dist_center = None
            if feats is not None:
                dist_center = math.hypot(feats[0] - 0.5, feats[1] - 0.5)
            if hand_here and need_recenter and dist_center is not None \
                    and dist_center < ENGAGE_R:
                need_recenter = False
                print('✔ 掌心已回中心圓，恢復搖操', flush=True)
            if not engaged and feats is not None:
                if dist_center is not None and dist_center < ENGAGE_R:
                    if engage_t0 is None:
                        engage_t0 = now
                    if now - engage_t0 >= ENGAGE_T:
                        # 校準：快照深度/滾轉基準；機械臂零位 = 現在位置(不動才不跳)
                        ref_ln = feats[2]
                        ref_roll = feats[3]
                        if arm is not None:
                            arm.set_base()
                            base_coords = list(arm.base)
                            tgt_ema = list(base_coords)
                        engaged = True
                        engage_t0 = None
                        need_recenter = False
                        print('✔ 校準完成，開始搖操 (掌心回中心圓即回到零位)',
                              flush=True)
                else:
                    engage_t0 = None
            elif engaged and feats is None and last_lost_t is not None \
                    and now - last_lost_t > MAX_HAND_LOST:
                engaged = False

            # ---- 偏移映射 → 絕對目標 (僅在掌心可見、不需回中心、未在關閉倒數時) ----
            if engaged and hand_here and not need_recenter and not exiting:
                offs = feat_offsets(feats[0], feats[1], feats[2],
                                    feats[3], ref_ln, ref_roll)
                last_offs = offs
                desired = target_from_offs(base_coords, offs, flip, ws_lo, ws_hi)
                # 目標平滑 + 每 tick 限步 → 動作連續不暴衝
                for i in range(6):
                    desired[i] = step_limit(tgt_ema, desired, MAX_STEP_MM if i < 3
                                            else (MAX_STEP_RZ if i == 5 else 0.0))[i]
                    tgt_ema[i] = TGT_ALPHA * desired[i] + (1 - TGT_ALPHA) * tgt_ema[i]

            # ---- 發送機械臂：~20Hz 絕對目標流 (掌心可見、不需回中心才發) ----
            if (arm is not None and engaged and hand_here and not need_recenter
                    and not exiting
                    and now - last_cmd_t >= CTRL_DT - 1e-4):
                arm.send_target(tgt_ema)
                last_cmd_t = now

            # ---- 繪製 ----
            scale = disp.shape[1] / w
            overlay = disp.copy()
            cx_px = int(0.5 * disp.shape[1])
            cy_px = int(0.5 * disp.shape[0])
            # 中心十字(固定畫面中央)
            cv2.drawMarker(overlay, (cx_px, cy_px), (0, 255, 0),
                           cv2.MARKER_CROSS, 26, 2)
            # 校準圈(掌心需進入)
            r_eng = int(ENGAGE_R * disp.shape[1])
            eng_col = (0, 200, 255) if not engaged else (0, 220, 0)
            cv2.circle(overlay, (cx_px, cy_px), max(r_eng, 20), eng_col, 1, cv2.LINE_AA)
            # 死區圈
            r_dead = int(DEAD_XY * disp.shape[1])
            cv2.circle(overlay, (cx_px, cy_px), max(r_dead, 10), (80, 80, 80),
                       1, cv2.LINE_AA)
            if feats is not None:
                px, py = int(feats[0] * disp.shape[1]), int(feats[1] * disp.shape[0])
                cv2.line(overlay, (cx_px, cy_px), (px, py), (0, 200, 255), 2, cv2.LINE_AA)
                cv2.circle(overlay, (px, py), 8, (0, 255, 255), -1)
            if f is not None:
                lm = f.landmarks
                for a, b in HAND_CONNS:
                    x1, y1 = int(lm[a].x * disp.shape[1]), int(lm[a].y * disp.shape[0])
                    x2, y2 = int(lm[b].x * disp.shape[1]), int(lm[b].y * disp.shape[0])
                    cv2.line(overlay, (x1, y1), (x2, y2), (255, 120, 120), 2)
                for l in lm:
                    cv2.circle(overlay, (int(l.x * disp.shape[1]),
                                         int(l.y * disp.shape[0])), 3, (120, 255, 120), -1)

            # 狀態文字
            if engaged and need_recenter:
                st_txt = '追丟 - 請把手掌放回中央圓圈以恢復'
                st_col = (60, 120, 255)
            elif engaged:
                st_txt = 'READY - 搖操中'
                st_col = (0, 220, 0)
            elif feats is not None and dist_center is not None and dist_center < ENGAGE_R:
                st_txt = '對準中... 停在圓內即可啟動'
                st_col = (0, 200, 255)
            else:
                st_txt = '請把手掌移到中央圓圈內'
                st_col = (60, 60, 255)
            cv2.putText(overlay, st_txt, (14, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, st_col, 2, cv2.LINE_AA)
            if feats is not None:
                cv2.putText(overlay,
                            'off LR %+5.0f  UD %+5.0f  FB %+5.0f mm   RZ %+5.0f deg'
                            % (last_offs[0], last_offs[1], last_offs[2], last_offs[3]),
                            (14, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(overlay,
                            f'flip LR/UD/FB/RZ = {flip[0]:+d} {flip[1]:+d} {flip[2]:+d} {flip[3]:+d}',
                            (14, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (200, 200, 200), 1, cv2.LINE_AA)
            if arm is not None:
                cv2.putText(overlay,
                            'TCP [%5.0f %5.0f %5.0f  rz%5.0f]  base[%5.0f %5.0f %5.0f]'
                            % (tgt_ema[0], tgt_ema[1], tgt_ema[2], tgt_ema[5],
                               base_coords[0], base_coords[1], base_coords[2]),
                            (14, disp.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 0), 1, cv2.LINE_AA)
            if exiting:
                left = max(0.0, exit_deadline - now)
                cv2.putText(overlay,
                            f'即將關閉: {left:.1f} 秒後關節歸零 (再按 Q/ESC 立即, 其他鍵取消)',
                            (14, disp.shape[0] - 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (60, 60, 255), 2, cv2.LINE_AA)
            elif args.preview:
                cv2.putText(overlay, 'PREVIEW (no robot)',
                            (14, disp.shape[0] - 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 255), 2, cv2.LINE_AA)
            elif not engaged and show_help:
                cv2.putText(overlay, 'put palm into the circle to start',
                            (14, disp.shape[0] - 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 1, cv2.LINE_AA)
            fps = 0.9 * fps + 0.1 / max(dt, 1e-3)
            cv2.putText(overlay, f'{fps:.0f} fps', (disp.shape[1] - 90, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1, cv2.LINE_AA)
            cv2.imshow('hand teleop (mirror view)', overlay)

            # ---- 鍵盤 ----
            k = cv2.waitKey(1) & 0xFF
            if exiting:
                # 倒數中：再按 Q/ESC = 立即歸零退出；按其他鍵 = 取消關閉
                if k == 255:                     # 沒按鍵
                    if now >= exit_deadline:
                        print('倒數結束, 歸零關閉 ...', flush=True)
                        break
                elif k in (ord('q'), 27):
                    print('確認關閉, 立即歸零退出', flush=True)
                    break
                else:
                    exiting = False
                    exit_deadline = 0.0
                    print('已取消關閉, 繼續搖操', flush=True)
            elif k in (ord('q'), 27):
                exiting = True
                exit_deadline = now + EXIT_DELAY
                print(f'\n準備關閉: {EXIT_DELAY:.0f} 秒後所有關節歸零並退出。'
                      '\n  再按一次 Q/ESC = 立即歸零退出；按其他鍵 = 取消', flush=True)
            elif k in (ord('r'), ord('R')):
                # 重設零位：機械臂現在位置 = 中性點；掌心在場一併校準基準
                if arm is not None:
                    if arm.set_base() is not None:
                        base_coords = list(arm.base)
                        tgt_ema = list(base_coords)
                if hand_here and f is not None:
                    ref_ln = f.ln_size
                    ref_roll = f.roll_deg
                engaged = True
                engage_t0 = None
                need_recenter = False
                print('✔ 已重設零位/基準', flush=True)
            elif k in (ord('0'),):
                if arm is not None:
                    arm.goto_home()
                    base_coords = list(arm.home)
                    tgt_ema = list(base_coords)
                    if hand_here and f is not None:
                        ref_ln = f.ln_size
                        ref_roll = f.roll_deg
                    print('✔ 已回啟動位置並設為零位', flush=True)
            elif ord('1') <= k <= ord('4'):
                flip[k - ord('1')] *= -1
                print(f'軸向翻轉 → {flip}', flush=True)
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，準備關閉(所有關節歸零)...', flush=True)
    finally:
        if arm is not None:
            print('\n關閉程序：所有關節歸零後關閉 ...', flush=True)
            arm.zero_all()
            arm.close()
        tracker.close()
        cap.release()
        cv2.destroyAllWindows()
    print('已退出', flush=True)


# MediaPipe 手部 21 點骨架連線 (與官方 HAND_CONNECTIONS 一致，
# 寫死避免依賴已被移除的 mp.solutions 舊 API)
HAND_CONNS = [
    (0, 1), (1, 2), (2, 3), (3, 4),         # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),         # 食指
    (5, 9), (9, 10), (10, 11), (11, 12),    # 中指
    (9, 13), (13, 14), (14, 15), (15, 16),  # 無名指
    (13, 17), (17, 18), (18, 19), (19, 20), # 小指
    (0, 17),                                # 手掌
]


if __name__ == '__main__':
    main()
