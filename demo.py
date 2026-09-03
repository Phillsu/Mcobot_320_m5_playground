from pymycobot.mycobot320 import MyCobot320
import time
import serial.tools.list_ports as lp

# 以上需写在代码开头，意为导入项目包

# MyCobot320 类初始化需要两个参数：串口和波特率

# 初始化一个MyCobot320对象
# 下面为 mycobot-raspi 版本创建对象代码
BAUD = 115200


def find_robot_port():
    for p in lp.comports():
        if 'CH9102' in p.description or 'USB-Enhanced-SERIAL' in p.description:
            return p.device
    return None


PORT = find_robot_port() or 'COM3'
print(f'使用串口: {PORT}')
mc = MyCobot320(PORT, BAUD, timeout=0.5, debug=True)

i = 7
# 循环7次
while i > 0:
    mc.set_color(0,0,255) # 蓝灯亮
    time.sleep(2)    # 等2秒
    mc.set_color(255,0,0) # 红灯亮
    time.sleep(2)    # 等2秒
    mc.set_color(0,255,0) # 绿灯亮
    time.sleep(2)    # 等2秒
    i -= 1