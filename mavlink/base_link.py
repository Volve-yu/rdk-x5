import serial
import time
import argparse

# 串口配置参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况修改
BAUD_RATE = 115200            # 与固件匹配的波特率
TIMEOUT = 1                   # 串口读写超时时间（秒）

# 初始化串口
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
# 延时以等待串口稳定
time.sleep(2)

# 发送命令并打印执行结果
def send_command(cmd: str):

    if not ser.is_open:
        raise serial.SerialException("串口未打开")

    data = f"{cmd}\n".encode('utf-8')
    print(f"Sending: {cmd}")
    ser.write(data)
    # 读取并打印一行响应
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    if response:
        print(f"Response: {response}")

# 各种移动指令接口
def move_forward(distance: float):
    send_command(f"F{distance}")

def move_backward(distance: float):
    send_command(f"B{distance}")

def turn_left(angle: float):
    send_command(f"L{angle}")

def turn_right(angle: float):
    send_command(f"R{angle}")

def stop():
    send_command("S")



move_forward(0.5)  # 向前移动0.5米



