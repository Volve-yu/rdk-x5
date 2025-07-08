#!/usr/bin/env python3
import collections
from collections.abc import MutableMapping
collections.MutableMapping = MutableMapping
from dronekit import connect, VehicleMode
import time


# 对应你的串口设备和波特率
connection_string = '/dev/ttyUSB0'  # 替换为你的设备路径
baud_rate = 921600

print("Connecting to vehicle on: %s @ %d" % (connection_string, baud_rate))
vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)

# 串口设备和波特率配置
connection_string = '/dev/ttyUSB0'  # 根据实际情况修改
baud_rate = 921600

print(f"Connecting to vehicle on: {connection_string} @ {baud_rate}")
vehicle = connect(connection_string, wait_ready=False, baud=baud_rate)

print("\n==== Vehicle Attribute Values ====\n")
print(f"Autopilot Firmware version: {vehicle.version}")
print(f"Autopilot capabilities (supports ftp): {vehicle.capabilities.ftp}")
print(f"Global Location: {vehicle.location.global_frame}")
print(f"Global Location (relative altitude): {vehicle.location.global_relative_frame}")
print(f"Local Location (NED): {vehicle.location.local_frame}")
print(f"Attitude: {vehicle.attitude}")
print(f"Velocity: {vehicle.velocity}")
print(f"GPS: {vehicle.gps_0}")
print(f"Groundspeed: {vehicle.groundspeed}")
print(f"Airspeed: {vehicle.airspeed}")
print(f"Gimbal status: {vehicle.gimbal}")
print(f"Battery: {vehicle.battery}")
print(f"EKF OK?: {vehicle.ekf_ok}")
print(f"Last Heartbeat: {vehicle.last_heartbeat}")
print(f"Rangefinder: {vehicle.rangefinder}")
print(f"Rangefinder distance: {vehicle.rangefinder.distance}")
print(f"Rangefinder voltage: {vehicle.rangefinder.voltage}")
print(f"Heading: {vehicle.heading}")
print(f"Is Armable?: {vehicle.is_armable}")
print(f"System status: {vehicle.system_status.state}")
print(f"Mode: {vehicle.mode.name}")    # 可读写
print(f"Armed: {vehicle.armed}")       # 可读写

# 最后断开
print("Closing vehicle connection")
vehicle.close()
