import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil


class Nav2MissionPlanner(Node):
    def __init__(self):
        super().__init__('nav2_mission_planner')
        # --- 参数 & DroneKit 连接 ---
        self.declare_parameter('connection_string', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        # 新增串口参数
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('serial_baud', 115200)

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_yaw', 0.0)

        # 读取参数
        conn_str   = self.get_parameter('connection_string').get_parameter_value().string_value
        baud       = self.get_parameter('baud_rate').get_parameter_value().integer_value
        serial_dev = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_baud= self.get_parameter('serial_baud').get_parameter_value().integer_value

        # 打开串口，用于向另一树莓派发送 '@'
        self.get_logger().info(f'Opening serial {serial_dev} @ {serial_baud}')
        try:
            self.serial = serial.Serial(serial_dev, serial_baud, timeout=1)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial = None

        # 目标点
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.goal.pose.position.x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal.pose.position.y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal.pose.orientation.z = self.get_parameter('goal_yaw').get_parameter_value().double_value
        self.goal.pose.orientation.w = 1.0

        # 连接飞控
        self.get_logger().info(f'Connecting to vehicle on: {conn_str} @ {baud}')
        self.vehicle = connect(conn_str, wait_ready=True, baud=baud)

        # Action Client，用于按需请求 Nav2 全局规划
        self._path_cli = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.waypoints = []      # 存放当前全局 Path 拆解后的列表
        self._idx = 0            # 当前正在执行的航点索引
        self._busy = False       # 标记“正在飞往某航点”

        # 定时检查是否到达当前航点
        self.create_timer(1.0, self._check_reached)

        # 启动流程
        self._request_next_plan()

    def _send_at_signal(self):
        """通过串口发送 '@' 给另一树莓派"""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn("Serial port not open, can't send '@'")
            return
        try:
            self.serial.write(b'@')
            self.get_logger().info("Sent '@' over serial")
        except Exception as e:
            self.get_logger().error(f"Failed to send '@' over serial: {e}")

    def _request_next_plan(self):
        """调用 Action 去请求从当前位置到最终 goal 的全局路径"""
        if self._busy:
            return
        self.get_logger().info('Requesting global path...')
        if not self._path_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Path action server not available!')
            return
        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = self.goal
        send_goal = self._path_cli.send_goal_async(goal_msg)
        send_goal.add_done_callback(self._on_path_response)
        self._busy = True

    def _on_path_response(self, future):
        """收到全局 Path 后，拆解成点列表，并上传第一个航点"""
        result = future.result().result
        path: Path = result.path
        self.get_logger().info(f'Received Path with {len(path.poses)} points')
        self.waypoints = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in path.poses
        ]
        self._idx = 0
        self._busy = False
        if self.waypoints:
            self._upload_and_start(self.waypoints[self._idx])

    def _upload_and_start(self, wp):
        """上传单点航点并切换到 AUTO 模式执行"""
        lat, lon, alt = wp
        cmds = self.vehicle.commands
        cmds.clear()

        # 起点：当前位置
        home = self.vehicle.location.global_relative_frame
        cmds.add(Command(
            0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            home.lat, home.lon, home.alt
        ))

        # 目标点
        cmds.add(Command(
            0,0,0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,0,0,0,0,0,
            lat, lon, alt
        ))

        self.get_logger().info(f'Uploading WP {self._idx+1}/{len(self.waypoints)}: ({lat},{lon},{alt})')
        cmds.upload()

        # 切到 AUTO 并解锁
        self.vehicle.mode = VehicleMode('AUTO')
        self.vehicle.armed = True
        self.get_logger().info('Armed & AUTO mode set')
        self._busy = True

    def _check_reached(self):
        """
        定时检查 vehicle.commands.next，判断当前航点是否到达；
        到达后发送 '@' 串口信号，延时 8 秒，再执行下一步。
        """
        if not self._busy:
            return

        next_wp = self.vehicle.commands.next
        # next_wp > 1 表示已飞过第一个航点
        if next_wp > 1:
            self.get_logger().info(f'Waypoint {self._idx+1} reached')
            self._busy = False

            # 1) 发送 '@'
            self._send_at_signal()

            # 2) 8 秒后启动下一步 —— 用一次性定时器避免阻塞
            def delayed_callback():
                # 取消此定时器
                timer.cancel()
                # 上传或重新规划
                self._idx += 1
                if self._idx < len(self.waypoints):
                    self._upload_and_start(self.waypoints[self._idx])
                else:
                    self.get_logger().info('All WPs done, requesting new plan')
                    self._request_next_plan()

            timer = self.create_timer(8.0, delayed_callback)

    def destroy(self):
        self.get_logger().info('Shutting down: closing vehicle & serial')
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass
        self.vehicle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
