#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math

def euler_from_quaternion(x, y, z, w):
    """将四元数转换为欧拉角（偏航角 yaw）"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        self.path = []
        self.current_target_idx = 0
        self.current_pose = None
        
        # 订阅绿线路径和底盘当前位置
        self.sub_path = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 发布电机控制指令
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 100ms 控制周期
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('自动驾驶追踪节点已启动！等待 RViz 下发路径...')

    def path_callback(self, msg):
        # 提取绿线上的所有坐标点
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_target_idx = 0
        self.get_logger().info('收到新路径，挂挡起步！')

    def odom_callback(self, msg):
        # 获取小车真实的 X, Y 和 朝向角
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_pose = (x, y, yaw)

    def control_loop(self):
        cmd = Twist()
        
        # 如果没有路径，或者已经到达终点，就停车
        if not self.path or self.current_pose is None or self.current_target_idx >= len(self.path):
            self.cmd_pub.publish(cmd)
            return

        cx, cy, cyaw = self.current_pose
        tx, ty = self.path[self.current_target_idx]

        # 计算距离和角度偏差
        dx = tx - cx
        dy = ty - cy
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        # 容差：距离目标点小于 0.3 米就算到达该网格，切换下一个点
        if distance < 0.3:
            self.current_target_idx += 1
            return

        # 计算最少旋转角度 [-pi, pi]
        yaw_error = target_yaw - cyaw
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        # 简单的 PID 比例控制：角度偏差大就原地转，角度对准了就往前开
        if abs(yaw_error) > 0.2: 
            cmd.angular.z = 1.0 if yaw_error > 0 else -1.0
            cmd.linear.x = 0.05
        else:
            cmd.angular.z = yaw_error * 3
            cmd.linear.x = 2.0  # 全速前进！

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
