#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import time

class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')
        
        # 接收代驾司机的“油门”和“方向盘”指令
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        # 向系统汇报自己的位置
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # 在 RViz 中画出自己
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        # 初始坐标 (1.0, 1.0)，在地图左下角安全区
        self.x = 1.0
        self.y = 1.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()

        # 50Hz 刷新率更新物理引擎
        self.timer = self.create_timer(0.02, self.update_physics)
        self.get_logger().info("RViz虚拟机器人引擎已启动")

    def cmd_cb(self, msg):
        """接收速度指令"""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_physics(self):
        """核心：积分推算运动学模型"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        # 发布位置信息 (供寻路追踪使用)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

        # 发送 3D 机器人模型给 RViz (一个亮黄色的作业底盘)
        marker = Marker()
        marker.header = odom.header
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = odom.pose.pose
        marker.scale.x = 0.6  # 长宽 0.6 米
        marker.scale.y = 0.6
        marker.scale.z = 0.2  # 高度 0.2 米
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 1.0  # 红色拉满
        marker.color.g = 0.8  # 绿色加一点变成黄色
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
