#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time

class RiskAssessmentNode(Node):
    def __init__(self):
        super().__init__('risk_assessment_node')
        
        # 订阅感知数据，格式升级为：[X坐标, Y坐标, 种类危害, 面积占比, 生态破坏]
        self.perception_sub = self.create_subscription(
            Float32MultiArray, '/perception_data', self.perception_callback, 10)
        
        self.risk_level_pub = self.create_publisher(String, '/risk_level', 10)
        self.warning_map_pub = self.create_publisher(OccupancyGrid, '/warning_map', 10)
        
        # 初始化一个 20x20 的局部预警地图（分辨率 1米/格）
        self.map_width = 20
        self.map_height = 20
        self.map_data = [-1] * (self.map_width * self.map_height)  # -1 表示该区域未知
        
        # 创建定时器，每 1 秒发布一次最新的预警地图
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('动态风险评估节点已升级！支持坐标映射与预警地图实时发布。')

    def perception_callback(self, msg):
        """处理感知数据并更新地图"""
        if len(msg.data) < 5:
            self.get_logger().warn('数据维度不足！需包含 [x, y, 种类, 面积, 生态]')
            return
            
        x = int(msg.data[0])
        y = int(msg.data[1])
        c_species = msg.data[2]
        c_area = msg.data[3]
        c_eco = msg.data[4]
        
        # AHP 算法计算得分
        risk_score = 0.4 * c_species + 0.3 * c_area + 0.3 * c_eco
        hazard_level = self.evaluate_hazard_level(risk_score)
        
        # 核心：更新地图数据
        # 确保坐标在地图范围内
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            index = y * self.map_width + x
            # 将 0-10 分的风险值映射为 ROS 地图的 0-100 代价 (0=畅通, 100=致命危险)
            cost = int((risk_score / 10.0) * 100)
            self.map_data[index] = cost
            self.get_logger().info(f'更新坐标 ({x}, {y}) -> 得分: {risk_score:.2f} ({hazard_level}), 地图代价: {cost}')

    def evaluate_hazard_level(self, score):
        if score >= 8.0: return "极高"
        elif score >= 6.0: return "高"
        elif score >= 3.0: return "中"
        else: return "低"

    def publish_map(self):
        """将地图数组封装为 OccupancyGrid 消息并发布"""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'  # 绑定全局坐标系
        grid.info.resolution = 1.0
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        # 设置地图原点
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.data = self.map_data
        
        self.warning_map_pub.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = RiskAssessmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
