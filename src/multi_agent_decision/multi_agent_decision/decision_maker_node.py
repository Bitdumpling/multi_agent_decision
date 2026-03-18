#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class DecisionMakerNode(Node):
    def __init__(self):
        super().__init__('decision_maker_node')
        
        # 1. 订阅环境上下文：预警地图和目标作业点
        self.map_sub = self.create_subscription(OccupancyGrid, '/warning_map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.decision_callback, 10)
        
        # 2. 发布控制指令：向底层硬件驱动节点下发作业方案
        self.cmd_pub = self.create_publisher(String, '/execution_command', 10)
        
        self.grid = None
        self.map_width = 0
        self.map_height = 0
        
        self.get_logger().info('终极信息决策节点已上线！随时准备下发精准治理指令...')

    def map_callback(self, msg):
        """保持预警地图的最新状态"""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.grid = msg.data

    def decision_callback(self, msg):
        """核心决策逻辑：到达目标点后该干什么？"""
        if not self.grid:
            self.get_logger().warn('缺少预警地图数据，无法做出安全决策！')
            return
            
        target_x = int(msg.pose.position.x)
        target_y = int(msg.pose.position.y)
        
        if not (0 <= target_x < self.map_width and 0 <= target_y < self.map_height):
            return # 超出范围的错误已在规划节点处理，这里直接跳过
            
        # 获取目标点的危险代价值 (Cost: 0-100)
        cost = self.grid[target_y * self.map_width + target_x]
        
        # 状态机：基于危害等级输出推荐治理方案
        decision_msg = String()
        
        self.get_logger().info(f'-----------------------------------')
        self.get_logger().info(f'执行机器人已接收目标靶区: ({target_x}, {target_y})')
        self.get_logger().info(f'目标区域危害代价: {cost}/100')
        
        if cost >= 80:
            plan = "【重度治理】启动重型物理机械臂强力连根清除，并辅以高浓度生态除草剂定点深层喷洒！"
        elif cost >= 60:
            plan = "【中度治理】展开常规除草切割组件，执行大面积切割，随后进行中浓度药剂覆盖。"
        elif cost >= 30:
            plan = "【轻度治理】无需物理破坏，直接开启喷洒云台，进行低浓度生态药剂预防性喷雾。"
        elif cost >= 0:
            plan = "【日常巡检】目标为安全本地植被区，保持机动待命，开启摄像头记录生态数据。"
        else:
            plan = "【异常】未知区域，停止行进并请求人工介入！"
            
        decision_msg.data = plan
        
        # 模拟发布给底层的控制指令
        self.cmd_pub.publish(decision_msg)
        
        self.get_logger().info(f'下发精准作业指令: {plan}')
        self.get_logger().info(f'-----------------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
