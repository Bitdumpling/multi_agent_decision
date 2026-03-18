#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class TaskPlanningNode(Node):
    def __init__(self):
        super().__init__('task_planning_node')
        
        # 订阅预警地图和目标点 (使用 RViz 的 2D Goal Pose 工具)
        self.map_sub = self.create_subscription(OccupancyGrid, '/warning_map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 发布规划好的路径
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        self.grid = None
        self.map_width = 0
        self.map_height = 0
        # 假设机器人的初始位置在安全区的左下角 (1, 1)
        self.start_pos = (1, 1) 
        
        self.get_logger().info('任务规划节点已启动！请在 RViz 中使用 [2D Goal Pose] 工具点击目标点。')

    def map_callback(self, msg):
        """保存最新的预警地图数据供寻路使用"""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.grid = msg.data
        self.get_logger().info('成功接收预警地图，等待下发任务指令...', once=True)

    def goal_callback(self, msg):
        """接收 RViz 传来的目标点并开始规划"""
        if not self.grid:
            self.get_logger().warn('尚未接收到预警地图，无法规划！')
            return
            
        # 将实际坐标转换为网格索引
        goal_x = int(msg.pose.position.x)
        goal_y = int(msg.pose.position.y)
        
        if not (0 <= goal_x < self.map_width and 0 <= goal_y < self.map_height):
            self.get_logger().warn('设定的目标点超出了地图范围！')
            return
            
        self.get_logger().info(f'收到作业指令！正在计算从起点 {self.start_pos} 到靶区 ({goal_x}, {goal_y}) 的最优路径...')
        
        # 执行 A* 算法
        path_coords = self.a_star(self.start_pos, (goal_x, goal_y))
        
        if path_coords:
            self.publish_path(path_coords)
            self.get_logger().info('路径规划成功！已下发执行指令。')
        else:
            self.get_logger().warn('该目标点被极高危区域完全包围，无法找到安全路径！')

    def a_star(self, start, goal):
        """A* 寻路算法核心逻辑"""
        # 优先级队列：(f_score, (x, y))
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # 记录路径来源，用于回溯
        came_from = {}
        
        # g_score：从起点到当前点的实际代价
        g_score = {start: 0}
        
        # 8个移动方向
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # 找到终点，回溯路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1] # 反转路径
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查边界
                if not (0 <= neighbor[0] < self.map_width and 0 <= neighbor[1] < self.map_height):
                    continue
                    
                # 获取该网格的危险程度代价 (Cost: 0-100)
                grid_cost = self.grid[neighbor[1] * self.map_width + neighbor[0]]
                
                # 如果是绝对的危险死角（比如代价值接近 100），视为不可通行障碍
                if grid_cost > 90:
                    continue
                
                # 计算步长代价（对角线移动代价更高）
                step_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                # 核心：将环境危险代价加入寻路权重（危险度越高，机器人越倾向于绕开）
                weight = 1.0 + (grid_cost / 10.0) 
                
                tentative_g_score = g_score[current] + step_cost * weight
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    # f = g + h (h使用欧几里得距离)
                    h_score = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    f_score = tentative_g_score + h_score
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return None

    def publish_path(self, path_coords):
        """将规划好的网格坐标转化为 ROS 标准路径消息并发布"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for (x, y) in path_coords:
            pose = PoseStamped()
            pose.header = path.header
            # 将网格坐标偏移到网格中心点展示更美观
            pose.pose.position.x = x + 0.5
            pose.pose.position.y = y + 0.5
            pose.pose.position.z = 0.1 # 稍微抬高一点，防止被地图遮挡
            path.poses.append(pose)
            
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
