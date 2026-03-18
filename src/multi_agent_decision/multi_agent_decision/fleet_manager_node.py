#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import heapq
import math

class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager_node')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/warning_map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/fleet_markers', 10)
        
        self.grid = None
        self.map_width = 0
        self.map_height = 0
        
        # 终极形态：加入硬件资源状态约束 (battery: 电量, payload: 药剂量)
        # 剧本设定：一号车虽然在左下角，但是药剂快耗尽了（只剩20%）
        self.robots = {
            'A': {'pos': (1.0, 1.0), 'color': (1.0, 0.8, 0.0), 'path': [], 'name': '一号重装车(黄色)', 'battery': 80, 'payload': 20},
            'B': {'pos': (18.0, 18.0), 'color': (0.0, 0.8, 1.0), 'path': [], 'name': '二号轻巡车(蓝色)', 'battery': 100, 'payload': 100}
        }
        
        self.timer = self.create_timer(0.15, self.update_fleet_physics)
        self.get_logger().info('多智能体协同调度系统已上线，随时待命！')

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.grid = msg.data

    def goal_callback(self, msg):
        if not self.grid: return
            
        goal_x = int(msg.pose.position.x)
        goal_y = int(msg.pose.position.y)
        goal = (goal_x, goal_y)
        
        # 假设每次执行治理任务，固定消耗 10%电量 和 30%药剂
        req_battery = 10
        req_payload = 30
        
        self.get_logger().info(f'\n================================================')
        self.get_logger().info(f'接收到指令 ({goal_x}, {goal_y})，正在进行多维协同评估...')
        self.get_logger().info(f'本次任务资源需求：预计耗电: {req_battery}%, 预计耗药: {req_payload}%')
        
        path_a = self.a_star((int(self.robots['A']['pos'][0]), int(self.robots['A']['pos'][1])), goal)
        path_b = self.a_star((int(self.robots['B']['pos'][0]), int(self.robots['B']['pos'][1])), goal)
        
        cost_a = len(path_a) if path_a else float('inf')
        cost_b = len(path_b) if path_b else float('inf')
        
        self.get_logger().info(f'[A] {self.robots["A"]["name"]} 状态 -> 距离代价: {cost_a}, 电量: {self.robots["A"]["battery"]}%, 药剂: {self.robots["A"]["payload"]}%')
        self.get_logger().info(f'[B] {self.robots["B"]["name"]} 状态 -> 距离代价: {cost_b}, 电量: {self.robots["B"]["battery"]}%, 药剂: {self.robots["B"]["payload"]}%')
        
        # 多重约束评估逻辑
        valid_candidates = []
        
        if cost_a < float('inf'):
            if self.robots['A']['battery'] >= req_battery and self.robots['A']['payload'] >= req_payload:
                valid_candidates.append(('A', cost_a))
            else:
                self.get_logger().warn('一号车(黄色)距离虽近，但【药剂或电量不足】，安全协议已将其剔除候选名单！')

        if cost_b < float('inf'):
            if self.robots['B']['battery'] >= req_battery and self.robots['B']['payload'] >= req_payload:
                valid_candidates.append(('B', cost_b))
            else:
                self.get_logger().warn('二号车(蓝色)电量或药剂不足！')

        if not valid_candidates:
            self.get_logger().error('警告：所有在线机器人均无法满足作业条件（路径不可达或资源枯竭），请求人工干预！')
            self.get_logger().info(f'================================================\n')
            return
            
        # 优中选优：在能完成任务的车里，选距离最近的
        winner = min(valid_candidates, key=lambda x: x[1])[0]
        loser = 'B' if winner == 'A' else 'A'
        best_path = path_a if winner == 'A' else path_b
        
        # 扣除作业消耗的资源
        self.robots[winner]['battery'] -= req_battery
        self.robots[winner]['payload'] -= req_payload
        
        self.get_logger().info(f'最终指派 {self.robots[winner]["name"]} 跨区执行任务！')
        self.get_logger().info(f'预计作业后剩余电量: {self.robots[winner]["battery"]}%, 药剂: {self.robots[winner]["payload"]}%')
        self.get_logger().info(f'================================================\n')
        
        self.robots[winner]['path'] = best_path
        self.robots[loser]['path'] = []
        self.publish_path(best_path)

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.map_width and 0 <= neighbor[1] < self.map_height): continue
                    
                grid_cost = self.grid[neighbor[1] * self.map_width + neighbor[0]]
                # 已经去掉了 >90 的限制，允许机器人杀入重围
                
                step_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                weight = 1.0 + (grid_cost / 10.0) 
                tentative_g_score = g_score[current] + step_cost * weight
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    heapq.heappush(open_set, (f_score, neighbor))
        return None

    def update_fleet_physics(self):
        for r_id, robot in self.robots.items():
            if robot['path']:
                next_pos = robot['path'].pop(0)
                robot['pos'] = (float(next_pos[0]) + 0.5, float(next_pos[1]) + 0.5)
                
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"robot_{r_id}"
            marker.id = ord(r_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = robot['pos'][0]
            marker.pose.position.y = robot['pos'][1]
            marker.pose.position.z = 0.2
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.4
            marker.color.a = 1.0
            marker.color.r = robot['color'][0]
            marker.color.g = robot['color'][1]
            marker.color.b = robot['color'][2]
            self.marker_pub.publish(marker)

    def publish_path(self, path_coords):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for (x, y) in path_coords:
            pose = PoseStamped()
            pose.pose.position.x = x + 0.5
            pose.pose.position.y = y + 0.5
            path.poses.append(pose)
        self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
