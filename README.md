Based on the README document content, here is the full translation to English, as requested. The technical terminology, code snippets, file/folder names, and topic names are preserved in their original format.

# **README**

Core Decision Brain (**Module Three**) of the "Intelligent Invasive Plant Management System".

Based on **ROS2 Humble**, this module is responsible for receiving wide-area multispectral visual perception data from upstream (Module One/Two), performing dynamic ecological risk assessment, generating a high-fidelity cost/warning map, and completing **multi-agent fleet collaborative task assignment** and dispatching **precise control commands** under multiple physical constraints (battery, chemical). It is the key bridge connecting "Perception" and "Execution".

------

## Core Features

| Feature Module                                   | Description                                                  |
| ------------------------------------------------ | ------------------------------------------------------------ |
| **1. Dynamic Ecological Risk Assessment**        | Abandons binary static maps, introducing a **Gaussian Diffusion Attenuation Model** to simulate the natural spread characteristics of invasive plants. Integrates plant species, area, and ecological vulnerability to generate a warning map with a **continuous 0-100 cost gradient**. |
| **2. Multi-Agent Collaborative Task Scheduling** | Builds a central scheduling hub (`Fleet Manager`). During task assignment, it **simultaneously verifies hardware resource thresholds** (e.g., triggering a safety protocol for chemical levels below 20%), dynamically removes depleted nodes, and achieves optimal cross-region allocation. |
| **3. Environmentally Sensitive Path Planning**   | Redefines the A* heuristic function `f(n) = g(n) + h(n) + ecological_cost(n)`, integrating the warning map cost into the step weight. This enables an intelligent penetration strategy of "**Avoid the periphery, strike directly at the target core**", reducing unnecessary ecological damage and mechanical wear. |
| **4. Structured Operational Command Output**     | A state machine automatically assesses the target area's hazard level (High/Medium/Low/Patrol) and outputs **standardized JSON control packets** to the lower layer (Module Four), precisely defining physical quantities such as blade rotation speed and spray concentration, eliminating ambiguous commands. |

------

## Quick Start

To facilitate team debugging and demonstrations, this module has built-in virtual data publishing and a simulated two-robot formation. Please run the following commands in order in 4 terminals:

> **Prerequisite**: Ensure ROS2 Humble is installed, this package is placed in the `src/`directory of your workspace, and compilation is complete.

1. **Start the Risk Situation Awareness Node**

   ```
   # Terminal 1
   source install/setup.bash
   ros2 run multi_agent_decision risk_node
   ```
   
2. **Start the Mock Drone Data Publisher (Simulated Input)**

   ```
   # Terminal 2
   source install/setup.bash
   python3 ~/mock_drone.py
   ```
   
3. **Start the Core Decision and Operational Control Node**

   ```
   # Terminal 3
   source install/setup.bash
   ros2 run multi_agent_decision decision_node
   ```
   
4. **Start the Multi-Agent Central Scheduling Hub (Two-vehicle formation)**

   ```
   # Terminal 4
   source install/setup.bash
   ros2 run multi_agent_decision fleet_manager
   ```

### RViz Visualization and Interaction

1. Open `rviz2`.

2. **Add Map**: Set the `Fixed Frame`to `map`. Add a display type of `Map`. Subscribe to the topic `/warning_map`to see the dynamically generated ecological risk warning map.

3. **Add Path**: Add a display type of `Path`. Subscribe to the topic `/planned_path`to observe the globally planned path by the algorithm.

4. **Add Formation**: Add a display type of `Marker`. Subscribe to the topic `/fleet_markers`to see the **yellow** and **blue** virtual execution vehicles.

5. **Interactive Demo**: Use the `2D Goal Pose`tool in the top toolbar and click on a red high-risk target area within the RViz map to set a goal. Subsequently, you will see detailed **resource scheduling simulation logs** in the terminal and observe in real-time on RViz the complete process of **multi-vehicle deployment and dynamic path planning**!

------

## Upstream and Downstream Interfaces

### 1. Inputs Subscribed by This Module

| Topic Name         | Message Type                 | Source Module                             | Data Format Description                                      |
| ------------------ | ---------------------------- | ----------------------------------------- | ------------------------------------------------------------ |
| `/perception_data` | `std_msgs/Float32MultiArray` | Visual Perception Module (Module One/Two) | Array format: `[Target_X_Coordinate, Target_Y_Coordinate, Plant_Species_Hazard_Weight, Invasion_Area_Ratio, Region_Ecological_Vulnerability]` |
| `/goal_pose`       | `geometry_msgs/PoseStamped`  | Manual UI / Central Control               | The target area coordinates requiring treatment (The RViz 2D Goal Pose tool publishes to this topic by default). |

### 2. Outputs Published by This Module

| Topic Name           | Message Type                     | Target Module                                      | Data Format Description                                      |
| -------------------- | -------------------------------- | -------------------------------------------------- | ------------------------------------------------------------ |
| `/warning_map`       | `nav_msgs/OccupancyGrid`         | Global Sharing (RViz, etc.)                        | **0-100 Continuous Cost Gradient Warning Map**. Resolution 1m. Used for global situation display. |
| `/planned_path`      | `nav_msgs/Path`                  | Underlying Motion Control (Module Four)            | **Global topological guidance path** generated by the environment-cost-aware A* algorithm. Module Four should combine it with local planners like DWA or TEB for tracking and obstacle avoidance. |
| `/execution_command` | `std_msgs/String`(Contains JSON) | Underlying Motion Control / Actuator (Module Four) | **Precise operational command**. The underlying MCU/controller needs to parse this JSON packet to drive the robotic arm and spray nozzle. |

### JSON Example

```
{
  "mission_id": "target_001",
  "timestamp": 1678886400.0,
  "vehicle_id": "ugv_01",
  "target_pose": {
    "x": 10.5,
    "y": 8.2,
    "theta": 0.0
  },
  "hazard_level": "HIGH", // Hazard level: HIGH, MEDIUM, LOW, PATROL
  "action_sequence": [
    {
      "action_type": "CUT",
      "parameters": {
        "blade_speed_rpm": 3500,
        "cut_height_cm": 5.0
      }
    },
    {
      "action_type": "SPRAY",
      "parameters": {
        "chemical_concentration": 0.05,
        "spray_volume_ml": 200,
        "duration_ms": 1000
      }
    }
  ],
  "resource_check": {
    "battery_percent": 85,
    "chemical_remaining_ml": 1500
  }
}
```

------

## Project Structure

```
multi_agent_decision/
├── package.xml
├── setup.py
├── README.md
├── resource/
│   └── multi_agent_decision
├── test/
└── multi_agent_decision/
    ├── __init__.py
    ├── risk_assessment_node.py    # Warning Map Generation
    ├── task_planning_node.py      # Single-agent Planning (Early Test Version)
    ├── decision_maker_node.py     # Decision State Machine
    ├── path_follower_node.py      # Path Follower
    ├── virtual_robot_node.py      # Single Virtual Robot
    └── fleet_manager_node.py      # Two-vehicle Scheduling & A* Pathfinding
```