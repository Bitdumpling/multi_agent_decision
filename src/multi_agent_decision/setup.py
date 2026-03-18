from setuptools import find_packages, setup

package_name = 'multi_agent_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jeff@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'risk_node = multi_agent_decision.risk_assessment_node:main',
            'planner_node = multi_agent_decision.task_planning_node:main',
            'decision_node = multi_agent_decision.decision_maker_node:main',
            'follower_node = multi_agent_decision.path_follower_node:main',
            'virtual_robot = multi_agent_decision.virtual_robot_node:main',
            'fleet_manager = multi_agent_decision.fleet_manager_node:main'
        ],
    },
)
