from setuptools import find_packages, setup

package_name = "go2_nav2"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/sport_bridge.launch.py",
                "launch/slam_mapping.launch.py",
                "launch/sensors.launch.py",
                "launch/robot_description.launch.py",
                "launch/nav2_slam_loc.launch.py",
                "launch/patrol.launch.py",
                "launch/bringup_viz.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/rviz",
            ["rviz/nav.rviz", "rviz/slam.rviz", "rviz/bringup.rviz"],
        ),
        (
            "share/" + package_name + "/urdf",
            ["urdf/go2_vis.urdf"],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/sport_bridge.yaml",
                "config/odom_tf.yaml",
                "config/pointcloud_to_laserscan.yaml",
                "config/slam_toolbox_mapping.yaml",
                "config/go2_nav2_minimal.yaml",
                "config/slam_toolbox_localization.yaml",
                "config/patrol_example.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="danya",
    maintainer_email="danya@local",
    description="Go2 Edu: SLAM mapping, slam_toolbox localization, Nav2, sport_bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sport_bridge = go2_nav2.sport_bridge:main",
            "go2_odom_tf = go2_nav2.odom_tf:main",
            "go2_joint_state_bridge = go2_nav2.joint_state_bridge:main",
            "go2_simple_navigator = go2_nav2.simple_navigator:main",
            "go2_goal_pose_nav = go2_nav2.goal_pose_nav:main",
            "go2_patrol = go2_nav2.patrol:main",
            "go2_waypoint_recorder = go2_nav2.waypoint_recorder:main",
            "go2_map_odom_relay = go2_nav2.map_odom_relay:main",
            "go2_cloud_stamp_sync = go2_nav2.cloud_stamp_sync:main",
            "go2_sport_state_odom = go2_nav2.sport_state_odom:main",
        ],
    },
)
