"""
Autonomous exploration + FAR-planner return-home launch for A2.

This mirrors exploration.launch.py (TARE exploration stack) but adds the FAR
planner and a home_return_mux node so the robot drives itself back to its start
point once exploration is complete -- using FAR (graph-based, long-range) for
the return trip instead of relying on TARE's own rush-home.

Handoff design (see src/home_return_mux.cpp):
  - TARE's waypoints are remapped /way_point -> /way_point_tare
  - FAR's  waypoints are remapped /way_point -> /way_point_far
  - home_return_mux forwards exactly one of them to the real /way_point that the
    local planner consumes. While exploring it forwards TARE; when TARE publishes
    /exploration_finish == true it latches into "return home" mode, forwards FAR
    instead, and publishes the home goal to /goal_point until FAR reports
    /far_reach_goal_status == true.

Everything else (terrain analysis, local planner, path follower, visualization,
RViz) is identical to exploration.launch.py.

Usage:
  ros2 launch a2_ros exploration_home_return.launch.py rviz:=true
  ros2 launch a2_ros exploration_home_return.launch.py home_x:=1.0 home_y:=-2.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    a2_ros_dir       = get_package_share_directory('a2_ros')
    far_planner_dir  = get_package_share_directory('far_planner')
    graph_decoder_dir = get_package_share_directory('graph_decoder')
    rviz_path        = os.path.join(a2_ros_dir, 'rviz', 'exploration.rviz')
    tare_config      = os.path.join(a2_ros_dir, 'config', 'autonomy', 'tare_a2.yaml')
    workspace_root   = a2_ros_dir.split('/install/')[0]
    vis_log_dir      = os.path.join(workspace_root, 'src', 'meta_packages', 'a2_ros', 'log')
    os.makedirs(vis_log_dir, exist_ok=True)

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    visualization_map_file_arg = DeclareLaunchArgument(
        'visualization_map_file',
        default_value='',
        description='Optional .ply map used by visualization_tools for /overall_map'
    )
    far_config_arg = DeclareLaunchArgument(
        'far_config',
        default_value='default',
        description='FAR planner config file name (without .yaml) under far_planner/config'
    )
    home_x_arg = DeclareLaunchArgument(
        'home_x', default_value='0.0',
        description='Return-home goal X in the world frame'
    )
    home_y_arg = DeclareLaunchArgument(
        'home_y', default_value='0.0',
        description='Return-home goal Y in the world frame'
    )
    home_z_arg = DeclareLaunchArgument(
        'home_z', default_value='0.0',
        description='Return-home goal Z in the world frame'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame', default_value='map',
        description='Frame the home goal is published in (must match /state_estimation)'
    )
    graph_decoder_arg = DeclareLaunchArgument(
        'graph_decoder', default_value='true',
        description='Launch the FAR graph_decoder (visibility-graph visualization)'
    )

    nodes = [
        rviz_arg,
        visualization_map_file_arg,
        far_config_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        world_frame_arg,
        graph_decoder_arg,
        SetParameter(name='use_sim_time', value=False),

        # ---- terrain analysis (local map) ----
        Node(
            package='terrain_analysis',
            executable='terrainAnalysis',
            name='terrainAnalysis',
            output='screen',
            parameters=[{
                'scanVoxelSize':       0.05,
                'decayTime':           10.0,
                'noDecayDis':          3.0,
                'clearingDis':         8.0,
                'useSorting':          True,
                'quantileZ':           0.25,
                'considerDrop':        True,
                'limitGroundLift':     True,
                'maxGroundLift':       0.25,
                'clearDyObs':          False,
                'minDyObsDis':         0.3,
                'minDyObsAngle':       0.0,
                'minDyObsRelZ':        -0.5,
                'absDyObsRelZThre':    0.2,
                'minDyObsVFOV':        -16.0,
                'maxDyObsVFOV':        16.0,
                'minDyObsPointNum':    1,
                'noDataObstacle':      False,
                'noDataBlockSkipNum':  0,
                'minBlockPointNum':    10,
                'vehicleHeight':       0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 2.0,
                'minRelZ':             -1.0,
                'maxRelZ':             1.0,
                'disRatioZ':           0.2,
            }],
        ),

        # ---- terrain analysis ext (global map) ----
        Node(
            package='terrain_analysis_ext',
            executable='terrainAnalysisExt',
            name='terrainAnalysisExt',
            output='screen',
            parameters=[{
                'scanVoxelSize':        0.1,
                'decayTime':            10.0,
                'noDecayDis':           0.0,
                'clearingDis':          30.0,
                'useSorting':           True,
                'quantileZ':            0.25,
                'vehicleHeight':        0.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre':  2.0,
                'lowerBoundZ':          -1.0,
                'upperBoundZ':          1.0,
                'disRatioZ':            0.1,
                'checkTerrainConn':     True,
                'terrainUnderVehicle':  -0.75,
                'terrainConnThre':      0.5,
                'ceilingFilteringThre': 2.0,
                'localTerrainMapRadius': 4.0,
            }],
        ),
       # ---- local planner (obstacle avoidance + path following) ----
        Node(
            package='local_planner',
            executable='localPlanner',
            name='localPlanner',
            output='screen',
            parameters=[{
                'pathFolder':          get_package_share_directory('local_planner') + '/paths',
                'vehicleLength':       1.0,
                'vehicleWidth':        0.44,
                'sensorOffsetX':       0.0,
                'sensorOffsetY':       0.0,
                'twoWayDrive':         False,
                'laserVoxelSize':      0.05,
                'terrainVoxelSize':    0.2,
                'useTerrainAnalysis':  True,
                'checkObstacle':       True,
                'checkRotObstacle':    True,
                'adjacentRange':       2.0,
                'obstacleHeightThre':  0.25,
                'groundHeightThre':    0.2,
                'costHeightThre':      0.1,
                'costScore':           0.02,
                'useCost':             True,
                'pointPerPathThre':    2,
                'minRelZ':             -0.5,
                'maxRelZ':             0.8,
                'maxSpeed':            0.5,
                'dirWeight':           0.15,
                'dirThre':             130.0,
                'dirToVehicle':        False,
                'pathScale':           0.6,
                'minPathScale':        0.2,
                'pathScaleStep':       0.25,
                'pathScaleBySpeed':    False,
                'minPathRange':        1.0,
                'pathRangeStep':       0.5,
                'pathRangeBySpeed':    True,
                'pathCropByGoal':      True,
                'autonomyMode':        True,
                'autonomySpeed':       1.0,
                'joyToSpeedDelay':     2.0,
                'joyToCheckObstacleDelay': 5.0,
                'goalClearRange':      0.4,
                'goalX':               0.0,
                'goalY':               0.0,
            }],
        ),

        Node(
            package='local_planner',
            executable='pathFollower',
            name='pathFollower',
            output='screen',
            parameters=[{
                'sensorOffsetX':    0.0,
                'sensorOffsetY':    0.0,
                'pubSkipNum':       1,
                'twoWayDrive':      False,
                'lookAheadDis':     0.4,
                'yawRateGain':      8.0,
                'stopYawRateGain':  5.0,
                'maxYawRate':       40.0,
                'maxSpeed':         0.5,
                'maxAccel':         2.0,
                'switchTimeThre':   1.0,
                'dirDiffThre':      0.2,
                'stopDisThre':      0.1,
                'slowDwnDisThre':   0.2,
                'useInclRateToSlow': False,
                'inclRateThre':     120.0,
                'slowRate1':        0.25,
                'slowRate2':        0.5,
                'slowTime1':        2.0,
                'slowTime2':        2.0,
                'useInclToStop':    False,
                'inclThre':         45.0,
                'stopTime':         5.0,
                'noRotAtStop':      False,
                'noRotAtGoal':      True,
                'autonomyMode':     True,
                'autonomySpeed':    1.0,
                'joyToSpeedDelay':  2.0,
            }],
        ),

        # ---- TARE planner (autonomous exploration) ----
        # Its waypoints go to /way_point_tare so the mux, not the local planner,
        # decides whether TARE or FAR drives.
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[tare_config, {'pub_waypoint_topic_': '/way_point_tare'}],
        ),

        # ---- FAR planner (graph-based return-home navigation) ----
        # Mirrors far_planner.launch's remaps, plus /way_point -> /way_point_far so
        # FAR's output is gated by the mux. FAR subscribes /goal_point (published by
        # the mux) and reports arrival on /far_reach_goal_status.
        Node(
            package='far_planner',
            executable='far_planner',
            name='far_planner',
            output='screen',
            parameters=[
                PythonExpression([
                    '"', far_planner_dir, '/config/',
                    LaunchConfiguration('far_config'), '.yaml"'])
            ],
            remappings=[
                ('/odom_world', '/state_estimation'),
                ('/terrain_cloud', '/terrain_map_ext'),
                ('/scan_cloud', '/registered_scan'),
                ('/terrain_local_cloud', '/terrain_map'),
                ('/way_point', '/way_point_far'),
            ],
        ),

        # ---- graph_decoder (FAR visibility-graph decoding/visualization) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(graph_decoder_dir, 'launch', 'decoder.launch')),
            condition=IfCondition(LaunchConfiguration('graph_decoder')),
        ),

        # ---- home_return_mux (TARE -> FAR waypoint handoff) ----
        # home_x/y/z are declared as doubles in the node, so the string-valued
        # launch args must be cast to float; world_frame stays a string.
        Node(
            package='a2_ros',
            executable='home_return_mux',
            name='home_return_mux',
            output='screen',
            parameters=[{
                'home_x':              ParameterValue(LaunchConfiguration('home_x'), value_type=float),
                'home_y':              ParameterValue(LaunchConfiguration('home_y'), value_type=float),
                'home_z':              ParameterValue(LaunchConfiguration('home_z'), value_type=float),
                'world_frame':         ParameterValue(LaunchConfiguration('world_frame'), value_type=str),
                'goal_repub_period_s': 1.0,
            }],
        ),

        # ---- visualization tools (/overall_map, /explored_areas, /trajectory) ----
        Node(
            package='visualization_tools',
            executable='visualizationTools',
            name='visualizationTools',
            output='screen',
            parameters=[{
                'metricFile': os.path.join(workspace_root, 'install', 'meta_packages', 'a2_ros', 'log', 'metrics'),
                'trajFile': os.path.join(workspace_root, 'install', 'meta_packages', 'a2_ros', 'log', 'trajectory'),
                'mapFile': LaunchConfiguration('visualization_map_file'),
                'overallMapVoxelSize': 0.5,
                'exploredAreaVoxelSize': 0.3,
                'exploredVolumeVoxelSize': 0.5,
                'transInterval': 0.2,
                'yawInterval': 10.0,
                'overallMapDisplayInterval': 2,
                'exploredAreaDisplayInterval': 1,
            }],
        ),


        # ---- RViz ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    return LaunchDescription(nodes)
