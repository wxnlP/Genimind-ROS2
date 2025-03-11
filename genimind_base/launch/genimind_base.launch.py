import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_launch_path = get_package_share_directory("genimind_base")
    script_path = os.path.join(this_launch_path, "launch", "can_init.sh")
    action_declare_arg_open_can = launch.actions.DeclareLaunchArgument(
        "open_can",
        default_value="True"
    )

    open_can = launch.substitutions.LaunchConfiguration(
        "open_can",
        default="True"
    )

    action_node_genimind_base = launch_ros.actions.Node(
        # 功能包名称
        package="genimind_base",
        # 可执行文件名称
        executable="genimind_base",
        # 输出方式，screen、log、both三种
        output="screen"
    )

    action_cmd_open_can = launch.actions.ExecuteProcess(
        condition=launch.conditions.IfCondition(open_can),
        cmd=['bash', script_path],
        output='screen'
    )

    action_group = launch.actions.GroupAction([
        # 动作5-定时器，定时第几秒启动什么指令
        launch.actions.TimerAction(period=0.0, actions=[action_cmd_open_can]),
        launch.actions.TimerAction(period=1.0, actions=[action_node_genimind_base])
    ])

    action_log_info = launch.actions.LogInfo(
        condition=launch.conditions.IfCondition(open_can),
        msg="打开can0 ..."
    )

    # 合成启动描述
    launch_description = launch.LaunchDescription([
        action_log_info,
        action_group
    ])
    return launch_description