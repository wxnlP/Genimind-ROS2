import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_launch_path = get_package_share_directory("genimind_base")
    can_script_path = os.path.join(this_launch_path, "script", "can_init.sh")
    gpio_script_path = os.path.join(this_launch_path, "script", "can_signal.py")

    # 参数声明
    action_declare_arg_open_can = launch.actions.DeclareLaunchArgument(
        "open_can",
        default_value="False"
    )

    # 参数替换
    open_can = launch.substitutions.LaunchConfiguration(
        "open_can",
        default="False"
    )

    action_node_genimind_base = launch_ros.actions.Node(
        # 功能包名称
        package="genimind_base",
        # 可执行文件名称
        executable="genimind_base",
        # 输出方式，screen、log、both三种
        output="screen"
    )

    # 终端Open Can指令
    action_cmd_open_can = launch.actions.ExecuteProcess(
        condition=launch.conditions.IfCondition(open_can),
        cmd=['bash', can_script_path],
        output='screen'
    )
    # 终端Signal High指令
    action_cmd_signal = launch.actions.ExecuteProcess(
        cmd=['python3', gpio_script_path],
        output='screen'
    )

    action_group = launch.actions.GroupAction([
        # 定时器，定时第几秒启动什么指令
        launch.actions.TimerAction(period=0.0, actions=[action_cmd_open_can]),
        launch.actions.TimerAction(period=1.0, actions=[action_cmd_signal]),
        launch.actions.TimerAction(period=2.0, actions=[action_node_genimind_base])
    ])

    # 日志打印
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