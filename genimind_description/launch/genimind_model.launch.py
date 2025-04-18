import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions


def generate_launch_description():
    # 获取urdf功能包的路径
    urdf_pkg_path = get_package_share_directory("genimind_description")
    # 获取urdf文件的路径
    urdf_file_path = urdf_pkg_path + "/urdf/genimind_description.urdf"
    # 获取rviz初始化文件路径
    rviz_config_path = urdf_pkg_path + "/config/rviz/genimind_descriptionconfig.rviz"

    #----------------------------------参数声明-------------------------------------
    # 声明rqt启动控制参数
    action_declare_arg_start_rqt = launch.actions.DeclareLaunchArgument(
        "start_rqt",
        default_value="False"
    )
    # 替换launch参数
    start_rqt = launch.substitutions.LaunchConfiguration(
        "start_rqt",
        default="False"
    )
    #----------------------------------参数声明-------------------------------------

    # 声明一个urdf文件路径的参数
    action_declare_arg_urdf_path = launch.actions.DeclareLaunchArgument(
        "urdf", 
        default_value=urdf_file_path,
        description="urdf文件的绝对路径"
    )

    # 1.获取指令的返回内容
    substitutions_cmd = launch.substitutions.Command(
        ["cat  ", launch.substitutions.LaunchConfiguration("urdf")]
    )
    # 2.获取指令的返回内容(多一次类型转换)
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_cmd,
        value_type=str
    )

    # robot_state_publisher话题节点启动
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    # joint_state_publisher话题节点启动
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    # rviz节点启动
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        condition=launch.conditions.IfCondition(start_rqt)
    )

    # 合成启动描述
    launch_description = launch.LaunchDescription([
        action_declare_arg_urdf_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
    return launch_description
