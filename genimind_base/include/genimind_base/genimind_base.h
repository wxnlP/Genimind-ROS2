#include "rclcpp/rclcpp.hpp"
/* CAN通信接口 */
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "can_msgs/msg/frame.hpp"
/* 主要数据接口 */
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
/* 自定义接口 */
#include "genimind_interfaces/msg/genimind_status.hpp"
#include "genimind_interfaces/srv/genimind_buzzer_led.hpp"

/* 命名空间 */
// 时间命名空间
using namespace std::chrono_literals;
// socket_can命名空间
using SocketCanReceiver = drivers::socketcan::SocketCanReceiver;
using SocketCanSender = drivers::socketcan::SocketCanSender;
using SocketCanId = drivers::socketcan::CanId;
using FrameType = drivers::socketcan::FrameType;
using StandardFrame_ = drivers::socketcan::StandardFrame_;
// Genimind消息接口命名空间
using GenimindStatus = genimind_interfaces::msg::GenimindStatus;
using GenimindBuzzerLed = genimind_interfaces::srv::GenimindBuzzerLed;

// 主要数据消息接口命名空间
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;

/* typedef */
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float w;
    float x;
    float y;
    float z;
} ImuData;

typedef struct {
    float v_tx;
    float v_ty;
    float omega;
} OdomData;

typedef struct {
    unsigned char rx_msg[8];
    float rx_msg_real[2];
    uint32_t rx_msg_id;
} CanFrame;

typedef struct {
    float voltage;
    bool buzzer_on;
    bool led_on;
} RobotStatus;

typedef enum {
    ID_QUATERNION_H = 0x411,
    ID_QUATERNION_L,
    ID_LINEAR = 0x421,
    ID_ANGULAR,
    ID_IMU_H = 0x431,
    ID_IMU_M,
    ID_IMU_L,
    ID_STATUS = 0x441,
} FrameID;

typedef enum {
    // 线速度 8字节
    CMD_LINEAR = 0x511,
    // 角速度 4字节
    CMD_ANGULAR,
    // 蜂鸣器和LED
    CMD_BUZZER_LED,
} Ros2Cmd;


/* Genimind底盘 */
class GenimindBaseNode : public rclcpp::Node
{
public:
    /* 构造函数 */
    GenimindBaseNode(std::string nodeName);

private:
    /* 功能函数 */
    void bytes_to_float(const unsigned char *bytes, float *data);
    void process_can_read_frame(void);
    void process_analyse_frame(CanFrame* frame);
    void imu_converte(ImuData* data);
    void odom_calculate(OdomData* data);
    /* 发布函数 */
    void imu_publish(ImuData* data);
    void odom_publish(OdomData* data);
    /* 控制指令函数 */
    bool buzzer_led_control(bool buzzer, bool led);
    /* 回调函数 */
    void cmd_vel_callback(Twist::SharedPtr  msg);
    void genimind_buzzer_led_callback(std::shared_ptr<GenimindBuzzerLed::Request> request,
        std::shared_ptr<GenimindBuzzerLed::Response> response);
    void timer_callback(void);
private:
    /* 参数定义 */
    // 单独一个接收CAN数据帧线程
    std::shared_ptr<std::thread> read_frame_thread_;
    // IMU的数据
    ImuData imu_data_;
    // 记录STM32传来的电压值，同时记录发送的LED、蜂鸣器控制结果
    RobotStatus robot_status_;
    // 里程计数据
    OdomData odom_data_;
    // 里程计积分计算
    float odom_x_=0.0, odom_y_=0.0, odom_th_=0.0;
    // 记录时间间隔
    rclcpp::Time last_time_;
    // 是否发布里程计
    bool pub_odom_;
    // CAN
    std::string can_x_;
    std::shared_ptr<SocketCanReceiver> can_receiver_;
    std::shared_ptr<SocketCanSender> can_sender_;
    SocketCanId can_rx_id_;
    // 发布者(话题使用)
    rclcpp::Publisher<Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<GenimindStatus>::SharedPtr status_publisher_;
    // 订阅者(话题使用)
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_Subscription_;
    // 服务端(服务使用)
    rclcpp::Service<GenimindBuzzerLed>::SharedPtr buzzer_led_service_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};