#include "genimind_base/genimind_base.h"


/* 构造函数 */
GenimindBaseNode::GenimindBaseNode(std::string nodeName) : Node(nodeName)
{
    /* 打印日志 */
    RCLCPP_INFO(this->get_logger(), "创建节点%s", nodeName.c_str());

    /* 声明参数 */
    this->declare_parameter("can_x", "can0");
    this->get_parameter("can_x", can_x_);
    this->declare_parameter("pub_odom", false);
    this->get_parameter("pub_odom", pub_odom_);
    /* 初始化CAN接收器 */
    can_receiver_ = std::make_shared<SocketCanReceiver>(
        // 使用can0
        can_x_,
        // 不使用CAN_FD
        false
    );
    /* 创建发送者 */
    can_sender_ = std::make_shared<SocketCanSender>(
        // can接口
        can_x_,
        // 启用can_fd
        false,
        // 默认配置
        SocketCanId(
            // 默认ID
            0x123,
            // 时间戳，0表示立即发送
            0,
            // 数据帧、遥控帧、错误帧
            FrameType::DATA,
            // 标准格式、扩展格式
            StandardFrame_{})
    );
    /* 创建发布者 */
    // 里程计
    odom_publisher_ = this->create_publisher<Odometry>("odom", 10);
    // IMU
    imu_publisher_ = this->create_publisher<Imu>("imu", 10);
    // 机器人状态
    status_publisher_ = this->create_publisher<GenimindStatus>("genimind_status", 10);

    /* 创建速度指令的订阅者 */
    cmd_vel_Subscription_ = this->create_subscription<Twist>(
        "cmd_vel",
        10,
        std::bind(&GenimindBaseNode::cmd_vel_callback, this, std::placeholders::_1)
    );

    /* 创建蜂鸣器、LED控制服务 */
    buzzer_led_service_ = this->create_service<GenimindBuzzerLed>(
        "genimind_buzzer_led",
        std::bind(&GenimindBaseNode::genimind_buzzer_led_callback, this,
             std::placeholders::_1, std::placeholders::_2)
    );
    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 启动一个10ms的定时器，处理订阅者之外的其他信息
    timer_ = this->create_wall_timer(100ms, std::bind(&GenimindBaseNode::timer_callback, this));
    // 输出日志
    RCLCPP_INFO(this->get_logger(), "Genimind-v1 is ready.");
    // 单开一个接收can数据的线程
    read_frame_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&GenimindBaseNode::process_can_read_frame, this)));
}

/**
 * @brief 字节数组转浮点真实数据
 * 
 * @param[in] bytes 8字节数组
 * @param[out] data 浮点数数组
 */
void GenimindBaseNode::bytes_to_float(const unsigned char *bytes, float *data)
{
    for (int i = 0; i < 2; i++) {
        memcpy(&data[i], bytes + i * sizeof(float), sizeof(float));
    }
}

/* ----------------------------------CAN数据帧处理---------------------------------------------- */

/**
 * @brief 处理接收的数据帧
 */
void GenimindBaseNode::process_can_read_frame(void)
{
    /* 初始化变量 */
    CanFrame frame;
    while (rclcpp::ok())
    {
        try {
            /* 接收数据 */
            can_rx_id_ = can_receiver_->receive(frame.rx_msg, 1ms);
            /* 字节数组转浮点 */
            bytes_to_float(frame.rx_msg, frame.rx_msg_real);
            frame.rx_msg_id = can_rx_id_.get();
            /* 解析数据 */
            process_analyse_frame(&frame);
        } catch (const drivers::socketcan::SocketCanTimeout& e) {
            RCLCPP_DEBUG(this->get_logger(), "接收超时（正常现象）");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "接收错误: %s", e.what());
        }
        
    }
   
}

void GenimindBaseNode::process_analyse_frame(CanFrame* frame)
{
    static uint8_t imu_h_flag, imu_m_flag, imu_l_flag;
    static uint8_t quat_h_flag, quat_l_flag;
    static uint8_t linear_flag, angular_flag;
    switch (frame->rx_msg_id)
    {
        // 四元数前两位
        case ID_QUATERNION_H:
            quat_h_flag = 1;
            imu_data_.w = frame->rx_msg_real[0];
            imu_data_.x = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // 四元数后两位
        case ID_QUATERNION_L:
            quat_l_flag = 1;
            imu_data_.y = frame->rx_msg_real[0];
            imu_data_.z = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // 线速度
        case ID_LINEAR:
            linear_flag = 1;
            odom_data_.v_tx = frame->rx_msg_real[0];
            odom_data_.v_ty = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // 角速度
        case ID_ANGULAR:
            angular_flag = 1;
            odom_data_.omega = frame->rx_msg_real[0];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);    
            break;
        // X、Y轴加速度
        case ID_IMU_H:
            imu_h_flag = 1;
            imu_data_.acc_x = frame->rx_msg_real[0];
            imu_data_.acc_y = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // Z轴加速度，X轴陀螺仪
        case ID_IMU_M:
            imu_m_flag = 1;
            imu_data_.acc_z = frame->rx_msg_real[0];
            imu_data_.gyro_x = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // X、Y轴陀螺仪
        case ID_IMU_L:
            imu_l_flag = 1;
            imu_data_.gyro_y = frame->rx_msg_real[0];
            imu_data_.gyro_z = frame->rx_msg_real[1];
            // RCLCPP_INFO(this->get_logger(), "收到Frame ID [%d] 数据", frame->rx_msg_id);
            break;
        // 电池电压√
        case ID_STATUS:
            robot_status_.voltage = frame->rx_msg_real[0];
            break;
        default:
            // RCLCPP_ERROR(this->get_logger(), "Frame ID Error[%d]", can_rx_id_.get());
            break;
    }
    // 发布IMU数据
    if (imu_h_flag == 1 && imu_m_flag == 1 && imu_l_flag == 1 && quat_h_flag ==1 && quat_l_flag ==1) {
        // 清零标志位
        imu_h_flag = 0;
        imu_m_flag = 0;
        imu_l_flag = 0;
        quat_h_flag = 0;
        quat_l_flag = 0;
        imu_converte(&imu_data_);
        imu_publish(&imu_data_);
    }
    if (linear_flag == 1 && angular_flag == 1) {
        linear_flag = 0;
        angular_flag = 0;
        odom_calculate(&odom_data_);
        odom_publish(&odom_data_);
    }
}

/* ----------------------------------里程计数据处理---------------------------------------------- */

/** 
 * @brief 里程计之位置计算
 * 
 * @param data 里程计数据，线速度、角速度
 */
void GenimindBaseNode::odom_calculate(OdomData* data)
{
    // 周期计算
    auto current_time_ = this->now();
    double dt = (current_time_.seconds() - last_time_.seconds());
    last_time_ = current_time_;
    // 计算里程计单周期内的姿态
    double delta_x = (data->v_tx * cos(odom_th_) - data->v_ty * sin(odom_th_)) * dt;
    double delta_y = (data->v_tx * sin(odom_th_) + data->v_ty * cos(odom_th_)) * dt;
    double delta_th = data->omega * dt;
    // 计算里程计的累积姿态
    odom_x_  += delta_x;
    odom_y_  += delta_y;
    odom_th_ += delta_th;
    // 校正姿态角度，让机器人处于-180~180度之间
    if(odom_th_ > M_PI) {
        odom_th_ -= M_PI*2;
    } else if(odom_th_ < (-M_PI)) {
        odom_th_ += M_PI*2;
    }
}

void GenimindBaseNode::odom_publish(OdomData* data)
{
    Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    // 位置
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    // 欧拉转四元数
    q.setRPY(0, 0, odom_th_);
    odom.child_frame_id = "base_footprint";
    // 四元数
    odom.pose.pose.orientation.x = q[0];
    odom.pose.pose.orientation.y = q[1];
    odom.pose.pose.orientation.z = q[2];
    odom.pose.pose.orientation.w = q[3];

    // 协方差
    const double ODOM_POSE_COVARIANCE[36] = {
        1e-3, 0,    0,    0,    0,    0,
        0,    1e-3, 0,    0,    0,    0,
        0,    0,    1e6,  0,    0,    0,
        0,    0,    0,    1e6,  0,    0,
        0,    0,    0,    0,    1e6,  0,
        0,    0,    0,    0,    0,    1e-3
      };
      
    const double ODOM_TWIST_COVARIANCE[36] = {
        1e-3, 0,    0,    0,    0,    0,
        0,    1e-3, 0,    0,    0,    0,
        0,    0,    1e6,  0,    0,    0,
        0,    0,    0,    1e6,  0,    0,
        0,    0,    0,    0,    1e6,  0,
        0,    0,    0,    0,    0,    1e-3
    };
    //线速度
    odom.twist.twist.linear.x = data->v_tx;
    odom.twist.twist.linear.y = data->v_ty;
    odom.twist.twist.linear.z = 0;
    // 角速度
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = data->omega;

    memcpy(&odom.pose.covariance, ODOM_POSE_COVARIANCE, sizeof(ODOM_POSE_COVARIANCE));
    memcpy(&odom.twist.covariance, ODOM_TWIST_COVARIANCE, sizeof(ODOM_TWIST_COVARIANCE));

    // 发布里程计话题
    odom_publisher_->publish(odom);

    // TF
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_footprint";

    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q[0];
    t.transform.rotation.y = q[1];
    t.transform.rotation.z = q[2];
    t.transform.rotation.w = q[3];

    if(pub_odom_){
        // 广播里程计TF
        tf_broadcaster_->sendTransform(t);
    }
}
/* ----------------------------------接收运动控制指令---------------------------------------------- */

void GenimindBaseNode::cmd_vel_callback(Twist::SharedPtr msg) 
{
    unsigned char msg_linear[8] = {0};
    unsigned char msg_angular[8] = {0};
    OdomData odom = {
        .v_tx = static_cast<float>(msg->linear.x),
        .v_ty = static_cast<float>(msg->linear.y),
        .omega = static_cast<float>(msg->angular.z),
    };
    memcpy(msg_linear, &odom.v_tx, sizeof(float));
    memcpy(msg_linear+4, &odom.v_ty, sizeof(float));
    memcpy(msg_angular, &odom.omega, sizeof(float));
    // RCLCPP_INFO(this->get_logger(), "cmd_vel_callback %d %d %d %d %d %d %d %d", 
    //     msg_linear[0], msg_linear[1], msg_linear[2], msg_linear[3], msg_linear[4], msg_linear[5], msg_linear[6], msg_linear[7]);
    // 发送can数据帧
    try {
        can_sender_->send(
            // 必要参数
            msg_linear,
            // 灵活配置(选配)
            SocketCanId(
                // 默认ID
                CMD_LINEAR,
                // 时间戳，0表示立即发送
                0,
                // 数据帧、遥控帧、错误帧
                FrameType::DATA,
                // 标准格式、扩展格式
                StandardFrame_{}),
            // 选配
            1ms);
        can_sender_->send(
            // 必要参数
            msg_angular,
            // 灵活配置(选配)
            SocketCanId(
                // 默认ID
                CMD_ANGULAR,
                // 时间戳，0表示立即发送
                0,
                // 数据帧、遥控帧、错误帧
                FrameType::DATA,
                // 标准格式、扩展格式
                StandardFrame_{}),
            // 选配
            1ms);
    } catch (const std::system_error& e) {
        // 系统调用错误（如write失败）
        RCLCPP_ERROR(this->get_logger(), "发送失败（系统错误）: %s", e.what());
    } catch (const std::invalid_argument& e) {
        // 数据格式错误（如ID超出范围）
        RCLCPP_ERROR(this->get_logger(), "数据格式错误: %s", e.what());
    } catch (const std::exception& e) {
        // 其他未知异常
        RCLCPP_ERROR(this->get_logger(), "未知错误: %s", e.what());
    }
}

/* ----------------------------------IMU数据处理--------------------------------------------------- */

/**
 * @brief 将imu数据转换为 sensor_msgs::msg::Imu 需要的单位
 * 
 * --> 陀螺仪(弧度/秒) 加速度计(米/平方秒)
 * 
 * @param data 陀螺仪(°/s)、加速度计(g)的真实值 
 */
void GenimindBaseNode::imu_converte(ImuData* data)
{
    data->acc_x *= (M_PI / 180);
    data->acc_y *= (M_PI / 180);
    data->acc_z *= (M_PI / 180);
    data->gyro_x *= 9.80665;
    data->gyro_y *= 9.80665;
    data->gyro_z *= 9.80665;
}

/**
 * @brief 发送IMU数据
 * 
 * @param data IMU数据
 */
void GenimindBaseNode::imu_publish(ImuData* data)
{
    Imu imu;
    imu.header.stamp = this->get_clock()->now();
    imu.header.frame_id = "imu_link";
    imu.orientation.w = data->w;
    imu.orientation.x = data->x;
    imu.orientation.y = data->y;
    imu.orientation.z = data->z;
    imu.angular_velocity.x = data->gyro_x;
    imu.angular_velocity.y = data->gyro_y;
    imu.angular_velocity.z = data->gyro_z;
    imu.linear_acceleration.x = data->acc_x;
    imu.linear_acceleration.y = data->acc_y;
    imu.linear_acceleration.z = data->acc_z;
    imu.linear_acceleration_covariance = {0.05, 0.00, 0.00, 0.00, 0.05, 0.00, 0.00, 0.00, 0.05};
    imu.angular_velocity_covariance = {0.01, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.01};
    imu.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025, 0.0000, 0.0000, 0.0000, 0.0025};
    imu_publisher_->publish(imu);
}

/* ---------------------------------机器人状态处理------------------------------------------------- */

/**
 * @brief 定时器处理控制器发送的机器人状态信息，已经记录的LED、蜂鸣器状态
 * 
 */
void GenimindBaseNode::timer_callback(void)
{
    GenimindStatus genimind_status;
    genimind_status.voltage = robot_status_.voltage;
    genimind_status.buzzer_on = robot_status_.buzzer_on;
    genimind_status.led_on = robot_status_.led_on;
    status_publisher_->publish(genimind_status);
}

bool GenimindBaseNode::buzzer_led_control(bool buzzer, bool led)
{
    unsigned char buffer[8] = {0};
    // 处理前两位
    if (buzzer) {
        buffer[0] = 0xFF;
    } else {
        buffer[0] = 0x00;
    }
    if (led) {
        buffer[1] = 0xFF;
    } else {
        buffer[1] = 0x00;
    }
    // 发送can数据帧
    try {
        can_sender_->send(
            // 必要参数
            buffer,
            // 灵活配置(选配)
            SocketCanId(
                // 默认ID
                CMD_BUZZER_LED,
                // 时间戳，0表示立即发送
                0,
                // 数据帧、遥控帧、错误帧
                FrameType::DATA,
                // 标准格式、扩展格式
                StandardFrame_{}),
            // 选配
            1ms);
    } catch (const std::system_error& e) {
        // 系统调用错误（如write失败）
        RCLCPP_ERROR(this->get_logger(), "发送失败（系统错误）: %s", e.what());
    } catch (const std::invalid_argument& e) {
        // 数据格式错误（如ID超出范围）
        RCLCPP_ERROR(this->get_logger(), "数据格式错误: %s", e.what());
    } catch (const std::exception& e) {
        // 其他未知异常
        RCLCPP_ERROR(this->get_logger(), "未知错误: %s", e.what());
    }
    return true;
}

/* 蜂鸣器LED服务回调函数 */
void GenimindBaseNode::genimind_buzzer_led_callback(
    std::shared_ptr<GenimindBuzzerLed::Request> request,
    std::shared_ptr<GenimindBuzzerLed::Response> response)
{
    // 更新状态
    robot_status_.buzzer_on = request->buzzer_on;
    robot_status_.led_on = request->led_on;
    if (buzzer_led_control(robot_status_.buzzer_on, robot_status_.led_on)) {
        RCLCPP_INFO(this->get_logger(), "Set Genimind State: buzzer->%d led->%d", 
            robot_status_.buzzer_on, robot_status_.led_on);
        response->result = true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Set Genimind State Error");
        response->result = false;
    }
}

/* ---------------------------------主函数--------------------------------------------------------- */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenimindBaseNode>("genimind_base_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}