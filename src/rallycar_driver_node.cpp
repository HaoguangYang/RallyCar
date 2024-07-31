#include "rallycar/serial_comms_interface.hpp"
#include "../firmware/rallycar_arduino_v2024.3/rallycar_msgs/msg/imu_raw.h"
#include "../firmware/rallycar_arduino_v2024.3/std_msgs/msg/float32.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <unistd.h>

namespace rallycar {

/**
 * @brief The driver node that bridges ROS2 messages with framed serial port communication
 */
class RallycarDriverNode : public rclcpp::Node, public SerialCommsInterface<256, 256, 3> {
    using SerialImuRaw = rallycar_msgs__msg::ImuRaw;
    using SerialFloat32 = std_msgs__msg::Float32;
    using Imu = sensor_msgs::msg::Imu;
    using Float32 = std_msgs::msg::Float32;
public:
    RallycarDriverNode(
        const rclcpp::NodeOptions& options,
        const uint8_t& session_id = 0
    ) :
        rclcpp::Node("rallycar_driver_node", options),
        SerialCommsInterface(session_id),
        ser_imu_sub(&RallycarDriverNode::pub_ros_imu, this),
        ser_accel_brake_pub(this),
        ser_steer_pub(this)
    {
        // set node config at startup from parameters
        // which port to use
        std::string port = this->declare_parameter<std::string>("serial_port_fd", "/dev/ttyACM0");
        this->setPort(port);
        // baudrate of the communication
        int baud = this->declare_parameter<int>("serial_port_baudrate", 115200);
        this->setBaudrate((unsigned long)baud);
        // what frame id should we entitle the IMU messages with
        std::string imu_frame = this->declare_parameter<std::string>("imu_frame_id", "imu_frame");
        imu.header.frame_id = imu_frame;

        // MPU-6000 Has 0.05 deg/s RMS noise, plus 1 deg/s bias
        double gyro_cov = std::pow(0.05 * M_PI / 180.0, 2.) +
                            std::pow(M_PI / 180.0, 2.);
        imu.angular_velocity_covariance[0] = gyro_cov;
        imu.angular_velocity_covariance[4] = gyro_cov;
        imu.angular_velocity_covariance[8] = gyro_cov;
        // MPU-6000 has 400ug/sqrt(Hz) power spectral density, plus 80mg bias
        double accel_cov = std::pow(400 * 1e-6 * 9.80665, 2) * 100.0 +
                            std::pow(80 * 1.e-3 * 9.80665, 2);
        imu.linear_acceleration_covariance[0] = accel_cov;
        imu.linear_acceleration_covariance[4] = accel_cov;
        imu.linear_acceleration_covariance[8] = accel_cov;
        // let's assume a 5 deg orientation covariance of the MPU
        double orientation_cov = std::pow(5. * M_PI / 180.0, 2);
        imu.orientation_covariance[0] = orientation_cov;
        imu.orientation_covariance[4] = orientation_cov;
        imu.orientation_covariance[8] = orientation_cov;

        // ros side
        ros_imu_pub = this->create_publisher<Imu>("/imu", rclcpp::SensorDataQoS());
        ros_acc_sub = this->create_subscription<Float32>(
            "/accelerator_cmd", rclcpp::SensorDataQoS(),
            std::bind(&RallycarDriverNode::pub_ser_acc, this, std::placeholders::_1));
        ros_steer_sub = this->create_subscription<Float32>(
            "/steering_cmd", rclcpp::SensorDataQoS(),
            std::bind(&RallycarDriverNode::pub_ser_steer, this, std::placeholders::_1));

        // serial port side, make sure the IDs are set the same as defined in the firmware code
        registerEndpoint(&ser_imu_sub, 0);
        registerEndpoint(&ser_accel_brake_pub, 1);
        registerEndpoint(&ser_steer_pub, 2);

        // initialize serial port and publishers & subscribers.
        // after the loop exits (succeeded), the node starts listening
        while (rclcpp::ok()) {
            RCLCPP_INFO(get_logger(),
                        "Trying to initialize communication over %s ...", port.c_str());
            if (this->transferInit()) {
                RCLCPP_INFO(get_logger(), "Communication initialized over %s .", port.c_str());
                break;
            }
            sleep(1);
        }
    }

protected:
    Imu imu;
    SerialFloat32 ser_accel_cmd, ser_steer_cmd;

    sensor_network::Subscription<SerialImuRaw, RallycarDriverNode> ser_imu_sub;
    sensor_network::Publisher<SerialFloat32, RallycarDriverNode> ser_accel_brake_pub, ser_steer_pub;

    rclcpp::Publisher<Imu>::SharedPtr ros_imu_pub;
    rclcpp::Subscription<Float32>::SharedPtr ros_acc_sub, ros_steer_sub;

    virtual SerialTime time_now() override final {
        // time getter for time synchoronization with serial devices
        SerialTime ret;
        double t = now().seconds();
        ret.sec = (int32_t)t;
        ret.nanosec = (uint32_t)((t - (double)ret.sec) * 1000000000U);
        return ret;
    }

    void pub_ros_imu(const SerialImuRaw& msg) {
        // upon reception of latest IMU data from serial port, deserialize it
        // and bridge it to the ROS side
        imu.header.stamp.sec = msg.stamp.sec;
        imu.header.stamp.nanosec = msg.stamp.nanosec;

        imu.orientation.x = msg.orientation.x;
        imu.orientation.y = msg.orientation.y;
        imu.orientation.z = msg.orientation.z;
        imu.orientation.w = msg.orientation.w;

        imu.angular_velocity.x = msg.angular_velocity.x;
        imu.angular_velocity.y = msg.angular_velocity.y;
        imu.angular_velocity.z = msg.angular_velocity.z;

        imu.linear_acceleration.x = msg.linear_acceleration.x;
        imu.linear_acceleration.y = msg.linear_acceleration.y;
        imu.linear_acceleration.z = msg.linear_acceleration.z;

        ros_imu_pub->publish(imu);
    }

    void pub_ser_acc(const Float32& msg) {
        // upon reception of ROS side data, serialize it and bridge it
        // to the serial port side.
        ser_accel_cmd.data = msg.data;
        ser_accel_brake_pub.publish(ser_accel_cmd);
    }

    void pub_ser_steer(const Float32& msg) {
        // upon reception of ROS side data, serialize it and bridge it
        // to the serial port side.
        ser_steer_cmd.data = msg.data;
        ser_steer_pub.publish(ser_steer_cmd);
    }
};

}   // namespace rallycar

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rallycar::RallycarDriverNode)
