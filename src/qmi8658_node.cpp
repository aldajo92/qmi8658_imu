#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unistd.h> // for usleep

// Include I2C library
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

// I2C address of the QMI8658
#define QMI8658_ADDR 0x6b

class QMI8658Node : public rclcpp::Node
{
public:
    QMI8658Node()
    : Node("qmi8658_imu_node")
    {
        // Initialize publisher
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::imu>("imu/data", 10);

        // Initialize I2C communication
        init_i2c();

        // Set up a timer to publish IMU data at a fixed interval
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&QMI8658Node::publish_imu_data, this));
    }

private:
    void init_i2c()
    {
        // Open the I2C device file
        file_ = open("/dev/i2c-1", O_RDWR);
        if (file_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus");
            return;
        }

        // Set the I2C address for communication
        if (ioctl(file_, I2C_SLAVE, QMI8658_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
            return;
        }

        // Initialization of QMI8658 could be added here if necessary
    }

    void publish_imu_data()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // Read data from the QMI8658 and populate imu_msg
        read_accel_gyro(imu_msg);

        // Publish the IMU data
        imu_publisher_->publish(imu_msg);
    }

    void read_accel_gyro(sensor_msgs::msg::Imu &imu_msg)
    {
        uint8_t buffer[12];
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;

        // Reading 12 bytes of data from the QMI8658 (6 bytes for accel, 6 bytes for gyro)
        if (read(file_, buffer, 12) != 12) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data from the QMI8658");
            return;
        }

        // Process accelerometer data
        accel_x = (int16_t)(buffer[1] << 8 | buffer[0]);
        accel_y = (int16_t)(buffer[3] << 8 | buffer[2]);
        accel_z = (int16_t)(buffer[5] << 8 | buffer[4]);

        imu_msg.linear_acceleration.x = accel_x * 9.81 / 16384.0; // Assuming 16G range
        imu_msg.linear_acceleration.y = accel_y * 9.81 / 16384.0;
        imu_msg.linear_acceleration.z = accel_z * 9.81 / 16384.0;

        // Process gyroscope data
        gyro_x = (int16_t)(buffer[7] << 8 | buffer[6]);
        gyro_y = (int16_t)(buffer[9] << 8 | buffer[8]);
        gyro_z = (int16_t)(buffer[11] << 8 | buffer[10]);

        imu_msg.angular_velocity.x = gyro_x * (M_PI / 180.0) / 131.0; // Assuming 250dps range
        imu_msg.angular_velocity.y = gyro_y * (M_PI / 180.0) / 131.0;
        imu_msg.angular_velocity.z = gyro_z * (M_PI / 180.0) / 131.0;
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int file_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QMI8658Node>());
    rclcpp::shutdown();
    return 0;
}
