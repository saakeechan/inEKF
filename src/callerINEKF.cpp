#include "estimator/InEKF_estimator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <cmath>

//put constructor and member functions inside the class

class InEKFNode : public rclcpp::Node {
public:
    InEKFNode() : Node("inekf_node") {
        // Load parameters and initialize estimator
        init_estimator();
        
        // Create subscriber for LowState messages
        low_state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
            "/lowstate",  // Topic name - adjust as needed
            10,
            std::bind(&InEKFNode::low_state_callback, this, std::placeholders::_1));
        
        // Create publisher for estimated state (TODO: define output message type)
        // state_pub_ = this->create_publisher<YourStateMsg>("/inekf/state", 10);
        
        RCLCPP_INFO(this->get_logger(), "InEKF Node initialized and ready");
    }

private:
    void init_estimator() {
        // Initialize sensor and contact vectors
        Sensor_.resize(30, 1);  // [gyro(3), acc(3), encoder(12), encoderdot(12)]
        Sensor_.setZero();
        Contact_.resize(estimator_IEKF.robot.num_legs, false);
        
        RCLCPP_INFO(this->get_logger(), "Estimator initialized from YAML configuration");
    }
    
    void low_state_callback(const std::shared_ptr<const unitree_hg::msg::LowState> message) {
        // Extract IMU data (gyroscope and accelerometer)
        // Sensor_ structure: [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, 
        //                     q1...q12 (encoders), dq1...dq12 (encoder velocities)]
        
        // IMU Gyroscope (angular velocity)
        Sensor_(0, 0) = message->imu_state.gyroscope[0];  // gyro_x
        Sensor_(1, 0) = message->imu_state.gyroscope[1];  // gyro_y
        Sensor_(2, 0) = message->imu_state.gyroscope[2];  // gyro_z
        
        // IMU Accelerometer
        Sensor_(3, 0) = message->imu_state.accelerometer[0];  // acc_x
        Sensor_(4, 0) = message->imu_state.accelerometer[1];  // acc_y
        Sensor_(5, 0) = message->imu_state.accelerometer[2];  // acc_z
        
        // Motor states (encoders and velocities)
        // Assuming message has NUM_MOTOR motors (typically 12 for humanoid)
        for (size_t i = 0; i < message->motor_state.size() && i < 12; ++i) {
            Sensor_(6 + i, 0) = message->motor_state[i].q;        // Joint positions
            Sensor_(18 + i, 0) = message->motor_state[i].dq;      // Joint velocities
        }
        
        // Extract contact information if available
        // TODO: Determine contact detection logic from message data
        // For now, using simple heuristic or you can subscribe to separate contact topic
        // Contact_.assign(estimator_IEKF.robot.num_legs, false);  // Default: no contact
        
        // Update mode machine if needed
        if (mode_machine_ != message->mode_machine) {
            if (mode_machine_ == 0) {
                RCLCPP_INFO(this->get_logger(), "G1 type: %d", 
                           static_cast<unsigned>(message->mode_machine));
            }
            mode_machine_ = message->mode_machine;
        }
        
        // Call the estimator's Onestep function
        estimator_IEKF.Onestep(Sensor_, Contact_);
        
        // Publish the estimated state
        publish_state();
    }
    
    void publish_state() {
        // TODO: Create and publish state message
        // Example structure (define your own message type):
        // auto msg = std::make_shared<YourStateMsg>();
        // msg->position = {state_.position[0], state_.position[1], state_.position[2]};
        // msg->velocity = {state_.velocity[0], state_.velocity[1], state_.velocity[2]};
        // msg->orientation = {state_.orientation[0], ...};
        // state_pub_->publish(*msg);
        
        // For debugging, log the state
        RCLCPP_DEBUG(this->get_logger(), "State updated - Position: [%.3f, %.3f, %.3f]",
                    state_.position[0], state_.position[1], state_.position[2]);
    }

    // Estimator and data members
    InEKFilter estimator_IEKF{"../config/initialization_config.yaml", "../config/digit_config.yaml"};  // Constructor with config paths

    Eigen::MatrixXd Sensor_;
    std::vector<bool> Contact_;
    ROBOT_STATES state_;
    
    // ROS2 communication
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;
    // rclcpp::Publisher<YourStateMsg>::SharedPtr state_pub_;
    
    // Mode tracking
    uint8_t mode_machine_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InEKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}