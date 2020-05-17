#pragma once

#include <ros/ros.h>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "vesc_sim.h"

#include "std_msgs/Float32MultiArray.h"

constexpr const char* COMMAND_POSITION = "/commands/servo/position";
constexpr const char* COMMAND_THROTTLE = "/commands/motor/speed";
constexpr const char* COMMAND_BRAKE = "/commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* CMD_VEL = "cmd_vel";

namespace simulation
{
    constexpr const char* WHEEL_LEFT_BACK_VELOCITY = "/racer/left_wheel_back_velocity_controller/command";
    constexpr const char* WHEEL_LEFT_FRONT_VELOCITY = "/racer/left_wheel_front_velocity_controller/command";
    constexpr const char* WHEEL_RIGHT_BACK_VELOCITY = "/racer/right_wheel_back_velocity_controller/command";
    constexpr const char* WHEEL_RIGHT_FRONT_VELOCITY = "/racer/right_wheel_front_velocity_controller/command";

    constexpr const char* FRONTFRONT_LEFT_STEERING_POSITION = "/racer/frontfront_left_steering_hinge_position_controller/command";
    constexpr const char* FRONTFRONT_RIGHT_STEERING_POSITION = "/racer/frontfront_right_steering_hinge_position_controller/command";
    constexpr const char* BACKFRONT_LEFT_STEERING_POSITION = "/racer/backfront_left_steering_hinge_position_controller/command";
    constexpr const char* BACKFRONT_RIGHT_STEERING_POSITION = "/racer/backfront_right_steering_hinge_position_controller/command";

    constexpr const char*  BACK_WHEEL_LEFT_BACK_VELOCITY     = "/racer2/left_wheel_back_velocity_controller/command";
    constexpr const char*  BACK_WHEEL_LEFT_FRONT_VELOCITY    = "/racer2/left_wheel_front_velocity_controller/command";
    constexpr const char*  BACK_WHEEL_RIGHT_BACK_VELOCITY    = "/racer2/right_wheel_back_velocity_controller/command";
    constexpr const char*  BACK_WHEEL_RIGHT_FRONT_VELOCITY   = "/racer2/right_wheel_front_velocity_controller/command";

    constexpr const char*  FRONTBACK_LEFT_STEERING_POSITION  = "/racer2/frontfront_left_steering_hinge_position_controller/command";
    constexpr const char*  FRONTBACK_RIGHT_STEERING_POSITION = "/racer2/frontfront_right_steering_hinge_position_controller/command";
    constexpr const char*  BACKBACK_LEFT_STEERING_POSITION   = "/racer2/backfront_left_steering_hinge_position_controller/command";
    constexpr const char*  BACKBACK_RIGHT_STEERING_POSITION  = "/racer2/backfront_right_steering_hinge_position_controller/command";

//    constexpr const char* BACK_WHEEL_LEFT_BACK_VELOCITY     =   "/racer2/back_left_wheel_back_velocity_controller/command";
//    constexpr const char* BACK_WHEEL_LEFT_FRONT_VELOCITY    =  "/racer2/back_left_wheel_front_velocity_controller/command";
//    constexpr const char* BACK_WHEEL_RIGHT_BACK_VELOCITY    =  "/racer2/back_right_wheel_back_velocity_controller/command";
//    constexpr const char* BACK_WHEEL_RIGHT_FRONT_VELOCITY   = "/racer2/back_right_wheel_front_velocity_controller/command";
//
//    constexpr const char* FRONTBACK_LEFT_STEERING_POSITION  =     "/racer2/frontback_left_steering_hinge_position_controller/command";
//    constexpr const char* FRONTBACK_RIGHT_STEERING_POSITION =    "/racer2/frontback_right_steering_hinge_position_controller/command";
//    constexpr const char* BACKBACK_LEFT_STEERING_POSITION   =      "/racer2/backback_left_steering_hinge_position_controller/command";
//    constexpr const char* BACKBACK_RIGHT_STEERING_POSITION  =     "/racer2/backback_right_steering_hinge_position_controller/command";
};

/**
 * @brief Class to convert Drive Parameter Messages into single messages
 *
 * Class to convert Drive Parameter Messages (steering angle and velocity)
 * into single messages for each wheel velocity and for front wheel steering angles
 * based on Ackermann equations.
 */
class VESCSimulationDriver
{
    public:
    /**
     * @brief Constructor that creates subscribers and publishers
     *
     */
    VESCSimulationDriver();

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param motor_speed contains electrical RPM
     */
    void motorSpeedCallback(const std_msgs::Float64::ConstPtr& throttle_message);

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param servo_position contains angle with value 0 upto 1
     */
    void servoPositionCallback(const std_msgs::Float64::ConstPtr& servo_position);

    /**
     * @brief Callback for ROS Subscriber
     *
     * @param motor_brake
     */
    void motorBrakeCallback(const std_msgs::Float64::ConstPtr& motor_brake);
    void chatterCallback(const std_msgs::Float32 msg);
    void estCallback(const std_msgs::Float32MultiArray msg);
    private:
    /**
     * @brief Return type to return two angles as return value
     *
     */
    struct AckermannSteeringAngles
    {
        double left_wheel_angle;
        double right_wheel_angle;
    };

    /**
     * @brief Calculates the angles of the front wheels based on the angle of the center of the front axis with
     * Ackermann equation/trigonometry
     *
     * @param angle Angle of the center of the front axis
     * @return Angles One Ackermann angle for each front wheel
     */
    AckermannSteeringAngles calculateSteeringAngles(const double& angle);

    /**
     * @brief ROS Handle
     *
     */
    ros::NodeHandle m_node_handle;

    /**
     * @brief ROS Subscriber for servo position messages
     * TOPIC: "/commands/servo/position"
     */
    ros::Subscriber m_servo_position_subscriber;

    /**
     * @brief ROS Subscriber for motor speed messages
     * TOPIC: "/commands/motor/speed"
     */
    ros::Subscriber m_motor_speed_subscriber;

    /**
     * @brief ROS Subscriber for brake messages
     * TOPIC: "/commands/motor/brake"
     */
    ros::Subscriber m_motor_brake_subscriber;

    ros::Subscriber subs;
    ros::Subscriber car_est;
    ros::Publisher m_left_rear_wheel_velocity_publisher;
    ros::Publisher m_right_rear_wheel_velocity_publisher;
    ros::Publisher m_left_front_wheel_velocity_publisher;
    ros::Publisher m_right_front_wheel_velocity_publisher;

    ros::Publisher m_frontfront_left_steering_position_publisher;
    ros::Publisher m_frontfront_right_steering_position_publisher;
    ros::Publisher m_backfront_left_steering_position_publisher;
    ros::Publisher m_backfront_right_steering_position_publisher;

    ros::Publisher m_back_left_rear_wheel_velocity_publisher;
    ros::Publisher m_back_right_rear_wheel_velocity_publisher;
    ros::Publisher m_back_left_front_wheel_velocity_publisher;
    ros::Publisher m_back_right_front_wheel_velocity_publisher;

    ros::Publisher m_frontback_left_steering_position_publisher;
    ros::Publisher m_frontback_right_steering_position_publisher;
    ros::Publisher m_backback_left_steering_position_publisher;
    ros::Publisher m_backback_right_steering_position_publisher;

    VESCSimulator m_VESC_simulator;
};