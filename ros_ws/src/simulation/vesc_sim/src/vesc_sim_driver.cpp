#include "vesc_sim_driver.h"
#include "car_config.h"
#include <cmath>



float car;
float car_esti[3] = {0};


VESCSimulationDriver::VESCSimulationDriver()
{
    this->m_servo_position_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_POSITION, 1,
                                                         &VESCSimulationDriver::servoPositionCallback, this);

    this->m_motor_speed_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_THROTTLE, 1, &VESCSimulationDriver::motorSpeedCallback,
                                                         this);

    this->m_motor_brake_subscriber =
        this->m_node_handle.subscribe<std_msgs::Float64>(COMMAND_BRAKE, 1, &VESCSimulationDriver::motorBrakeCallback,
                                                         this);

    this->subs = this->m_node_handle.subscribe<std_msgs::Float32>("/car_select", 1, &VESCSimulationDriver::chatterCallback, this);
    this->car_est = this->m_node_handle.subscribe<std_msgs::Float32MultiArray>("/car_est", 1, &VESCSimulationDriver::estCallback, this);

    this->m_left_rear_wheel_velocity_publisher =   this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_LEFT_BACK_VELOCITY, 10);
    this->m_right_rear_wheel_velocity_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_RIGHT_BACK_VELOCITY, 10);
    this->m_left_front_wheel_velocity_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_LEFT_FRONT_VELOCITY, 10);
    this->m_right_front_wheel_velocity_publisher = this->m_node_handle.advertise<std_msgs::Float64>(simulation::WHEEL_RIGHT_FRONT_VELOCITY, 10);

    this->m_frontfront_left_steering_position_publisher=   this->m_node_handle.advertise<std_msgs::Float64>(simulation::FRONTFRONT_LEFT_STEERING_POSITION, 10);
    this->m_frontfront_right_steering_position_publisher = this->m_node_handle.advertise<std_msgs::Float64>(simulation::FRONTFRONT_RIGHT_STEERING_POSITION, 10);
    this->m_backfront_left_steering_position_publisher=    this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACKFRONT_LEFT_STEERING_POSITION, 10);
    this->m_backfront_right_steering_position_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACKFRONT_RIGHT_STEERING_POSITION, 10);

    this->m_back_left_rear_wheel_velocity_publisher =   this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACK_WHEEL_LEFT_BACK_VELOCITY, 10);
    this->m_back_right_rear_wheel_velocity_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACK_WHEEL_RIGHT_BACK_VELOCITY, 10);
    this->m_back_left_front_wheel_velocity_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACK_WHEEL_LEFT_FRONT_VELOCITY, 10);
    this->m_back_right_front_wheel_velocity_publisher = this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACK_WHEEL_RIGHT_FRONT_VELOCITY, 10);

    this->m_frontback_left_steering_position_publisher=    this->m_node_handle.advertise<std_msgs::Float64>(simulation::FRONTBACK_LEFT_STEERING_POSITION, 10);
    this->m_frontback_right_steering_position_publisher =  this->m_node_handle.advertise<std_msgs::Float64>(simulation::FRONTBACK_RIGHT_STEERING_POSITION, 10);
    this->m_backback_left_steering_position_publisher=     this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACKBACK_LEFT_STEERING_POSITION, 10);
    this->m_backback_right_steering_position_publisher =   this->m_node_handle.advertise<std_msgs::Float64>(simulation::BACKBACK_RIGHT_STEERING_POSITION, 10);


    m_VESC_simulator.start();
}

void VESCSimulationDriver::chatterCallback(const std_msgs::Float32 msg)
{
    car=msg.data;
}
void VESCSimulationDriver::estCallback(const std_msgs::Float32MultiArray msg)
{
    car_esti[0] =msg.data.at(0);
    car_esti[1] =msg.data.at(1);
    car_esti[2] =msg.data.at(2);
}

void VESCSimulationDriver::motorSpeedCallback(const std_msgs::Float64::ConstPtr& throttle_message)
{
    std_msgs::Float64 throttle;
    throttle.data = throttle_message->data * car_config::ERPM_TO_RAD_PER_SEC / car_config::TRANSMISSION;

    m_VESC_simulator.setSpeed(throttle_message->data);

    if (car==0){
        this->m_left_rear_wheel_velocity_publisher.publish(throttle);
        this->m_right_rear_wheel_velocity_publisher.publish(throttle);
        this->m_left_front_wheel_velocity_publisher.publish(throttle);
        this->m_right_front_wheel_velocity_publisher.publish(throttle);
    }
    else{
        this->m_back_left_rear_wheel_velocity_publisher.publish(throttle);
        this->m_back_right_rear_wheel_velocity_publisher.publish(throttle);
        this->m_back_left_front_wheel_velocity_publisher.publish(throttle);
        this->m_back_right_front_wheel_velocity_publisher.publish(throttle);
    }



}

void VESCSimulationDriver::motorBrakeCallback(const std_msgs::Float64::ConstPtr& motor_brake)
{
    if (motor_brake->data != 0)
    {
        std_msgs::Float64 throttle;
        throttle.data = 0;

        m_VESC_simulator.setSpeed(0);

        this->m_left_rear_wheel_velocity_publisher.publish(throttle);
        this->m_right_rear_wheel_velocity_publisher.publish(throttle);
        this->m_left_front_wheel_velocity_publisher.publish(throttle);
        this->m_right_front_wheel_velocity_publisher.publish(throttle);
    }
}

void VESCSimulationDriver::servoPositionCallback(const std_msgs::Float64::ConstPtr& servo_position)
{
    double angle = (servo_position->data - car_config::STEERING_TO_SERVO_OFFSET) / car_config::STEERING_TO_SERVO_GAIN;
    AckermannSteeringAngles angles = calculateSteeringAngles(angle);

    m_VESC_simulator.setServoAngle(servo_position->data);

    std_msgs::Float64 left_wheel;
    std_msgs::Float64 right_wheel;
    std_msgs::Float64 left_wheel2;
    std_msgs::Float64 right_wheel2;
    left_wheel.data = angles.left_wheel_angle;
    right_wheel.data = angles.right_wheel_angle;
    left_wheel2.data = -angles.left_wheel_angle;
    right_wheel2.data = -angles.right_wheel_angle;

//    this->m_left_steering_position_publisher.publish(left_wheel);
//    this->m_right_steering_position_publisher.publish(right_wheel);
//    this->m_back_left_steering_position_publisher.publish(right_wheel2);
//    this->m_back_right_steering_position_publisher.publish(left_wheel2);
    car=0;
    if (car==0){
        this->m_frontfront_left_steering_position_publisher.publish(left_wheel);
        this->m_frontfront_right_steering_position_publisher.publish(right_wheel);
        this->m_backfront_left_steering_position_publisher.publish(left_wheel2);
        this->m_backfront_right_steering_position_publisher.publish(right_wheel2);

        double angle2 = -3*car_esti[0];
//        ROS_INFO("%f", angle2);
        AckermannSteeringAngles angles2 = calculateSteeringAngles(angle2);
        left_wheel.data = angles2.left_wheel_angle;
        right_wheel.data = angles2.right_wheel_angle;
        left_wheel2.data = -angles2.left_wheel_angle;
        right_wheel2.data = -angles2.right_wheel_angle;

        this->m_frontback_left_steering_position_publisher.publish(left_wheel);
        this->m_frontback_right_steering_position_publisher.publish(right_wheel);
        this->m_backback_left_steering_position_publisher.publish(left_wheel2);
        this->m_backback_right_steering_position_publisher.publish(right_wheel2);
    }
    else{
        this->m_frontback_left_steering_position_publisher.publish(left_wheel);
        this->m_frontback_right_steering_position_publisher.publish(right_wheel);
        this->m_backback_left_steering_position_publisher.publish(left_wheel2);
        this->m_backback_right_steering_position_publisher.publish(right_wheel2);
    }


}

VESCSimulationDriver::AckermannSteeringAngles VESCSimulationDriver::calculateSteeringAngles(const double& angle)
{
    AckermannSteeringAngles angles;
    double radius = std::tan(angle + M_PI / 2) * car_config::WHEELBASE;
    angles.left_wheel_angle = -std::atan(car_config::WHEELBASE / (radius + car_config::REAR_WHEEL_DISTANCE / 2));
    angles.right_wheel_angle = -std::atan(car_config::WHEELBASE / (radius - car_config::REAR_WHEEL_DISTANCE / 2));
    return angles;
}
