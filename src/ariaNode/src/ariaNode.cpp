/*  
*   A basic node for ros2 that runs with ariaCoda
*   To run use 'ros2 run ariaNode ariaNode -rp /dev/ttyUSB0'
*
*   Author: Kieran Quirke-Brown
*   Date: 12/01/2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

/* For Odom */
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

# include "Aria/Aria.h"

//used with signal handler as signal handler function doesn't accept parameters
bool stopRunning = false;

using namespace std::chrono_literals;
/*
*   Basic ROS node that updates velocity of pioneer robot, Aria doesn't like
*   being spun as a node therefore we just use a single subscriber
*   parameters:
*       forward and ratation speeds are float that are bound to the node
*       but point at the same location as the aria velocities
*/
class ariaNode : public rclcpp::Node {
    public:
        ariaNode(float* forwardSpeed, float* rotationSpeed, ArRobot* robotPtr) : Node("Aria_node") {
            currentForwardSpeed = forwardSpeed;
            currentRotationSpeed = rotationSpeed;
            robot = robotPtr;

            cmdVelSub = create_subscription<geometry_msgs::msg::Twist> (
                "cmd_vel", 10, std::bind(&ariaNode::cmdVelCallback, this, std::placeholders::_1)
            );
            
            /* Odom */
            posePub = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_pose", 10);

            poseTimer = this->create_wall_timer(50ms, std::bind(&ariaNode::publishPose, this));
        }

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            
            double linearSpeed = msg->linear.x;
            double angularSpeed = msg->angular.z;

            *currentForwardSpeed = linearSpeed;
            *currentRotationSpeed = angularSpeed;

            RCLCPP_DEBUG(this->get_logger(), "message received.");

        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        float* currentForwardSpeed;
        float* currentRotationSpeed;
    
};

// Deals with ctl+c handling to stop the motors correctly.
void my_handler(int s){
           printf("Caught signal %d\n",s);
           stopRunning = true;
}

//Odom Function
void publishPose() {
    ArPose p = robot->getPose();

    geometry_msgs::msg::Pose2D msg;
    msg.x = p.getX() / 1000.0;
    msg.y = p.getY() / 1000.0;
    msg.theta = p.getTh() * M_PI / 180.0;

    posePub->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot* robot;
    robot = new ArRobot();

    signal(SIGINT, my_handler);
    
    // RCLCPP_DEBUG(this->get_logger(),"Trying to connect to robot...");
    ArRobotConnector robotConnector(&parser, robot);
    if(!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    robot->setAbsoluteMaxTransVel(400);

    float forwardSpeed = 0.0;
    float rotationSpeed = 0.0;
    
    
    // RCLCPP_DEBUG(aNode->get_logger(),"Run Async");
    robot->runAsync(true);
    // RCLCPP_DEBUG(aNode->get_logger(),"Enable Motors");
    robot->enableMotors();

    auto aNode = std::make_shared<ariaNode>(&forwardSpeed, &rotationSpeed, robot);
    RCLCPP_DEBUG(aNode->get_logger(),"Before Spin!...");

    /*
     *   Aria does not like to run in a ros node therefore we run a while loop
     *   that continuously spins the node to update velocities which are 
     *   then sent using the normal Aria commands.
    */
    while (!stopRunning) {
        rclcpp::spin_some(aNode);
        // RCLCPP_DEBUG(aNode->get_logger(), "sending motor command.");
            robot->lock();
            robot->setVel(forwardSpeed * 500);
            robot->setRotVel(rotationSpeed * 50);
            robot->unlock();
            // RCLCPP_DEBUG(aNode->get_logger(), "motor command sent.");
            // RCLCPP_DEBUG(aNode->get_logger(), "forward speed is now %f.", forwardSpeed);
            // RCLCPP_DEBUG(aNode->get_logger(), "rotational speed is now %f.", rotationSpeed);
    }
    RCLCPP_DEBUG(aNode->get_logger(), "After Spin");

    robot->disableMotors();
    robot->stopRunning();
    // wait for the thread to stop
    robot->waitForRunExit();

    // exit
    RCLCPP_DEBUG(aNode->get_logger(), "ending Aria node");
    Aria::exit(0);
    return 0;
}