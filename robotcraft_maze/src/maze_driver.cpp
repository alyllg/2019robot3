#include <iostream>

#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Range.h>


class Solver
{
private:
    ros::NodeHandle n;
    // Publisher
    ros::Publisher ir_front_sensor;
    ros::Publisher ir_right_sensor;
    ros::Publisher ir_left_sensor;
    // Subscriber
    ros::Subscriber sensR_sub;
    ros::Subscriber sensL_sub;
    ros::Subscriber sensF_sub;

    float obstacle_distance_left;
    float obstacle_distance_front;
    float obstacle_distance_right;
    

    sensor_msgs::Range	parameterMsgSensorFront() 
    {
        auto msg = sensor_msgs::Range();
        msg.header.frame_id = "/front_ir";
        msg.radiation_type = 1;
        msg.field_of_view = 0.034906585; // 2 degrees
        msg.min_range = 0.1; // m 
        msg.max_range = 0.8; // m 
        msg.range = obstacle_distance_front;
        return msg;
    }
    
    sensor_msgs::Range	parameterMsgSensorRight() 
    {
        auto msg = sensor_msgs::Range();
        msg.header.frame_id = "/right_ir";
        msg.radiation_type = 1;
        msg.field_of_view = 0.034906585; // 2 degrees
        msg.min_range = 0.1; // m 
        msg.max_range = 0.8; // m 
        msg.range = obstacle_distance_right;
        return msg;
    }
    
    sensor_msgs::Range	parameterMsgSensorLeft() 
    {
        auto msg = sensor_msgs::Range();
        msg.header.frame_id = "/left_ir";
        msg.radiation_type = 1;
        msg.field_of_view = 0.034906585; // 2 degrees
        msg.min_range = 0.1; // m 
        msg.max_range = 0.8; // m 
        msg.range = obstacle_distance_left;
        return msg;
    } 
    
    void leftSensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance_left = *std::min_element(msg->ranges.begin(),msg->ranges.end()); 
    }

    void frontSensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance_front = *std::min_element(msg->ranges.begin(),msg->ranges.end()); 
    }
    
    void rightSensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance_right = *std::min_element(msg->ranges.begin(),msg->ranges.end()); 
    }
    
    
    
       
public:
    
     Solver(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        // Publisher for /front
        this->ir_front_sensor = this->n.advertise<sensor_msgs::Range>("ir_front_sensor", 100);
        // Publisher for /right
        this->ir_right_sensor = this->n.advertise<sensor_msgs::Range>("ir_right_sensor", 100);       
        // Publisher for /left
        this->ir_left_sensor = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 100);       
        // Subscriber for sensR:
        this->sensR_sub = n.subscribe("base_scan_3", 100, &Solver::rightSensorCallback, this);
        // Subscriber for sensL:
        this->sensL_sub = n.subscribe("base_scan_2", 100, &Solver::leftSensorCallback, this);
        // Subscriber for sensF:
        this->sensF_sub = n.subscribe("base_scan_1", 100, &Solver::frontSensorCallback, this);

    }

    void run(){
      // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            
            // Paramater message of the free sensors

            auto msgSensorFront = parameterMsgSensorFront();
            auto msgSensorRight = parameterMsgSensorRight();
            auto msgSensorLeft = parameterMsgSensorLeft();

            // Publish the three sensors
            this->ir_front_sensor.publish(msgSensorFront);
            this->ir_right_sensor.publish(msgSensorRight);
            this->ir_left_sensor.publish(msgSensorLeft);
            
            
            ros::spinOnce();
            // And throttle the loop
            loop_rate.sleep();
        }
    }
};
        

int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "maze_solver");
    
    auto robot = Solver();
    robot.run();

    return 0;
}
        
