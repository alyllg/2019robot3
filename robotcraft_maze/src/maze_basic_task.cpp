#include <iostream>

#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Range.h>


#define STATE_LOST  0
#define STATE_CCW   1
#define STATE_WALL1 2
#define STATE_WALL2 3

bool sensR, sensL;

class Solver
{
private:
    ros::NodeHandle n;
    // Publisher
    ros::Publisher cmd_vel_pub;
    // Subscriber
    ros::Subscriber sensR_sub;
    ros::Subscriber sensL_sub;
    ros::Subscriber sensF_sub;
    ros::Subscriber laser_sub;

    int state = STATE_LOST; 
    double range_max = 0.8;
    double range_min = 0.1;
    double linear_speed = 0.3; // + : left - : right
    double angular_speed = 0.2; 
    float obstacle_distance;
    float obstacle_distance_left;
    float obstacle_distance_front;
    float obstacle_distance_right;

    void rightSensorCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        if(msg->range < range_max){
            sensR = true;
        }else{
            sensR = false;
        }
    }
    
    void leftSensorCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        if(msg->range < range_max){
            sensL = true;
        }else{
            sensL = false;
        }
    }
    
    void frontSensorCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
    }
    
    void move (double linear_speed){
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = linear_speed;
        cmd_vel_pub.publish(cmd_msg);
    }
    
    void turn (double angular_speed){
        geometry_msgs::Twist cmd_msg;
        cmd_msg.angular.z = angular_speed;
        cmd_vel_pub.publish(cmd_msg);
    }
    
    int lost(){
        if (!sensR && !sensL){
            move(linear_speed);
        }else{
            return STATE_CCW;
        }
        return STATE_LOST;
    }
    
    int ccw(){
        if (sensR || sensL){
            turn(angular_speed);
        }else{
            return STATE_WALL1;
        }
        return STATE_CCW;
    }
    
    int wall1(){
        if (!sensR){
            move(linear_speed);
            turn(-angular_speed);
        }else{
            return STATE_WALL2;
        }
        return STATE_WALL1;
    }
    
    int wall2(){
        if (!sensL){
            if (sensR){
                turn(angular_speed);
                move(linear_speed);
            }else{
                return STATE_WALL1;
            }
        }else{
            return STATE_CCW;
        }
        return STATE_WALL2;
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance_right = *std::min_element(msg->ranges.begin(),msg->ranges.begin()+80);
        obstacle_distance_front = *std::min_element(msg->ranges.begin()+81,msg->ranges.begin()+160.0);
        obstacle_distance_left = *std::min_element(msg->ranges.begin()+160.0,msg->ranges.end());
        obstacle_distance = *std::min_element(msg->ranges.begin(),msg->ranges.end()); 
    }
    
    
    
       
public:
    
     Solver(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        // Publisher for /cmd_vel
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
        // Subscriber for sensR:
        this->sensR_sub = n.subscribe("ir_right_sensor", 100, &Solver::rightSensorCallback, this);
        // Subscriber for sensL:
        this->sensL_sub = n.subscribe("ir_left_sensor", 100, &Solver::leftSensorCallback, this);
        // Subscriber for sensF:
        this->sensF_sub = n.subscribe("ir_front_sensor", 100, &Solver::frontSensorCallback, this);
        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan_0", 10, &Solver::laserCallback, this);

    }

    void run(){
      // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            switch(state) {
                
                case STATE_LOST : 
                    ROS_INFO("Lost");
                    state = lost();
                    break;
                    
                case STATE_CCW :
                    ROS_INFO("CCW");
                    state = ccw();
                    break;
                    
                case STATE_WALL1 :
                    ROS_INFO("Wall1");
                    state = wall1();
                    break;
                    
                case STATE_WALL2 :
                    ROS_INFO("Wall2");
                    state = wall2();
                    break;
                    
                default :
                    ROS_INFO("Invalid_State");
                    state = STATE_LOST;
            }
            
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
        
