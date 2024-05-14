#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

int estat = 3;
const double PI = 3.14159265358979323846;
double angle = PI/2;
using Twist = geometry_msgs::Twist;
geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;
Twist vel;
int casella[2] = {0, 0};
double velx = 0.1;
double angz = 0;
double alfa = 0;

struct Regions{
    
    double right = 0;
    double front = 0;
    double left = 0;
};

void canvia_estat(int state){
    if(state != estat){
        estat = state;
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose.theta = yaw;
    pub_pose2d.publish(current_pose);

    casella[0] = static_cast<int>(std::floor(current_pose.x + 8));
    
    casella[1] = static_cast<int>(std::floor(current_pose.y + 8));

    ROS_INFO("(%d, %d)", casella[0],casella[1]);

}

void comprova_estat(const Regions& r){
    double d = 0.5;
    double d2 = 0.7;
    
    // Cas 0: 180º
    // Cas 1: 90º dreta
    // Cas 2: 90º esquerra
    // Cas 3: tira recte


   if(r.front < d2){
        velx = 0;
        ROS_INFO("PARAA PUTEROOO");
        alfa = current_pose.theta;
        if(r.right > r.left and r.right > d and estat == 3){
            canvia_estat(1);
        }
        else if(r.left > r.right and r.left > d and estat == 3){
            canvia_estat(2);
        }
        else if(r.left < d and r.right < d and estat == 3){
            canvia_estat(0);
        }      
    }
    else{
        estat = 3;
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Regions r;

    r.front = msg->ranges[0];
    r.right = msg->ranges[90];
    r.left = msg->ranges[270];
    
    ROS_INFO("laser FRONT:%f", r.front);
    ROS_INFO("-----------------------------");
    ROS_INFO("laser right:%f", r.right);
    ROS_INFO("laser left:%f", r.left);

    comprova_estat(r);
}

double modul(double x, double y) {
    return x - y * floor(x / y);
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;

    ROS_INFO("start");
    
    ros::init(argc, argv, "maze_pub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 100, scanCallback);
    ros::Subscriber sub_odometry = n.subscribe("odom", 100, odomCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100); //for sensors the value after, should be higher to get a more accurate result (queued)
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("turtlebot_pose2d", 100);
    ros::Rate rate(10);                                                             //the larger the value, the "smoother", try value of 1 to see "jerk" velment

    Twist vel;

    float distance = 1;
    double angle = PI/2;

    float x_now = current_pose.x;
    float y_now = current_pose.y;
    float theta_now = current_pose.theta;

        //vel forward
    ROS_INFO("vel forward");
    ros::Time start = ros::Time::now();

    ros::Time start_turn = ros::Time::now();

    while(ros::ok()){
        vel.linear.x = velx;

        if(estat == 3){
            vel.linear.x = 0.1;
            vel.angular.z = 0;
        }

        if(estat == 0){
            while(fabs(current_pose.theta - theta_now) - PI < 0.1 and modul(current_pose.theta, PI/2) > 0.2){
                vel.linear.x = 0;
                vel.angular.z = 0.1;
                

                pub.publish(vel);
                ros::spinOnce();
                rate.sleep();
                
            }
            theta_now = current_pose.theta;
            estat = 3;
        }

        if(estat == 1){
            while(fabs(current_pose.theta - theta_now) - PI/2 < 0.1 and modul(current_pose.theta, PI/2) > 0.2){
                vel.linear.x = 0;
                vel.angular.z = 0.1;
                

                pub.publish(vel);
                ros::spinOnce();
                rate.sleep();
            }
            
            theta_now = current_pose.theta;
            estat = 3;
        }

        if(estat == 2){
            while(fabs(current_pose.theta - theta_now) - PI/2 < 0.1 and modul(current_pose.theta, PI/2) > 0.2){
                vel.linear.x = 0;
                vel.angular.z = -0.1;

                pub.publish(vel);
                ros::spinOnce();
                rate.sleep();
            }
            theta_now = current_pose.theta;
            estat = 3;
        }

        pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


