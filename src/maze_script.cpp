#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

int estat = 0;
using Twist = geometry_msgs::Twist;
Twist vel;

struct Regions{

    double right = 0;
    double fright = 0;
    double front = 0;
    double fleft = 0;
    double left = 0;
};

void canvia_estat(int state){
    if(state != estat){
        estat = state;
    }
}

void comprova_estat(const Regions& r){
    double d = 0.3;
    double d2 = 0.5;
    /*
    if(r.front > d2){
            canvia_estat(2); // No hay obstáculos en frente, sigue recto.
    }
    else {
    
        if(r.fleft < d && r.fright > d){
            canvia_estat(0); // Evita estar atrapado entre dos obstáculos.
        }
        else{
            canvia_estat(1); // Hay un obstáculo delante, gira a la izquierda.
        } 
    }
    */
    
    if (r.front > d && r.fleft > d && r.fright > d) {
        canvia_estat(1);

    } else if (r.front > d2 && r.fleft < d && r.fright < d) {
        canvia_estat(0);
    
    } else if (r.front > d2 && r.fleft > d && r.fright < d) { 
        canvia_estat(2);

    } else if (r.front > d2 && r.fleft < d && r.fright > d) {
        canvia_estat(3);

    } else if (r.front < d2 && r.fleft > d && r.fright > d) {
        canvia_estat(2);

    } else if (r.front < d2 && r.fleft > d && r.fright < d) {
        canvia_estat(2);

    } else if (r.front < d2 && r.fleft < d && r.fright > d) {
        canvia_estat(3);

    } else if (r.front < d2 && r.fleft < d && r.fright < d) {
        canvia_estat(0);

    } else {
        ROS_INFO("Cas desconegut");
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Regions r;
    //mínim de tots els rangs depenent de la direcció
    r.right = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 143);
    r.fright = *std::min_element(msg->ranges.begin() + 144, msg->ranges.begin() + 287);
    //r.front = msg->ranges[0];
    r.fleft = *std::min_element(msg->ranges.begin() + 72, msg->ranges.begin() + 215);
    r.left = *std::min_element(msg->ranges.begin() + 576, msg->ranges.begin() + 719);
    
    double suma = 0;
    for(int i=-5;i <= 10;i++){
        suma += msg->ranges[i];
    }
    r.front = suma/11;
    
    //mínim amb 10, per evitar valors massa grans
    /*
    r.right = std::min(r.right, 10.0);
    r.fright = std::min(r.fright, 10.0);
    r.front = std::min(r.front, 10.0);
    r.fleft = std::min(r.fleft, 10.0);
    r.left = std::min(r.left, 10.0);*/

    
    ROS_INFO("laser FRONT:%f", r.front);
    ROS_INFO("-----------------------------");
    ROS_INFO("laser FRIGHT:%f", r.fright);
    ROS_INFO("laser FLEFT:%f", r.fleft);
    ROS_INFO("-----------------------------");
    ROS_INFO("laser RIGHT:%f", r.right);
    ROS_INFO("laser LEFT:%f", r.left);

    comprova_estat(r);
}

/*
void rang_minim(msg,inicial,final){
    int minim = msg->ranges[inicial]

    for(int i = inicial+1, i<=final, i++){
        if(msg->ranges[i] < minim)
            minim = msg->ranges[i]
    }
    return minim
}
*/

Twist find_wall(){      //Cas 0
    Twist vel;
    vel.linear.x = 0.1;
    vel.angular.z = 0.1;
    return vel;
}

Twist follow_the_wall(){    //Cas 1
    Twist vel;
    vel.linear.x = 0.3;
    vel.angular.z = 0;
    return vel;
}

Twist turn_left(){          //Cas 2
    Twist vel;
    vel.angular.z = 0.3;
    return vel;
}

Twist turn_right(){         //Cas3
    Twist vel;
    vel.angular.z = -0.3;
    return vel;
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "laberint");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<Twist>("cmd_vel", 20);
    ros::Subscriber sub = nh.subscribe("scan", 100, scanCallback);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ROS_INFO("vel:%f", vel.linear.x);
        ROS_INFO("estat:%d", estat);
        switch(estat){

            case 0: 
            vel = find_wall();
            break;
            
            case 1:
            vel = follow_the_wall();
            break;

            case 2:
            vel = turn_left();
            break;

            case 3:
            vel = turn_right();
            break;
        }
        pub.publish(vel);
        ros::spinOnce();

        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
