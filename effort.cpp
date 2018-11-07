#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
 
#include <math.h>
 
geometry_msgs::Pose2D current_pose;
//ros::Publisher pub_pose2d, 

//declare variables
double x_ini, y_ini;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    


    // position in space
   //current_pose.x = x_ini;
   //current_pose.y = y_ini;

    // angular position
   current_pose.theta = yaw;
   ROS_INFO("THETA %f x_position %f y_position %f ", current_pose.theta, current_pose.x, current_pose.y);
   //ROS_INFO("x_position %f ", current_pose.x);
    //pub_pose2d.publish(current_pose);
}
 
int main(int argc, char **argv)
{
    //matrix gains error initialisation
    double k_a = 0.8;
    double k_b = -0.15;
    double k_rho = 0.3;
    const double PI  =3.141592653589793238463;
    double Rcirc= 1;

    //declaration of variables
    double Xt, Yt, rho, alpha, beta, THETAt, linvel, angvel;

    //destination of goal
    double x_targetpose = 4;
    double y_targetpose = 8;
    double theta_targetpose = -1.5707963268;
    double statement = 0;

    
    
    ros::init(argc, argv, "effort");

    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe<nav_msgs::Odometry>("/odom_diffdrive", 10, odomCallback);
    ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10 );
    //for sensors the value after , should be higher to get a more accurate result (queued)
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
    
    //code of controller
    Xt = cos(theta_targetpose)*(current_pose.x-x_targetpose)+sin(theta_targetpose)*(current_pose.y-y_targetpose);
    Yt = -sin(theta_targetpose)*(current_pose.x-x_targetpose)+cos(theta_targetpose)*(current_pose.y-y_targetpose);
    THETAt = current_pose.theta-theta_targetpose;
    rho = sqrt(pow(-Xt,2.0)+pow(-Yt,2.0));
    alpha = atan2(-Yt,-Xt)-THETAt;
    if (alpha>3.14159265359){
	alpha=alpha-2*3.14159265359;
    }
    if (alpha<-3.14159265359){
	alpha=alpha+2*3.14159265359;
    }

    beta = atan2(-Yt,-Xt);
    if (beta>3.14159265359){
	beta=beta-2*3.14159265359;
    }
    if (beta<-3.14159265359){
	beta=beta+2*3.14159265359;
    }
    linvel = k_rho*rho;
    angvel = k_a*alpha+k_b*beta;

    statement=theta_targetpose-current_pose.theta;
    if (statement>3.14159265359){
	statement=statement-2*3.14159265359;
    }
    if (statement<-3.14159265359){
	statement=statement+2*3.14159265359;
    }




  
ros::Time start_turn = ros::Time::now();
 

        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z = 0;
        movement_pub.publish(move);
    


    //move forward again
    ros::Time start2 = ros::Time::now();
    while(ros::ok() && ((rho>=0.01)||abs(statement>=0.015)))
    {
        geometry_msgs::Twist move;
        
        move.linear.x = linvel;
        move.angular.z = angvel;
        movement_pub.publish(move);
        ros::spinOnce();

        Xt = cos(theta_targetpose)*(current_pose.x-x_targetpose)+sin(theta_targetpose)*(current_pose.y-y_targetpose);
        Yt = -sin(theta_targetpose)*(current_pose.x-x_targetpose)+cos(theta_targetpose)*(current_pose.y-y_targetpose);
        THETAt = current_pose.theta-theta_targetpose;
        rho = sqrt(pow(-Xt,2.0)+pow(-Yt,2.0));
        alpha = atan2(-Yt,-Xt)-THETAt;
        if (alpha>3.14159265359){
		alpha=alpha-2*3.14159265359;
        }
        if (alpha<-3.14159265359){
		alpha=alpha+2*3.14159265359;
        }

        beta = atan2(-Yt,-Xt);
        if (beta>3.14159265359){
		beta=beta-2*3.14159265359;
        }
        if (beta<-3.14159265359){
		beta=beta+2*3.14159265359;
        }
        linvel = k_rho*rho;
        angvel = k_a*alpha+k_b*beta;

        statement=theta_targetpose-current_pose.theta;
        if (statement>3.14159265359){
		statement=statement-2*3.14159265359;
        }
        if (statement<-3.14159265359){
		statement=statement+2*3.14159265359;
        }

        rate.sleep();
    }
        
    
    
    // just stop
    while(ros::ok()) {
        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z =0 ;
        movement_pub.publish(move);
    
        ros::spinOnce();
        ROS_INFO("Finally position x %f ", current_pose.x);
        ROS_INFO("Finally position y %f ", current_pose.y);

        rate.sleep();
    }
    
    return 0;
}
