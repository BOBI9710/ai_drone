#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

double theta = 0;
double rot_alpha = 0;
double rot_beta_x, rot_beta_y, rot_beta_z = 0;
double q_x,q_y,q_z,q_w = 0;
int iter=0;
double a,b = 0;
const double pi = 3.14159265359;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        rot_beta_x = pi/2;
        rot_beta_y = pi/2;
        rot_beta_z = 0; 

        q_x = sin(theta/2.0)*cos(rot_beta_x);     
        q_y = sin(theta/2.0)*cos(rot_beta_y);
        q_z = sin(theta/2.0)*cos(rot_beta_z);
        q_w = cos(theta/2.0);
 
        pose.pose.position.x = a;            
        pose.pose.position.y = b;
        pose.pose.position.z = 2;
            
        pose.pose.orientation.x = q_x;
        pose.pose.orientation.y = q_y;
        pose.pose.orientation.z = q_z;
        pose.pose.orientation.w = q_w;

        iter = iter + 1;

        if(iter>=500 && iter<800){
         a = a + 0.02;
         b = 0; 
        }
        if(iter>=800 && iter<1000){
         theta = theta + pi/400;
        }
        if(iter>=1000 && iter<1300){
         a = a;
         b = b + 0.02; 
        }
        if(iter>=1300 && iter<1500){
         theta = theta + pi/400;
        }
        if(iter>=1500 && iter<1800){
         a = a - 0.02;
         b = b; 
        }       
        if(iter>=1800 && iter<2000){
         theta = theta + pi/400;
        }
        if(iter>=2000 && iter<2300){
         a = a;
         b = b - 0.02; 
        }
        if(iter>=2300 && iter<2500){
         theta = theta + pi/400;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
