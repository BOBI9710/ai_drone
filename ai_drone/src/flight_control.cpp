#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ai_drone/error_data.h>

int count=0;
int initial=0;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//error_Callback function and variable settings

class error_var
{
    public:
    int mode=0;
    int x_data=0;
    int y_data=0;
    int x_status=0;
    int y_status=0;
  
    void error_Callback(const ai_drone::error_data::ConstPtr& error);
};

void error_var::error_Callback(const ai_drone::error_data::ConstPtr& error){

  x_data = error->x_data;
  y_data = error->y_data;
  x_status = error->x_status;
  y_status = error->y_status;
  mode = error->mode_data;

}

int main(int argc, char **argv)
{
    double a,b=0.0;
    int x_data_old = 40;
    int y_data_old = 40;
    double d_error_old_x =0.0;
    double d_error_old_y =0.0;   

    ros::init(argc, argv, "flight_example_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //subscribe error_data.msg
    error_var error_vars;
    ros::Subscriber error_data_sub = nh.subscribe("error_data_msg",30,&error_var::error_Callback, &error_vars);

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

            if (initial == 0){
                pose.pose.position.x = 0;            
                pose.pose.position.y = 0;
                pose.pose.position.z = 5;
            
                /*pose.pose.orientation.x = 1;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 0;*/ 

              }
            
                        
            if (error_vars.mode == 1){
              
              initial = 1;  

              a = error_vars.x_data*0.0055 + (error_vars.x_data - x_data_old) * 0.0001/0.02 + 0.005*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x);
              b = error_vars.y_data*0.0055 + (error_vars.y_data - y_data_old) * 0.0001/0.02 + 0.005*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y); 

              pose.pose.position.x = a;
              pose.pose.position.y = b; 

              x_data_old = error_vars.x_data;
              y_data_old = error_vars.y_data;

              d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
              d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;

            }
            else if(error_vars.mode == 2){
            initial = 1;  
            }
            else if(error_vars.mode == 3){
            initial = 1;  
            }
            else if(error_vars.mode == 4){
            initial = 1;  
            } 
            else if(error_vars.mode == -1){
            initial = 1; 
            pose.pose.position.z = 0.5; 
 
            pose.pose.orientation.x = 1;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 0;   
            }  
            else{ //mode=default;
                    
            }


                     
            


            //turn yaw 90deg
            /*pose.pose.orientation.w = 0.7071 ;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0.7071; 
            */
         
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
