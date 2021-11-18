#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ai_drone/error_data.h>
#include <math.h>

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
    float theta_data=0;
  
    void error_Callback(const ai_drone::error_data::ConstPtr& error);
};

void error_var::error_Callback(const ai_drone::error_data::ConstPtr& error){

  x_data = error->x_data;
  y_data = error->y_data;
  x_status = error->x_status;
  y_status = error->y_status;
  mode = error->mode_data;
  theta_data = error->theta_data;

}

int main(int argc, char **argv)
{
    double a,b=0.0;
    double e_a =0.0;
    double e_b =0.0;
    double e_theta =0;
    double K_P = 0.006;
    double K_D = 0.0002;
    double K_I = 0.017;
    int x_data_old = 40;
    int y_data_old = 40;
    int theta_data_old =0;
    double d_error_old_x =0.0;
    double d_error_old_y =0.0;
    double d_error_old_theta =0.0;
    int iter = 0;
    double theta = 0;
    double rot_alpha = 0;
    double rot_beta_x, rot_beta_y, rot_beta_z = 0;
    double q_x,q_y,q_z,q_w = 0;
    const double pi = 3.14159265359;

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
  
       //initial setting
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
   
     //// flight control start ////       

           //initial takeoff
            if (initial == 0){

                theta = 0;
                rot_beta_x = pi/2;
                rot_beta_y = pi/2;
                rot_beta_z = 0; 

                q_x = sin(theta/2.0)*cos(rot_beta_x);     
                q_y = sin(theta/2.0)*cos(rot_beta_y);
                q_z = sin(theta/2.0)*cos(rot_beta_z);
                q_w = cos(theta/2.0);
 
                pose.pose.position.x = 0;            
                pose.pose.position.y = 0;
                pose.pose.position.z = 5;
            
                pose.pose.orientation.x = q_x;
                pose.pose.orientation.y = q_y;
                pose.pose.orientation.z = q_z;
                pose.pose.orientation.w = q_w;
              }
 
            
            // mode1 (spawn -> start)            
            if (error_vars.mode == 1){ 
              
              initial = 1;
              iter = 0;

              e_a = error_vars.x_data*K_P + (error_vars.x_data - x_data_old) * K_D/0.02 + K_I*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x);
              e_b = error_vars.y_data*K_P + (error_vars.y_data - y_data_old) * K_D/0.02 + K_I*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y); 

              pose.pose.position.x = cos(theta)*e_a + -sin(theta)*e_b;
              pose.pose.position.y = sin(theta)*e_a + cos(theta)*e_b; 

              x_data_old = error_vars.x_data;
              y_data_old = error_vars.y_data;

              d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
              d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;

            }
            // mode2 (start -> line tracking)
            else if(error_vars.mode == 2){
            initial = 1;
            iter=0;

              e_b = error_vars.y_data*0.00001 + (error_vars.y_data - y_data_old) * 0.00001/0.02 + 0.003*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y);
              e_theta = -(error_vars.theta_data*0.006 + (error_vars.theta_data - theta_data_old) * 0.0002/0.02 + 0.017*((error_vars.theta_data + theta_data_old)*0.02*0.5 + d_error_old_theta));
          
              theta = e_theta;

              pose.pose.position.x = cos(theta)*(a) + -sin(theta)*(b + e_b);
              pose.pose.position.y = sin(theta)*(a) + cos(theta)*(b + e_b);

              q_x = sin(theta/2.0)*cos(rot_beta_x);     
              q_y = sin(theta/2.0)*cos(rot_beta_y);
              q_z = sin(theta/2.0)*cos(rot_beta_z);
              q_w = cos(theta/2.0);

              pose.pose.orientation.x = q_x;
              pose.pose.orientation.y = q_y;
              pose.pose.orientation.z = q_z;
              pose.pose.orientation.w = q_w;

              y_data_old = error_vars.y_data;
              d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;

              theta_data_old = error_vars.theta_data;
              d_error_old_theta = (error_vars.theta_data + theta_data_old)*0.02*0.5 + d_error_old_theta;


              a = a + 0.01;

            }
            else if(error_vars.mode == 3){
            initial = 1;

            iter = iter+1;
 
             if (iter == 1){
               a = e_a;
               b = e_b;
               e_a = 0;
               e_b = 0;

              }  

             pose.pose.position.x = cos(theta)*(a) + -sin(theta)*e_b;
             pose.pose.position.y = sin(theta)*(a) + cos(theta)*e_b;

              q_x = sin(theta/2.0)*cos(rot_beta_x);     
              q_y = sin(theta/2.0)*cos(rot_beta_y);
              q_z = sin(theta/2.0)*cos(rot_beta_z);
              q_w = cos(theta/2.0);

              pose.pose.orientation.x = q_x;
              pose.pose.orientation.y = q_y;
              pose.pose.orientation.z = q_z;
              pose.pose.orientation.w = q_w;

           
            }
            else if(error_vars.mode == 4){
            initial = 1;  
            } 
            else if(error_vars.mode == -1){ // error tack end -> update position, make error parameter 0 to use again
            initial = 1;
 
              iter = iter+1;
 
             if (iter == 1){
               a = e_a;
               b = e_b;
               e_a = 0;
               e_b = 0;

              }else if(iter >=20){
               
               pose.pose.position.x = a;
               pose.pose.position.y = b;
               pose.pose.position.z = 0.9; 
 
               pose.pose.orientation.x = q_x;
               pose.pose.orientation.y = q_y;
               pose.pose.orientation.z = q_z;
               pose.pose.orientation.w = q_w;

               a = a+ 0.01;
             }else{ }
                            
 
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
