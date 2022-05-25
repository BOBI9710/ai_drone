#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Autonomous_landing/error_data.h>
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
    int r_data=0;
    int x_status=0;
    int y_status=0;
    float phi_data=0;
  
    void error_Callback(const Autonomous_landing::error_data::ConstPtr& error);
};

void error_var::error_Callback(const Autonomous_landing::error_data::ConstPtr& error){

  x_data = error->x_data;
  y_data = error->y_data;
  r_data = error->r_data;
  x_status = error->x_status;
  y_status = error->y_status;
  mode = error->mode_data;
  phi_data = error->phi_data;

}


int main(int argc, char **argv)
{
    double a,b,c= 0.0;
    double e_a = 0.0;
    double e_b = 0.0;
    double da, db, dc = 0;
    double K_P = 0.0035;
    double K_D = 0.0004;
    double K_I = 0.0005;
    int x_data_old = 40;
    int y_data_old = 40;
    double d_error_old_x =0.0;
    double d_error_old_y =0.0;
    double d_error_old_phi =0.0;
    double phi = 0;
    double rot_alpha = 0;
    double rot_beta_x, rot_beta_y, rot_beta_z = 0;
    double q_x,q_y,q_z,q_w = 0;
    const double pi = 3.14159265359;
    int mode = 0;
    int initial = 0;
    int iter_guidance = 0;
    int iter_descend = 0;
    int ex_guidance = 0;
    int ey_guidance = 0;
    int r_pixel = 0;
    float a_save, b_save = 0;
    double k = 0;

    ros::init(argc, argv, "flight_control_node");
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
    pose.pose.position.z = 0.1;

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

                phi = 0;   
                c = 5;
                iter_guidance = 0;
                iter_descend = 0;
                rot_beta_x = pi/2;
                rot_beta_y = pi/2;
                rot_beta_z = 0; 

                q_x = sin(phi/2.0)*cos(rot_beta_x);     
                q_y = sin(phi/2.0)*cos(rot_beta_y);
                q_z = sin(phi/2.0)*cos(rot_beta_z);
                q_w = cos(phi/2.0);
 
                pose.pose.position.x = 0;            
                pose.pose.position.y = 0;
                pose.pose.position.z = 5;
            
                pose.pose.orientation.x = q_x;
                pose.pose.orientation.y = q_y;
                pose.pose.orientation.z = q_z;
                pose.pose.orientation.w = q_w;
              }
 
            //main flight control algorithm
           
            a = a + da + e_a;
            b = b + db + e_b;
            c = c + dc;   

            pose.pose.position.x = a;
            pose.pose.position.y = b;
            pose.pose.position.z = c;         

            initial = 1;
            // mode1 -> Mission Mode           
            if (error_vars.mode == 1){            
               
              da = 0.03;
              db = 0; 

            }
            // mode2
            else if(error_vars.mode == 2){

              iter_guidance = iter_guidance + 1;
              
              if(iter_guidance < 20){

                 da = 0;
                 db = 0;

                 x_data_old = 0;
                 y_data_old = 0;

                 d_error_old_x = 0;
                 d_error_old_y = 0;  

               }
              if(iter_guidance == 20){

                 ex_guidance = error_vars.x_data;
                 ey_guidance = error_vars.y_data;
                 r_pixel = error_vars.r_data;
                 a_save = a;
                 b_save = b;

               }
              if(iter_guidance >=50 && iter_guidance <= 300){   //Guidance Mode -> cos trajectory

                 a = (0.25/r_pixel) * ex_guidance * 0.5 * (1 - cos(pi * (iter_guidance-50)/250)) + a_save;
                 b = (0.25/r_pixel) * ey_guidance * 0.5 * (1 - cos(pi * (iter_guidance-50)/250)) + b_save;

               }
              else if(iter_guidance > 300){   //PID Mode
                 
                 da = 0;  
                 db = 0; 

                 e_a = 0.01*(error_vars.x_data*K_P + (error_vars.x_data - x_data_old) * K_D/0.02 + K_I*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x));
                 e_b = 0.01*(error_vars.y_data*K_P + (error_vars.y_data - y_data_old) * K_D/0.02 + K_I*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y));

                 x_data_old = error_vars.x_data;
                 y_data_old = error_vars.y_data;

                 d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
                 d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;          

               }                         
            }
            else if(error_vars.mode == 3){

              iter_descend = iter_descend + 1;

              if(iter_descend < 10){
                                 
                da = 0;
                db = 0;
                dc = 0;
                r_pixel = error_vars.r_data;
    
                x_data_old = 0;
                y_data_old = 0;

                d_error_old_x = 0;
                d_error_old_y = 0;    
              
              }
              if(iter_descend >= 10){ //PID + descending
                    
                 k = ((240*0.25)/(tan(24.4*(pi/180))*r_pixel*5)); // gain scheduling
                 e_a = k*0.01*(error_vars.x_data*K_P + (error_vars.x_data - x_data_old) * K_D/0.02 + K_I*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x));
                 e_b = k*0.01*(error_vars.y_data*K_P + (error_vars.y_data - y_data_old) * K_D/0.02 + K_I*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y));

                 x_data_old = error_vars.x_data;
                 y_data_old = error_vars.y_data;

                 d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
                 d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y; 

                 dc = -0.004;      

               }
            }
            else if(error_vars.mode == 4){

               da = 0; 
               db = 0;
               e_a = 0;
               e_b = 0;
               dc = -0.002;

            }
            else if(error_vars.mode == -2){    // Update Current Position and HOLD

               da = 0; 
               db = 0;
               e_a = 0;
               e_b = 0;

            }     
            else{ 
                    
            }
  
       
        ROS_INFO("mode: %d, a: %.2lf, b: %.2lf , c: %.2lf, da: %.2lf, db: %.2lf, dc: %.2lf , iter_guide : %d, e_a: %.2lf, e_b: %.2lf ",error_vars.mode, a, b, c, da, db, dc, iter_guidance, e_a, e_b);
        
 
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
