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

class frontd
{
  public:
  int mode4 = 0;

  void front_Callback(const ai_drone::error_data::ConstPtr& frontd);
};

void frontd::front_Callback(const ai_drone::error_data::ConstPtr& frontd){

  mode4 = frontd->mode_data;

}

int main(int argc, char **argv)
{
    double a,b=0.0;
    double e_a =0.0;
    double e_b =0.0;
    double da, db =0;
    double e_theta =0;
    double K_P = 0.005;
    double K_D = 0.0002;
    double K_I = 0.017;
    int x_data_old = 40;
    int y_data_old = 40;
    int theta_data_old =0;
    double d_error_old_x =0.0;
    double d_error_old_y =0.0;
    double d_error_old_theta =0.0;
    int iter = 0;
    int iter1 = 0;
    int iter2 = 0;
    int iter3 = 0;
    int iter4 = 0;
    int iter_f = 0;
    int iter_the = 0;
    double theta = 0;
    double phi = 0;
    double rot_alpha = 0;
    double rot_beta_x, rot_beta_y, rot_beta_z = 0;
    double q_x,q_y,q_z,q_w = 0;
    double theta_add =0;
    double dist = 0;
    double dist3 = 0;
    const double pi = 3.14159265359;
    int mode = 0;
    int c,d,e,f = 0;

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
    frontd frontm;
    ros::Subscriber front_data_sub = nh.subscribe("front_data_msg",30,&frontd::front_Callback, &frontm);

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

            //pi update
            if(theta >= 2*pi){
                theta = theta - 2*pi;
              }
           
            if(iter_the == 4){
                d = 1;
              } 
           
            // landing
            if(f == 1){ 
              break;
             }

            // mode1 (spawn -> start) // landing            
            if (error_vars.mode == 1){            
              initial = 1;
              iter = 0;
              iter1 = 0;
              dist = 0;

              if (d ==1){ //landing
                 e_a = error_vars.x_data*0.0005 + (error_vars.x_data - x_data_old) * 0.0003/0.02 + 0.005*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x);
                 e_b = error_vars.y_data*0.0005 + (error_vars.y_data - y_data_old) * 0.0003/0.02 + 0.005*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y);
                 pose.pose.position.z = 1.5;
              }else{ // start
                 e_a = error_vars.x_data*K_P + (error_vars.x_data - x_data_old) * K_D/0.02 + K_I*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x);
                 e_b = error_vars.y_data*K_P + (error_vars.y_data - y_data_old) * K_D/0.02 + K_I*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y);  
              }

              if(c == 1){
                e_a = 0;
                e_b = 0;
              }
            
              da = dist * cos(theta);
              db = dist * sin(theta); 

              a = a + da;
              b = b + db; 
 
              pose.pose.position.x = a + cos(theta)*e_a - sin(theta)*e_b;
              pose.pose.position.y = b + sin(theta)*e_a + cos(theta)*e_b;

              x_data_old = error_vars.x_data;
              y_data_old = error_vars.y_data;

              d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
              d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;

            }
            // mode2 (start -> line tracking)
            else if(error_vars.mode == 2){
              initial = 1;
              iter = 0;
              iter1 = 0;
            
              iter2 = iter2 + 1;

                if(frontm.mode4 == 4){ // turbulance reduce
                  e_a = 0.4 * (error_vars.x_data*0.0015 + (error_vars.x_data - x_data_old) * 0.00025/0.02 + 0.004*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x));
                  e_b = 0.4 * (error_vars.y_data*0.0015 + (error_vars.y_data - y_data_old) * 0.00025/0.02 + 0.004*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y));
                }
                else{
                  e_a = error_vars.x_data*0.0015 + (error_vars.x_data - x_data_old) * 0.00025/0.02 + 0.004*((error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x);
                  e_b = error_vars.y_data*0.0015 + (error_vars.y_data - y_data_old) * 0.00025/0.02 + 0.004*((error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y);
                }

              e_theta = -(error_vars.theta_data*0.2 + (error_vars.theta_data - theta_data_old) * 0.01/0.02 + 0.17*((error_vars.theta_data + theta_data_old)*0.02*0.5 + d_error_old_theta));

              da = dist * cos(theta);
              db = dist * sin(theta); 

              a = a + da;
              b = b + db; 

              pose.pose.position.x = a + cos(theta)*e_a - sin(theta)*e_b;
              pose.pose.position.y = b + sin(theta)*e_a + cos(theta)*e_b;

              q_x = sin((theta+e_theta)/2.0)*cos(rot_beta_x);     
              q_y = sin((theta+e_theta)/2.0)*cos(rot_beta_y);
              q_z = sin((theta+e_theta)/2.0)*cos(rot_beta_z);
              q_w = cos((theta+e_theta)/2.0);

              pose.pose.orientation.x = q_x;
              pose.pose.orientation.y = q_y;
              pose.pose.orientation.z = q_z;
              pose.pose.orientation.w = q_w;

              x_data_old = error_vars.x_data;
              y_data_old = error_vars.y_data;
              d_error_old_x = (error_vars.x_data + x_data_old)*0.02*0.5 + d_error_old_x;
              d_error_old_y = (error_vars.y_data + y_data_old)*0.02*0.5 + d_error_old_y;

              theta_data_old = error_vars.theta_data;
              d_error_old_theta = (error_vars.theta_data + theta_data_old)*0.02*0.5 + d_error_old_theta;

             if(iter2 >= 5){
                dist = 0.015;
              }

                if(frontm.mode4 == 4){ // meet obstacle -> go up to 2m and go down
                    iter4 = iter4 + 1;
 
                   if(iter4 >= 1 && iter4 < 15){
                       dist = 0;
                     }
                   else if(iter4 >= 15){
                       pose.pose.position.z = 2.5; 
                     }
                   if(iter4 >= 20){
                       dist = 0.02;
                     }
                   if(iter4 >= 150){
                       pose.pose.position.z = 0.9; 
                     }
               }
               else{
                  pose.pose.position.z = 0.9;
               } 
            }
            else if(error_vars.mode == 3){
             c = 0;
             if(frontm.mode4 == 4){
           
                da = dist * cos(theta);
                db = dist * sin(theta); 

                a = a + da;
                b = b + db; 

                pose.pose.position.x = a;
                pose.pose.position.y = b;  

                dist = 0.015;
                }
             else{
               initial = 1;
               iter2 = 0;
               iter3 = 0;
               iter4 = 0;

               iter = iter + 1;

               da = da + dist * -sin(theta) + dist3 * cos(theta);
               db = db + dist * cos(theta) + dist3 * sin(theta); 
 
               if (iter == 1){
                 a = a + da;
                 b = b + db;
                 e_a = 0;
                 e_b = 0;
                 da = 0;
                 db = 0;
                 dist = 0;
                }
                if (iter >= 1 && iter <= 17 ){
                 dist3 = 0.01;
                }else{
                 dist3 = 0;
                }

                pose.pose.position.x = a + da;
                pose.pose.position.y = b + db;

                q_x = sin(theta/2.0)*cos(rot_beta_x);     
                q_y = sin(theta/2.0)*cos(rot_beta_y);
                q_z = sin(theta/2.0)*cos(rot_beta_z);
                q_w = cos(theta/2.0);

                pose.pose.orientation.x = q_x;
                pose.pose.orientation.y = q_y;
                pose.pose.orientation.z = q_z;
                pose.pose.orientation.w = q_w;

              if (iter >= 3){
                 dist = 0.005;
                 }
               }
             }
            else if(error_vars.mode == -1){ // error tack end -> update position, make error parameter 0 to use again
            initial = 1;
            iter = 0;

            iter1 = iter1 + 1;
            c = 1;

              if(iter1 == 1){
                    a = a + cos(theta)*e_a - sin(theta)*e_b;
                    b = b + sin(theta)*e_a + cos(theta)*e_b;
                 }
                if(d == 1){ //landing
 
                 pose.pose.position.z = 0.1;
                 e = 1;
                }
                else{
                 pose.pose.position.z = 0.9;
                } 
             }
            else if(error_vars.mode == -2){ // error tack end -> update position, make error parameter 0 to use again
              initial = 1;
 
              iter = iter + 1;

              if (iter == 1){
               e_a = 0;
               e_b = 0;
               dist = 0;
              }
              if(iter >= 20){

               da = dist * cos(theta);
               db = dist * sin(theta);

               a = a + da;
               b = b + db;

               pose.pose.position.x = a;
               pose.pose.position.y = b;
 
               pose.pose.orientation.x = q_x;
               pose.pose.orientation.y = q_y;
               pose.pose.orientation.z = q_z;
               pose.pose.orientation.w = q_w;

             }
 
             if(iter >= 30){
                if(d == 1){
                  dist = 0.005;
                 }
                else{
                  dist = 0.015;
                 }
               }
             if( e == 1 ){
                pose.pose.position.z = 0.1;
                dist = 0;

                 iter_f = iter_f + 1;

                  if(iter_f >= 40){
                    f = 1;
                  }
               }
             else if(iter >= 50 && d == 1 && e != 1){
                pose.pose.position.z = 0.9 + (iter-50) * 0.005 ;
               }                          
            } 
            else if(error_vars.mode == -3){
            initial = 1;

            iter3 = iter3 + 1;

             da = da + dist * cos(theta);
             db = db + dist * sin(theta);
 
             if (iter3 == 1){
               a = a + da;
               b = b + db;
               e_a = 0;
               e_b = 0;
               da = 0;
               db = 0;
              }

              theta_add = 0;  

              pose.pose.position.x = a + da;
              pose.pose.position.y = b + db;

              q_x = sin(theta/2.0)*cos(rot_beta_x);     
              q_y = sin(theta/2.0)*cos(rot_beta_y);
              q_z = sin(theta/2.0)*cos(rot_beta_z);
              q_w = cos(theta/2.0);

              pose.pose.orientation.x = q_x;
              pose.pose.orientation.y = q_y;
              pose.pose.orientation.z = q_z;
              pose.pose.orientation.w = q_w;

              if (iter3 == 2){
                theta_add =  pi/2;
                theta = theta + theta_add;
                iter_the = iter_the + 1;
              }
              else if (iter3 >= 40){
                dist = 0.005;  
              }

            }  
            else{ 
                    
            }
  

       if(frontm.mode4 == 4){
          mode = frontm.mode4;
       }else{
          mode = error_vars.mode;
       }

        ROS_INFO("mode: %d, theta: %.4f, a: %.2lf, b: %.2lf, da: %.2lf, db: %.2lf, e_theta: %.4lf, e_a: %.2lf, e_b: %.2lf ",mode, theta, a, b, da, db, e_theta, e_a, e_b);
 
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
