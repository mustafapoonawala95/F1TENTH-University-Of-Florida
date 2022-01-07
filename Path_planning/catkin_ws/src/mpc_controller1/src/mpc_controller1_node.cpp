/* 
@file: mpc_controller1_node.cpp
@author: Mustafa Poonawala

MPC controller node, subscribes to a path topic to get reference path, /gt_pose for car's position and 
publishes the steering commands to /drive topic.
*/

/*Things TODO:
1) Add input rate constraints
2) Add provision to have different Np and Nc
3) Check the performance of different solvers. IPOPT V/S qpoasis...   
4) Add provision to warm start using decision variable values from previous iteration
5) Change the logic of generating x_ref and y_ref when distance to goal is less than prediction horizon. Refer for loop near line 297*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include<casadi/casadi.hpp>                                                    
#include<iostream>                                                              
#include <core/optistack.hpp>    
#include<vector>                                                       
using namespace casadi;                                                        
#define PI  3.14159265                                                                     


class mpc_controller1{
    private:
    ros::NodeHandle node_handle_;
    ros::Subscriber car_pose_subscriber_;
    ros::Subscriber path_subscriber_; 
    ros::Publisher drive_command_publisher_;
    nav_msgs::Path path_to_track_map_frame_;

    Opti MPCopti;
    MX x_over_Np;
    MX y_over_Np; 
    MX psi_over_Np;
    MX u_over_Nc;
    MX A1;
    MX A2;
    MX A;
    MX B;
    MX G;
    MX x_ref;
    MX y_ref;
    MX car_curr_x;
    MX car_curr_y;
    MX car_curr_psi;
    MX prev_steer_;
    std::vector<double> x_ref_temp;
    std::vector<double> y_ref_temp;


    int path_seq_;
    int path_size_;
    bool path_detected_;
    bool goal_reached_;
    std::pair<float,float> goal_point_;
    float max_steering_angle_ = 0.4189;        // Max steering angle. 21 degrees
    double speed_;                              
    double wheel_base_;
    double prev_steer_temp;

    // Tunable parameters. Tuned from launch file.
    float goal_threshold_distance_;
    int Np_;
    int Nc_;
    bool print_time_info_;
    //float error_penalty_;
    float control_over_error_penalty_;        
    float sampling_frequency_;                 // This allows us to make the controller slower so that we can mimic actual car's behaviour.
    float Ts_ = 1/sampling_frequency_;
    std::string planner_topic_;

    public:

    double Rad2Deg(double r) {return r*180/PI;}

    double Deg2Rad(double d) {return d*PI/180;}

    float dist(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2){
        float dx = pose2.pose.position.x - pose1.pose.position.x;
        float dy = pose2.pose.position.y - pose1.pose.position.y;
        return sqrt((dx*dx) + (dy*dy));
    }

    /*
    returns an integer based on the closeness to the goal.
    */
    int goal_closeness(std::pair<float,float> car_position){
        float dx = car_position.first - goal_point_.first;
        float dy = car_position.second - goal_point_.second;
        if(sqrt((dx*dx) + (dy*dy)) <= goal_threshold_distance_){
            return 1;
        }
        else if(sqrt((dx*dx) + (dy*dy)) <= 2.5){
            return 2;
        }
        else{
            return 3;
        }
    }


    void stop_car(){
        ackermann_msgs::AckermannDriveStamped drive_command;
        drive_command.header.stamp = ros::Time::now();
        drive_command.header.frame_id = "base_link";
        drive_command.drive.steering_angle = 0.0;
        drive_command.drive.speed = 0.0;
        drive_command_publisher_.publish(drive_command);
    }


    void clip_angle(float& steering_angl){
        if(steering_angl > max_steering_angle_){
            steering_angl = max_steering_angle_;
        }
        else if(steering_angl < -max_steering_angle_){
            steering_angl = -max_steering_angle_;
        }
    }

    /*
    MPC_controller class constructor.
    */
    mpc_controller1(){
        node_handle_ = ros::NodeHandle();
        // Getting parameters from parameter server. Check launch file for details.
        node_handle_.getParam("planner_topic_",planner_topic_);
        node_handle_.getParam("wheel_base_",wheel_base_);
        node_handle_.getParam("sampling_frequency_",sampling_frequency_);
        node_handle_.getParam("goal_threshold_distance_",goal_threshold_distance_);
        node_handle_.getParam("speed_",speed_);
        node_handle_.getParam("Np_",Np_);
        node_handle_.getParam("Nc_",Nc_); 
        node_handle_.getParam("control_over_error_penalty_",control_over_error_penalty_);
        node_handle_.getParam("print_time_info_",print_time_info_);
        drive_command_publisher_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
        path_subscriber_ = node_handle_.subscribe(planner_topic_, 1, &mpc_controller1::path_subscriber_callback,this);
        car_pose_subscriber_ = node_handle_.subscribe("/gt_pose", 1, &mpc_controller1::car_pose_callback,this);
        //ros::Duration(1.0).sleep();// Waiting for 1sec for the tf buffer to fill up so that transforms can be found by lookupTransform().
        goal_reached_ = false;
        path_detected_ = false;
        path_seq_ = 0;
        prev_steer_temp = 0.0;
        Ts_ = 1/sampling_frequency_;
        //prev_time_ = ros::Time::now().toSec();
        //std::cout<<"Prev time upon initialization: "<<prev_time_<<"\n";
        x_ref_temp = std::vector<double>(Np_+1);           // Vectors of zeros.
        y_ref_temp = std::vector<double>(Np_+1);
        std::cout<<"x_ref_temp, y_ref_temp"<<x_ref_temp.size()<<", "<<y_ref_temp.size()<<"\n";
        //===========================================================================
        //=========== Setting up solver structure. ==================================
        //===========================================================================
        DM control_over_error_penalty = DM(1);
        control_over_error_penalty = control_over_error_penalty_;
        MPCopti = Opti();
        Dict z;
        z["print_time"] = print_time_info_;
        //z["print_level"] = 0;
        MPCopti.solver("ipopt",z);
        x_over_Np = MX::sym("x_over_Np",Np_+1);
        y_over_Np = MX::sym("y_over_Np",Np_+1);
        psi_over_Np = MX::sym("psi_over_Np",Np_+1);
        u_over_Nc = MX::sym("u_over_Nc",Nc_+1);
        x_ref = MX::sym("x_ref",Np_+1);
        y_ref = MX::sym("y_ref",Np_+1);
        car_curr_x = MX::sym("car_curr_x",1);
        car_curr_y = MX::sym("car_curr_y",1);
        car_curr_psi = MX::sym("car_curr_psi",1);
        prev_steer_ = MX::sym("prev_steer_",1);
        G = MX::zeros(1,1);
        x_over_Np = MPCopti.variable(Np_+1);
        y_over_Np = MPCopti.variable(Np_+1);
        psi_over_Np = MPCopti.variable(Np_+1);
        u_over_Nc = MPCopti.variable(Nc_+1);
        x_ref = MPCopti.parameter(Np_+1);    // Making x_ref and y_ref as parameters that will be set at each iteration.
        y_ref = MPCopti.parameter(Np_+1);
        car_curr_x = MPCopti.parameter(1);
        car_curr_y = MPCopti.parameter(1);
        car_curr_psi = MPCopti.parameter(1);
        prev_steer_ = MPCopti.parameter(1);
        MPCopti.subject_to(x_over_Np(0) == car_curr_x);      // Using the values from sensors as the current start points
        MPCopti.subject_to(y_over_Np(0) == car_curr_y);      // Using the values from sensors as the current start points
        MPCopti.subject_to(psi_over_Np(0) == car_curr_psi);  // Using the values from sensors as the current start points
        MPCopti.subject_to(u_over_Nc(0) == prev_steer_);     // Usin the prev steer angle as constraint. 
        for(int i=0;i<(Np_);i++){
            MPCopti.subject_to(x_over_Np(i+1) == x_over_Np(i) - speed_*sin(car_curr_psi)*Ts_*psi_over_Np(i) + Ts_*speed_*(cos(car_curr_psi) + sin(car_curr_psi)*car_curr_psi));
            MPCopti.subject_to(y_over_Np(i+1) == y_over_Np(i) + speed_*cos(car_curr_psi)*Ts_*psi_over_Np(i) + Ts_*speed_*(sin(car_curr_psi) - cos(car_curr_psi)*car_curr_psi));
            MPCopti.subject_to(psi_over_Np(i+1) == psi_over_Np(i) + ((Ts_*speed_)/wheel_base_)*pow((1/cos(prev_steer_)),2)*u_over_Nc(i) + Ts_*(speed_/wheel_base_)*(tan(prev_steer_) - (pow((1/cos(prev_steer_)),2))*prev_steer_));
            MPCopti.subject_to(-21*(PI/180) <= u_over_Nc(i) <= 21*(PI/180));
        }
        MPCopti.subject_to(-21*(PI/180) <= u_over_Nc(Nc_) <= 21*(PI/180));
        A1 = sumsqr(x_over_Np);         //x[k]^2 + x[k+1]^2 + x[k+2]^2 + ....x[k+Np-1]^2
        std::cout<<"A1: size"<<A1.size()<<"\n";
        A2 = sumsqr(y_over_Np);         //y[k]^2 + y[k+1]^2 + y[k+2]^2 + ....y[k+Np-1]^2                                  
        A = A1+A2;
        std::cout<<"A:size="<<A.size()<<"\n";
        B = sumsqr(u_over_Nc);                   //u[k]^2 + u[k+1]^2 + .. u[k+Nc]^2     
        for(int i=0;i<Np_+1;i++){
            G = G - 2*x_over_Np(i)*x_ref(i) - 2*y_over_Np(i)*y_ref(i);
        }
        //std::cout<<"GSize: "<<G<<"\n";
        std::cout<<"A,B,G:"<<A.size()<<", "<<B.size()<<", "<<G.size()<<"\n";
        
        std::cout<<"G size upon creation: "<<G.size()<<"\n";
        MPCopti.minimize((A+G) + control_over_error_penalty*(B)); 
    }

    void path_subscriber_callback(const nav_msgs::Path::ConstPtr& planner_path){
        //Fetching the path published by planner and saving as class attribute.
        path_size_ = planner_path->poses.size();
        std::cout << "path size from " << planner_topic_ << "is: " << path_size_ <<"\n";
        std::vector<geometry_msgs::PoseStamped> temp_poses(path_size_);
        std::vector<geometry_msgs::PoseStamped> dense_temp_poses;
        for(int i=0; i<path_size_; i++){
            temp_poses[i].pose.position.x = planner_path->poses[i].pose.position.x;
            temp_poses[i].pose.position.y = planner_path->poses[i].pose.position.y;
            temp_poses[i].pose.position.z = planner_path->poses[i].pose.position.z;
            temp_poses[i].header.seq = i;        
        }
        //std::cout << "Adding first element to temp poses.\n";
        dense_temp_poses.push_back(temp_poses[0]);
        float point_spacing = Ts_*speed_;              // Distance between points = distance covered in Ts at the set speed  
        std::cout << "Point spacing "<<point_spacing<<"\n"; 
        for(int i=1;i<path_size_;i++){
            while(dist(dense_temp_poses.back(),temp_poses[i])>point_spacing){
                float DY = temp_poses[i].pose.position.y - dense_temp_poses.back().pose.position.y;
                float DX = temp_poses[i].pose.position.x - dense_temp_poses.back().pose.position.x; 
                float slope = DY/DX;
                float t = point_spacing/(sqrt((DX*DX) + (DY*DY)));
                geometry_msgs::PoseStamped pose_to_add;
                pose_to_add.pose.position.x = (1 - t)*dense_temp_poses.back().pose.position.x + t*temp_poses[i].pose.position.x;
                pose_to_add.pose.position.y = (1 - t)*dense_temp_poses.back().pose.position.y + t*temp_poses[i].pose.position.y;
                pose_to_add.pose.position.z = 0.0;
                pose_to_add.header.seq = dense_temp_poses.back().header.seq + 1;
                dense_temp_poses.push_back(pose_to_add);
            }
        }
        //std::cout << "Created dense_temp_poses succesfully.\n"; 
        geometry_msgs::PoseStamped final_point;
        final_point.pose.position.x = planner_path->poses.back().pose.position.x;
        final_point.pose.position.y = planner_path->poses.back().pose.position.y;
        final_point.pose.position.z = 0.0;
        final_point.header.seq = dense_temp_poses.back().header.seq + 1;
        dense_temp_poses.push_back(final_point);
        path_to_track_map_frame_.header.frame_id = "map";
        path_to_track_map_frame_.poses = dense_temp_poses;
        goal_point_.first = planner_path->poses[(path_size_-1)].pose.position.x;
        goal_point_.second = planner_path->poses[(path_size_-1)].pose.position.y;
        path_size_ = path_to_track_map_frame_.poses.size();
        std::cout << "Dense path size is: " << path_size_ << "\n";
        //std::cout << "Dense path_to_track_map_frame created succesfully.\n";
        path_detected_ = true;
    }

    void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& car_pose){
     // Pure pursuit algorithm implemented here.
        if(path_detected_){
            ros::Rate rate(sampling_frequency_);
            tf2::Quaternion my_quaternion(car_pose->pose.orientation.x,
                                    car_pose->pose.orientation.y,
                                    car_pose->pose.orientation.z,
                                    car_pose->pose.orientation.w);
            tf2::Matrix3x3 m(my_quaternion);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            double car_curr_x_temp = car_pose->pose.position.x;
            double car_curr_y_temp = car_pose->pose.position.y;
            double car_curr_psi_temp = yaw;
            double current_best_waypoint_dist = 999999.00;
            int current_best_waypoint_seq;
            double curr_x, curr_y, delta_x, delta_y, current_waypoint_dist;
            for(int j = path_seq_; j<path_size_; j++){
                curr_x = path_to_track_map_frame_.poses[j].pose.position.x;
                curr_y = path_to_track_map_frame_.poses[j].pose.position.y;
                delta_x = curr_x - car_curr_x_temp;
                delta_y = curr_y - car_curr_y_temp;
                current_waypoint_dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
                if(current_waypoint_dist < current_best_waypoint_dist){
                    current_best_waypoint_dist = current_waypoint_dist;
                    current_best_waypoint_seq = j;
                }
            }
            // Formulating the Optimization problem.
            x_ref_temp[0] = car_curr_x_temp;           // Making the first reference point same as the car's current position.
            y_ref_temp[0] = car_curr_y_temp;

            // If prediction horizon is longer than the portion of path left to track we set all remaining reference points as the goal point.                              
            // NOTE: This logic needs to be changed as it can lead to infeasable problem.
            for(int i=0;i<(Np_);i++){
                x_ref_temp[i+1] = path_to_track_map_frame_.poses[std::min((current_best_waypoint_seq+i),(path_size_-1))].pose.position.x; 
                y_ref_temp[i+1] = path_to_track_map_frame_.poses[std::min((current_best_waypoint_seq+i),(path_size_-1))].pose.position.y;
            }
            path_seq_ = current_best_waypoint_seq;   // Updating path_seq_ to prevent going in reverse.
            MPCopti.set_value(x_ref,x_ref_temp);
            MPCopti.set_value(y_ref,y_ref_temp);
            MPCopti.set_value(car_curr_x,car_curr_x_temp);
            MPCopti.set_value(car_curr_y,car_curr_y_temp);
            MPCopti.set_value(car_curr_psi,car_curr_psi_temp);
            MPCopti.set_value(prev_steer_,prev_steer_temp);

            std::cout<<"MPC problem formulate!\n";
            OptiSol sol = MPCopti.solve();
            //std::cout<<"steering angle: "<<(180/PI)*sol.value(u_over_Nc(1)).get_elements()<<"\n";
            //DM O = DM(1);
            std::vector<double> temp = sol.value(u_over_Nc(1)).get_elements();
            //std::cout<<"temp size: "<<temp.size()<<"\n";
            float steering_angle = temp.back();
            prev_steer_temp = steering_angle;
            //prev_steer_temp = 0.0; // SETING prev_steer_temp to 0 just for now. Later use the above line.
            //MPCopti.debug();
            /*for(int i=0;i<Np;i++){
                cout<<"x: "<<sol.value(x_over_Np(i))<<" y: "<<sol.value(y_over_Np(i))<<" psi: "<<(180/PI)*sol.value(psi_over_Np(i))
                <<" delta: "<<(180/PI)*sol.value(u_over_Nc(i))<<"\n";
            }*/
            ackermann_msgs::AckermannDriveStamped mpc_drive_command;

            if(goal_closeness(std::make_pair(car_curr_x_temp,car_curr_y_temp)) == 1){
                stop_car();
                // Once goal reached resetting class members so that next path can be follwed correctly.
                path_seq_ = 0;
                path_detected_ = false;
                goal_reached_ = true;
            }
            else{
                mpc_drive_command.drive.steering_angle = steering_angle;
                mpc_drive_command.drive.speed = speed_;
                drive_command_publisher_.publish(mpc_drive_command);
            }     
            //std::cout<<"TimeForCarPoseCallback: " << ros::Time::now().toSec()-start_time<<"\n";
            rate.sleep();    // Sleeping to maintain sampling frequency.
        } // "if(path_detected)" block ends here. 
        else{
            //std::cout << "No path detected to track.\n";
        }          
    }
};

int main(int argc, char **argv){                                                                     
    ros::init(argc, argv, "mpc_controller1_node");
    mpc_controller1 MPC;
    std::cout << "MPC node created!!\n";
    ros::spin();
    return 0;                                                              
}