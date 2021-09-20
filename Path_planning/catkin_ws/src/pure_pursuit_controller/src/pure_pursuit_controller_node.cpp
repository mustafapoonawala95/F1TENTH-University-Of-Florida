/* 
@file: pure_pursuit_controller_node.cpp
@author: Mustafa Poonawala

Pure pursuit controller node, subscribes to path planner topic loaded in the parameter server to get waypoints.

*/
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <ackermann_msgs/AckermannDriveStamped.h>

// A couple of global variables.
tf2_ros::Buffer tf_buffer;

class pure_pursuit_controller {

    private:

    ros::NodeHandle node_handle_;
    ros::Subscriber path_subscriber;
    ros::Subscriber car_pose_subscriber;
    ros::Publisher drive_command_publisher;
    nav_msgs::Path path_to_track_map_frame_;
    std::string planner_topic;
    float lookahead_distance_;
    int path_seq_;
    int path_size_;
    int current_best_waypoint_seq;
    float max_steering_angle_ = 0.4189;   
    float mid_speed_ = 3;        
    float slowest_speed_ = 2;
    float max_speed_ = 5;    // Speed 5 is optimum.
    bool path_detected;
    std::pair<float,float> goal_point_;
    float goal_threshold_distance_;
    
    public:
    
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

    float dist(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2){
        float dx = pose2.pose.position.x - pose1.pose.position.x;
        float dy = pose2.pose.position.y - pose1.pose.position.y;
        return sqrt((dx*dx) + (dy*dy));
    }

    void stop_car(){
        ackermann_msgs::AckermannDriveStamped drive_command;
        drive_command.header.stamp = ros::Time::now();
        drive_command.header.frame_id = "base_link";
        drive_command.drive.steering_angle = 0.0;
        drive_command.drive.speed = 0.0;
        drive_command_publisher.publish(drive_command);
    }

    pure_pursuit_controller(){
        node_handle_ = ros::NodeHandle();
        node_handle_.getParam("planner_topic",planner_topic);   // Getting planner topic from parameter server.
        node_handle_.getParam("lookahead_distance_",lookahead_distance_);
        path_subscriber = node_handle_.subscribe(planner_topic, 1, &pure_pursuit_controller::path_subscriber_callback,this);
        car_pose_subscriber = node_handle_.subscribe("/gt_pose", 100, &pure_pursuit_controller::car_pose_callback,this);
        drive_command_publisher = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
        // = 0.9;   // 0.9 is optimum for max speed 5.
        path_seq_ = 0;
        ros::Duration(1.0).sleep();// Waiting for 1sec for the tf buffer to fill up so that transforms can be found by lookupTransform().
        current_best_waypoint_seq = 0;
        path_detected = false;
        goal_threshold_distance_ = 0.5;
    }

    void path_subscriber_callback(const nav_msgs::Path::ConstPtr& planner_path){
        //Fetching the path published by planner and saving as class attribute.
        path_size_ = planner_path->poses.size();
        std::cout << "path size from " << planner_topic << "is: " << path_size_ <<"\n";
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
        float point_spacing = 0.1;
        //std::cout << "First element added to temp poses.\n"; 
        for(int i=1;i<path_size_;i++){
            while(dist(dense_temp_poses.back(),temp_poses[i])>=point_spacing){ // point_spacing was lookahead_distance_ earlier.
                float DY = temp_poses[i].pose.position.y - dense_temp_poses.back().pose.position.y;
                float DX = temp_poses[i].pose.position.x - dense_temp_poses.back().pose.position.x; 
                float slope = DY/DX;
                //float t = lookahead_distance_/(sqrt((DX*DX) + (DY*DY)));
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
        //std::cout << "Added final goal point to dense_temp_poses succesfully.\n";
        path_to_track_map_frame_.header.frame_id = "map";
        path_to_track_map_frame_.poses = dense_temp_poses;
        goal_point_.first = planner_path->poses[(path_size_-1)].pose.position.x;
        goal_point_.second = planner_path->poses[(path_size_-1)].pose.position.y;
        path_size_ = path_to_track_map_frame_.poses.size();
        std::cout << "Dense path size is: " << path_size_ << "\n";
        //std::cout << "Dense path_to_track_map_frame created succesfully.\n";
        path_detected = true;
    }

    void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& car_pose){
        // Pure pursuit algorithm implemented here.
        if(path_detected){
            float car_x = car_pose->pose.position.x;
            float car_y = car_pose->pose.position.y;
            float current_best_waypoint_dist = 99999.00;
            float curr_best_y_ = 99999.0;
            geometry_msgs::TransformStamped map_to_car_frame;
            while(!tf_buffer.canTransform("base_link", "map", ros::Time()));
            try{
                map_to_car_frame = tf_buffer.lookupTransform("base_link", "map", ros::Time(0)); // Getting latest transform.
                //std::cout << "Current transform is: " << map_to_car_frame.transform.translation.x << ", " <<
                //map_to_car_frame.transform.translation.y << "\n";
            }
            catch (tf2::TransformException &ex) {
                std::cout << "blah blah\n";
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            // Going over all path points in the path, coverting them to car frame and finding the best one to track at the moment.
            for(int i=0;i<path_size_; i++){
                geometry_msgs::PointStamped current_point_map_frame;
                geometry_msgs::PointStamped current_point_car_frame;
                current_point_map_frame.header.frame_id = "map";
                current_point_map_frame.point.x = path_to_track_map_frame_.poses[i].pose.position.x;
                current_point_map_frame.point.y = path_to_track_map_frame_.poses[i].pose.position.y;
                current_point_map_frame.point.z = path_to_track_map_frame_.poses[i].pose.position.z;
                tf2::doTransform(current_point_map_frame, current_point_car_frame, map_to_car_frame);
                float curr_point_x = current_point_car_frame.point.x;
                float curr_point_y = current_point_car_frame.point.y; 
                float curr_dist = sqrt(curr_point_x*curr_point_x + curr_point_y*curr_point_y);

                // Considering points only that are in the future path to avoid going in the reverse direction.
                if(i>=path_seq_){ 
                    if(abs(curr_dist-lookahead_distance_) < abs(current_best_waypoint_dist-lookahead_distance_)){
                        current_best_waypoint_dist = curr_dist;
                        current_best_waypoint_seq = i;
                        curr_best_y_ = curr_point_y;
                    }
                }

            }
            /*std::cout << "Current best waypoint distance is: " << current_best_waypoint_dist <<"\n";
            std::cout << "Path seq. chosen to follow is: " << current_best_waypoint_seq << "\n";
            std::cout << "the current best y is: " << curr_best_y_ << "\n";*/

            path_seq_ = current_best_waypoint_seq;    // keeping track of the seq of last waypoint chosen to follow.
            ackermann_msgs::AckermannDriveStamped drive_command;

            if(goal_closeness(std::make_pair(car_x,car_y)) == 1){

                stop_car();
                // Once goal reached resetting class members so that next path can be follwed correctly.
                path_seq_ = 0;
                current_best_waypoint_seq = 0;
                current_best_waypoint_dist = 99999.00;
                //curr_best_dist_within_lookahead_radius = 0;
                //curr_best_dist_beyond_lookahead_radius = 999999.00;
                path_detected = false;
            }

            else{
                float steering_angle = 2*(curr_best_y_)/(current_best_waypoint_dist*current_best_waypoint_dist);
                if(steering_angle > max_steering_angle_){
                    steering_angle = max_steering_angle_;
                    drive_command.drive.speed = slowest_speed_;
                }
                else if(steering_angle < -max_steering_angle_){
                    steering_angle = -max_steering_angle_;
                    drive_command.drive.speed = slowest_speed_;
                }
                else if(abs(steering_angle)>= 0.15){
                    drive_command.drive.speed = mid_speed_;
                    if(goal_closeness(std::make_pair(car_x,car_y)) == 2){
                        drive_command.drive.speed = slowest_speed_;
                    }
                }
                else {
                    drive_command.drive.speed = max_speed_;
                    if(goal_closeness(std::make_pair(car_x,car_y)) == 2){
                        drive_command.drive.speed = slowest_speed_;
                    }
                }
                drive_command.header.stamp = ros::Time::now();
                drive_command.header.frame_id = "base_link";
                drive_command.drive.steering_angle = steering_angle;
                //std::cout << "The speed is commanded is:" << drive_command.drive.speed << "\n";
                drive_command_publisher.publish(drive_command);
                //std::cout << "Steering angle commanded is: " << steering_angle << "\n";
            }
        } // if(path_detected) block ends here. 
        else{
            //std::cout << "No path detected to track, so bye!\n";
        }          
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "pure_pursuit_controller_node");
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    //tf2_ros::Buffer A_buffer;
    pure_pursuit_controller pp_controller;
    ros::spin();
    return 0;
}