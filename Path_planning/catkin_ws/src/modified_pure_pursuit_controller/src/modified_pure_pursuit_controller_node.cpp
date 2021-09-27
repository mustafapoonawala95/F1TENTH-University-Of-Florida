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
#define PI 3.14159265
// A couple of global variables.
tf2_ros::Buffer tf_buffer;

class pure_pursuit_controller {

    private:

    ros::NodeHandle node_handle_;
    ros::Subscriber path_subscriber;
    ros::Subscriber car_pose_subscriber;
    ros::Publisher drive_command_publisher;
    ros::Publisher dense_path_publisher;
    nav_msgs::Path path_to_track_map_frame_;
    std::string planner_topic;
    float lookahead_distance_;
    int path_seq_;
    int path_size_;
    int current_best_waypoint_seq;
    float max_steering_angle_ = 0.4189;   
    float mid_speed_;        
    float slowest_speed_;
    float max_speed_;    // Speed 5 is optimum.
    bool path_detected;
    std::pair<float,float> goal_point_;
    float goal_threshold_distance_;
    int window_size_;
    double beta_max_;
    float alpha_max_;
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

    int get_angle_quadrant(double angle){
        int angle_quadrant;
        if(angle >= 0.0){
            if(angle <=90.0){
                angle_quadrant = 1;
            }
            else{
                angle_quadrant = 2;
            }
        }
        else if(angle < -90.0){
                angle_quadrant = 3;
            }
        else{
            angle_quadrant = 4;
        }
        return angle_quadrant;
    }

    double difference_of_angles(int i){
        double x1 = path_to_track_map_frame_.poses[i].pose.position.x;
        double x2 = path_to_track_map_frame_.poses[i+1].pose.position.x;
        double x3 = path_to_track_map_frame_.poses[i+2].pose.position.x;
        double y1 = path_to_track_map_frame_.poses[i].pose.position.y;
        double y2 = path_to_track_map_frame_.poses[i+1].pose.position.y;
        double y3 = path_to_track_map_frame_.poses[i+2].pose.position.y;
        double angle1 = (atan((y2-y1)/(x2-x1)))*(180/PI);
        double angle2 = (atan((y3-y2)/(x3-x2)))*(180/PI);
        int angle1_quadrant, angle2_quadrant, case_identifier;
        angle1_quadrant = get_angle_quadrant(angle1);
        angle2_quadrant = get_angle_quadrant(angle2);
        case_identifier = angle1_quadrant*10 + angle2_quadrant;
        std::vector<int> case_1{11,21,12,22,33,44,43,34};
        std::vector<int> case_2{14,41};
        std::vector<int> case_3{23,32,13,31,24,42};
        if(std::find(case_1.begin(), case_1.end(), case_identifier) != case_1.end()){
            return abs(angle1 - angle2);
        }
        else if(std::find(case_2.begin(), case_2.end(), case_identifier) != case_2.end()){
            return (abs(angle1) + abs(angle2));
        }
        else{
            if(angle1<0.0){
                return (180.0 - abs(180 + angle1 - angle2));
            }
            else {
                return (180.0 - abs(180 + angle2 - angle1));
            }
        }
    }

    double get_avg_angle_diff(int current_path_seq){
        double angle_diff = 0.0;
        for(int l=0;l<(window_size_-1);l++){
            angle_diff += difference_of_angles(current_path_seq+l);
        }
        angle_diff = (angle_diff/(window_size_-1));
        if(isnan(angle_diff)){
            return 0.00;
        }
        return angle_diff;
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
        node_handle_.getParam("window_size_",window_size_);
        node_handle_.getParam("beta_max_",beta_max_);
        node_handle_.getParam("alpha_max_",alpha_max_);
        node_handle_.getParam("mid_speed_",mid_speed_);
        node_handle_.getParam("slowest_speed_",slowest_speed_);
        node_handle_.getParam("max_speed_",max_speed_);
        path_subscriber = node_handle_.subscribe(planner_topic, 1, &pure_pursuit_controller::path_subscriber_callback,this);
        car_pose_subscriber = node_handle_.subscribe("/gt_pose", 100, &pure_pursuit_controller::car_pose_callback,this);
        drive_command_publisher = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
        dense_path_publisher = node_handle_.advertise<nav_msgs::Path>("/dense_path", 1);
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
            while(dist(dense_temp_poses.back(),temp_poses[i])>point_spacing){ // point_spacing was lookahead_distance_ earlier.
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
        dense_path_publisher.publish(path_to_track_map_frame_);
        path_detected = true;
    }

    void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& car_pose){
     // Pure pursuit algorithm implemented here.
        if(path_detected){
            double car_x = car_pose->pose.position.x;
            double car_y = car_pose->pose.position.y;
            double target_point_x = 0.0;
            double target_point_y = 0.0; 
            double target_point_dist = 0.0;
            //float current_best_waypoint_dist = 99999.00;
            //float curr_best_y_ = 99999.0;
            geometry_msgs::TransformStamped map_to_car_frame;
            while(!tf_buffer.canTransform("base_link", "map", ros::Time()));
            try{
                map_to_car_frame = tf_buffer.lookupTransform("base_link", "map", ros::Time(0)); // Getting latest transform.
                //std::cout << "Current transform is: " << map_to_car_frame.transform.translation.x << ", " <<
                //map_to_car_frame.transform.translation.y << "\n";
            }
            catch (tf2::TransformException &ex) {
                //std::cout << "Cannot find transform between baselink and map.\n";
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            // Going over all path points in the path beyond the last path_seq_
            bool intersection_found = false;
            geometry_msgs::PointStamped target_point_map_frame;
            geometry_msgs::PointStamped target_point_car_frame;
            // Finding the closest point to the car on the path.
            double lowest_dist_from_car = 9999.0;
            int index_of_closest_point;
            for (int p=0;p<path_size_;p++){
                if(dist(path_to_track_map_frame_.poses[p],*car_pose) < lowest_dist_from_car){
                    lowest_dist_from_car = dist(path_to_track_map_frame_.poses[p],*car_pose);
                    index_of_closest_point = p;
                }
            }
            double average_diff_of_angles = get_avg_angle_diff(index_of_closest_point);
            //std::cout << "The average difference of angles in degrees is: " << average_diff_of_angles << "\n";
            for(int i=path_seq_;i<(path_size_-1); i++){
                double t1;
                double t2;
                double ex = path_to_track_map_frame_.poses[i].pose.position.x;
                double ey = path_to_track_map_frame_.poses[i].pose.position.y;
                double lx = path_to_track_map_frame_.poses[i+1].pose.position.x;
                double ly = path_to_track_map_frame_.poses[i+1].pose.position.y;
                double dx = lx - ex;
                double dy = ly - ey;
                double fx = ex - car_x;
                double fy = ey - car_y;
                double a = dx*dx + dy*dy;
                double b = 2*(fx*dx + fy*dy);
                double c = (fx*fx + fy*fy) - (lookahead_distance_*lookahead_distance_);
                double discriminant = (b*b) - (4*a*c);
                if(discriminant < 0){// Just chill
                }
                else{
                    discriminant = sqrt(discriminant);
                    t1 = (-b - discriminant)/(2*a);
                    t2 = (-b + discriminant)/(2*a);
                    if(t1 >= 0 && t1 <=1){
                        target_point_map_frame.header.frame_id = "map";
                        target_point_map_frame.point.x = ex + t1*dx;
                        target_point_map_frame.point.y = ey + t1*dy;
                        target_point_map_frame.point.z = 0.00;
                        tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);
                        path_seq_ = i;
                        intersection_found = true;
                        break;
                    }
                    if(t2 >= 0 && t2 <=1){
                        target_point_map_frame.header.frame_id = "map";
                        target_point_map_frame.point.x = ex + t2*dx;
                        target_point_map_frame.point.y = ey + t2*dy;
                        target_point_map_frame.point.z = 0.00;
                        tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);
                        path_seq_ = i;
                        intersection_found = true;
                        break;
                    }
                }
            }
            if(intersection_found){
                target_point_x = target_point_car_frame.point.x;
                target_point_y = target_point_car_frame.point.y; 
                target_point_dist = sqrt(target_point_x*target_point_x + target_point_y*target_point_y);
                //std::cout << "Intersection with path found and distance from car is: " << target_point_dist << "\n";
            }
            else{
                std::cout << "No intersection between lookahead radius cirlcle and path found.\n";
                // Since no point of intersection found. Finding the closest point on the path and tracking that.
                double current_best_waypoint_dist = 999999.00;
                double current_best_waypoint_seq;
                for(int j = path_seq_; j<path_size_; j++){
                    double curr_x = path_to_track_map_frame_.poses[j].pose.position.x;
                    double curr_y = path_to_track_map_frame_.poses[j].pose.position.y;
                    //double curr_z = path_to_track_map_frame_.poses[j].pose.position.z;
                    double delta_x = curr_x - car_x;
                    double delta_y = curr_y - car_y;
                    double current_waypoint_dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
                    if(current_waypoint_dist < current_best_waypoint_dist){
                        current_best_waypoint_dist = current_waypoint_dist;
                        current_best_waypoint_seq = j;
                    }
                }
                target_point_map_frame.header.frame_id = "map";
                target_point_map_frame.point.x = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.x;
                target_point_map_frame.point.y = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.y;
                target_point_map_frame.point.z = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.z;
                tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);

                target_point_x = target_point_car_frame.point.x;
                target_point_y = target_point_car_frame.point.y; 
                target_point_dist = sqrt(target_point_x*target_point_x + target_point_y*target_point_y);
                path_seq_ = current_best_waypoint_seq;
            }   
            
            ackermann_msgs::AckermannDriveStamped drive_command;

            if(goal_closeness(std::make_pair(car_x,car_y)) == 1){
                stop_car();
                // Once goal reached resetting class members so that next path can be follwed correctly.
                path_seq_ = 0;
                path_detected = false;
                target_point_x = 0.0;
                target_point_y = 0.0; 
                target_point_dist = 0.0;
            }
            else{
                //std::cout << "The lookahead distance used is: " << target_point_dist << "\n";
                double steering_angle_og = 2*(target_point_y)/(target_point_dist*target_point_dist);
                double cor = (average_diff_of_angles/beta_max_)*steering_angle_og;
                cor = std::min(abs(cor),abs(steering_angle_og));
                float alpha = lowest_dist_from_car/alpha_max_;
                double steering_angle = steering_angle_og - (1-alpha)*cor*(steering_angle_og/abs(steering_angle_og));
                std::cout << "angle_diff: " << average_diff_of_angles << ", og_steer" << (180.0/PI)*steering_angle_og
                << ", correct_deg: " << (180.0/PI)*cor << ", new_steer: " <<  (180.0/PI)*steering_angle <<"\n";
            
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
            //std::cout << "No path detected to track.\n";
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