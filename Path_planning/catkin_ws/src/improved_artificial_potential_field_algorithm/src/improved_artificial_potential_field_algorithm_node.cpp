/* 
@file: improved_artificial_potential_field_algorithm_node.cpp
@author: Mustafa Poonawala

ADD DESCRIPTION.

*/

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

// Some global variables.
tf2_ros::Buffer tf_buffer;
const float PI = 3.14159;

class apf_controller {
    private:

    ros::NodeHandle node_handle_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber car_pose_subscriber_;
    ros::Subscriber path_subscriber_; 
    ros::Publisher drive_command_publisher_;
    nav_msgs::Path path_to_track_map_frame_;
    int path_seq_;
    int path_size_;
    bool path_detected_;
    bool goal_reached_;
    std::pair<float,float> goal_point_;
    float max_steering_angle_ = 0.4189;        // Max steering angle.
    float mid_speed_;        
    float slowest_speed_;
    float max_speed_;    // Speed 5 is optimum.
    float scan_angle_min_;
    float scan_angle_max_;
    float scan_angle_increment_;
    float scan_range_min_;
    float scan_range_max_;
    double K_track_; 
    double wheel_base_;                          
    /* R_track_ is the radius of the trajectory which the pure pursuit algorithm will generate for tracking the 
    next lookahead point. This is done in car_pose callback function. */


    // Tunable parameters. Tuned from launch file.
    double obstacle_influence_distance_;            // Any obstacle further than this is not urgent enough to act on.  
    double field_of_view_angle_degrees_;            // Obstacles will only be considered in a window of this size in front of vehicle.
    float lookahead_distance_;  
    int sliding_window_length_;
    double R_avoid_factor_;
    double do_;
    float goal_threshold_distance_;
    std::string planner_topic_;

    //============================== velocity profile parameters.=========================
    double K_F_;
    double K_delta_;
    //====================================================================================

    public:

    double Rad2Deg(double r) {return r*180/PI;}

    double Deg2Rad(double d) {return d*PI/180;}

    std::vector<float> truncate_scan(const sensor_msgs::LaserScan::ConstPtr& original_scan){

        double half_fov = Deg2Rad(field_of_view_angle_degrees_/2);
        int fov_min_id = static_cast<int>(((-half_fov - original_scan->angle_min)/original_scan->angle_increment));
        int fov_max_id = static_cast<int>((half_fov - original_scan->angle_min)/original_scan->angle_increment);        
        return std::vector<float>(original_scan->ranges.begin() + fov_min_id,original_scan->ranges.begin() + fov_max_id);
    }

    void preprocessScan(std::vector<float>& truncated_scan_ranges){
        // clip ranges between min and max.
        for(float& r : truncated_scan_ranges) {
            if(r < scan_range_min_){
                r = scan_range_min_;
            }
            else if(r > scan_range_max_){
                r = scan_range_max_;
            }
        }
        // Impose sliding window running average on scan data
        std::vector<float> temp(truncated_scan_ranges.size(), 0.00);
        std::deque<float> running_avg;
        for(unsigned int i = 0; i < truncated_scan_ranges.size(); ++i) {
            running_avg.push_back(truncated_scan_ranges[i]);
            if(running_avg.size() > sliding_window_length_){
                running_avg.pop_front();
            }
            float sum = std::accumulate(running_avg.begin(), running_avg.end(), 0.00);
            //td::cout << "SUM=" << sum <<"\n";
            temp[i] = sum/running_avg.size();
            //std::cout << "temp:=" << temp[i] << "\n";
        }
        truncated_scan_ranges = temp;
    }

    
    double angle_from_car_heading(int idx_of_range){
        double half_fov = Deg2Rad(field_of_view_angle_degrees_/2);
        double angle = -half_fov + (idx_of_range)*(scan_angle_increment_);
        //std::cout << "angle_from func["<<idx_of_range<<"]: " << angle << "\n";
        return angle;
    }

    std::vector<double> non_holonomic_dists(std::vector<float>& truncated_scan_ranges){
        std::vector<double> non_holonomic_distances(truncated_scan_ranges.size(),0.00);
        for(int i=0;i<truncated_scan_ranges.size();i++){
            double alpha_i = abs(angle_from_car_heading(i));
            //std::cout << "alpha[" << i <<"]: " << alpha_i <<"\n";
            if(abs(alpha_i == 0.0)){
                non_holonomic_distances[i] = truncated_scan_ranges[i];
            }
            else{
                if(truncated_scan_ranges[i] > lookahead_distance_){
                    non_holonomic_distances[i] = (lookahead_distance_/(std::sin(alpha_i)))*alpha_i + (truncated_scan_ranges[i] - lookahead_distance_);
                    //std::cout << "range[i]: "<<truncated_scan_ranges[i]<<", non_holonomic_dist[" << i <<"]: " << non_holonomic_distances[i] << " diff: " << (non_holonomic_distances[i] - truncated_scan_ranges[i]) << "\n";
                }
                else{
                    non_holonomic_distances[i] = (truncated_scan_ranges[i]/(std::sin(alpha_i)))*alpha_i;
                    //std::cout << "range[i]: "<<truncated_scan_ranges[i]<<", non_holonomic_dist[" << i <<"]: " << non_holonomic_distances[i] <<"\n";
                }
            }
        }
        return non_holonomic_distances;
    }

    std::pair<double,double> resultant_repulsive_force(std::vector<double>& non_holonomic_distances){
        double fx=0.0;
        double fy = 0.0;
        for(int i=0;i<non_holonomic_distances.size();i++){
            if(non_holonomic_distances[i]<=obstacle_influence_distance_){
                double F = (1/std::pow(obstacle_influence_distance_+do_,2)) - (1/std::pow(non_holonomic_distances[i]+do_,2));
                //std::cout << "F[" << i <<"]: " << F << "\n";
                double fxi = cos(angle_from_car_heading(i))*F;
                double fyi = sin(angle_from_car_heading(i))*F;
                //std::cout << "angle: " << Rad2Deg(angle_from_car_heading(i)) << ", cos: " << cos(angle_from_car_heading(i)) << ", sin" << sin(angle_from_car_heading(i)) <<"\n";
                fx += cos(angle_from_car_heading(i))*F;
                fy += sin(angle_from_car_heading(i))*F;
                //std::cout << "fxi[" << i <<"]: " << fxi << ", fyi[" << i <<"]: " << fyi << "\n";
            }
        }
        return std::make_pair(fx,fy);
    }

    double get_R_avoid(std::pair<double,double>& force){
        double alpha_res = atan(force.second/force.first);
        //std::cout << "Alpha_res: " << Rad2Deg(alpha_res) << "\n";
        double mag_F = sqrt(std::pow(force.first,2) + std::pow(force.second,2));
        if(alpha_res == 0.0){
            double R_avoid =  1/(R_avoid_factor_*mag_F);
            //std::cout << "R_avoid: " << R_avoid << "\n";
            return std::min(R_avoid,1000000.00);
        }
        else{
            int sign = abs(alpha_res)/alpha_res;
            double R_avoid = -1*sign/(R_avoid_factor_*mag_F);
            //std::cout << "R_avoid: " << R_avoid << "\n";
            return std::min(R_avoid,1000000.00);
        }
    }
    // ================================================================================================
    // ================================= code from pp controller ======================================
    // ================================================================================================

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

    apf_controller(){
        node_handle_ = ros::NodeHandle();
        node_handle_.getParam("planner_topic_",planner_topic_);   // Getting planner topic from parameter server.
        node_handle_.getParam("obstacle_influence_distance_",obstacle_influence_distance_);
        node_handle_.getParam("field_of_view_angle_degrees_",field_of_view_angle_degrees_);
        node_handle_.getParam("lookahead_distance_",lookahead_distance_);
        node_handle_.getParam("R_avoid_factor_",R_avoid_factor_);
        node_handle_.getParam("K_F_",K_F_);
        node_handle_.getParam("K_delta_",K_delta_);
        node_handle_.getParam("wheel_base_",wheel_base_);
        node_handle_.getParam("do_",do_);
        node_handle_.getParam("sliding_window_length_",sliding_window_length_);
        node_handle_.getParam("goal_threshold_distance_",goal_threshold_distance_);
        node_handle_.getParam("slowest_speed_",slowest_speed_);
        node_handle_.getParam("mid_speed_",mid_speed_);
        node_handle_.getParam("max_speed_",max_speed_);

        scan_subscriber_ = node_handle_.subscribe("/scan", 1, &apf_controller::scan_callback,this);
        drive_command_publisher_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
        path_subscriber_ = node_handle_.subscribe(planner_topic_, 1, &apf_controller::path_subscriber_callback,this);
        car_pose_subscriber_ = node_handle_.subscribe("/gt_pose", 1, &apf_controller::car_pose_callback,this);
        ros::Duration(1.0).sleep();// Waiting for 1sec for the tf buffer to fill up so that transforms can be found by lookupTransform().
        goal_reached_ = false;
        path_detected_ = false;
        path_seq_ = 0;
        K_track_ = 0.0;
    }


    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
        //==================================================================================================
        //================================ Repulsive force and R_avoid calculation. =======================
        //==================================================================================================
        if(path_detected_){
            scan_angle_min_ = scan_msg->angle_min;
            scan_angle_max_ = scan_msg->angle_max;
            scan_angle_increment_ = scan_msg->angle_increment;
            scan_range_min_ = scan_msg->range_min;
            scan_range_max_ = scan_msg->range_max;
        
            std::vector<float> truncated_scan_ranges = truncate_scan(scan_msg);
            preprocessScan(truncated_scan_ranges);
            //for(int p=0;p<truncated_scan_ranges.size();p++){
            //    std::cout << truncated_scan_ranges[p] << ", \n";
            //}
            //std::cout << "Size of truncated scan ranges after preprocessing is is: " << truncated_scan_ranges.size() << "\n";
            std::vector<double> non_holo_dists = non_holonomic_dists(truncated_scan_ranges);
            std::pair<double,double> total_force = resultant_repulsive_force(non_holo_dists);
            double R_avoid = get_R_avoid(total_force);
            double K_avoid = 1/R_avoid;
            //std::cout << "K_avoid is: " << K_avoid << "\n";
            double K_total = K_avoid + K_track_;
            // Publishing command
            ackermann_msgs::AckermannDriveStamped apf_drive_command;
            apf_drive_command.header.stamp = ros::Time::now();
            apf_drive_command.header.frame_id = "base_link";
            float steer = wheel_base_*K_total;
            clip_angle(steer);
            apf_drive_command.drive.steering_angle = steer;
            double F = sqrt(std::pow(total_force.first,2) + std::pow(total_force.second,2));
            //std::cout << "Force: " << F << "\n"; 
            double speed = std::max(2.5,(max_speed_ - K_F_*F - K_delta_*abs(steer)));
            std::cout << "Force: " << F << " speed is: " << speed << "\n";
            apf_drive_command.drive.speed = speed;  
            drive_command_publisher_.publish(apf_drive_command);
            // Resetting K_track_ to 0 again so that the car does not steer away from path.
            //K_track_ = 0.0;
        }
    } // scan callback ends here.

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
        path_detected_ = true;
    }

    void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& car_pose){
     // Pure pursuit algorithm implemented here.
        if(path_detected_){
            double car_x = car_pose->pose.position.x;
            double car_y = car_pose->pose.position.y;
            double target_point_x = 0.0;
            double target_point_y = 0.0; 
            double target_point_dist = 0.0;
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
            for(int i=path_seq_;i<(path_size_-1); i++){
                //std::cout << "i: " << i << "\n"; 
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
                //std::cout << "ld: " << lookahead_distance_ << ", discriminant: " << discriminant << "\n"; 
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
                //std::cout << "Intersection found !!!\n";
                target_point_x = target_point_car_frame.point.x;
                target_point_y = target_point_car_frame.point.y; 
                target_point_dist = sqrt(target_point_x*target_point_x + target_point_y*target_point_y);
                //std::cout << "Intersection with path found and distance from car is: " << target_point_dist << "\n";
            }
            else{
                //std::cout << "No intersection between lookahead radius cirlcle and path found.\n";
                // Since no point of intersection found. Finding the closest point on the path and tracking that.
                double current_best_waypoint_dist = 999999.00;
                int current_best_waypoint_seq;
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
                path_detected_ = false;
                goal_reached_ = true;
                target_point_x = 0.0;
                target_point_y = 0.0; 
                target_point_dist = 0.0;
                K_track_ = 0.0;
            }
            else{
                //std::cout << "The lookahead distance used is: " << target_point_dist << "\n";
                float steering_angle = 2*(target_point_y)/(target_point_dist*target_point_dist);
                //std::cout << "Steering angle: " << steering_angle*(180/PI) << "\n";
                clip_angle(steering_angle);
                //std::cout << "Clipped steering angle: " << steering_angle*(180/PI) << "\n";
                K_track_ = steering_angle;
                //std::cout << "K_track is: " << steering_angle << "\n";
            }
        } // if(path_detected) block ends here. 
        else{
            //std::cout << "No path detected to track.\n";
        }          
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "improved_artificial_potential_field_algorithm_node");
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    apf_controller apf;
    //ros::Duration(1.0).sleep();
    std::cout << "APF node created!!\n";
    ros::spin();
    return 0;
}