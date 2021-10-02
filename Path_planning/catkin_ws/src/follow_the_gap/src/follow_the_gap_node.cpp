/* 
@file: follow_the_gap_node.cpp
@author: Mustafa Poonawala

ADD DESCRIPTION.

*/

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

// Some global variables.
tf2_ros::Buffer tf_buffer;
const float PI = 3.14159;

class follow_the_gap {

    private:

    ros::NodeHandle node_handle_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher drive_command_publisher_;
    float max_steering_angle_ = 0.4189;   
    //std::vector<float> scanned_ranges_;
    float scan_angle_min_;
    float scan_angle_max_;
    
    // Tunable parameters. Tuned from launch file.
    float obstacle_distance_threshold_;            // Any obstacle further than this is not urgent enough to act on.  
    float field_of_view_angle_degrees_;            // Obstacles will only be considered in a window of this size in front of vehicle.
    float lookahead_distance_;  
    float safety_bubble_radius_;
    int sliding_window_length_;
    float safety_angle_;
    float buffer_angle_scaling_factor_;
    std::string planner_topic_;

    public:

    double Rad2Deg(double r) {return r*180/PI;}

    double Deg2Rad(double d) {return d*PI/180;}

    std::vector<float> truncate_scan(const sensor_msgs::LaserScan::ConstPtr& original_scan,float& field_of_view_min_angle,
    float& field_of_view_max_angle, float& scan_angle_increment){

        double half_fov = Deg2Rad(field_of_view_angle_degrees_/2);
        int fov_min_id = static_cast<int>(((-half_fov - original_scan->angle_min)/original_scan->angle_increment));
        int fov_max_id = static_cast<int>((half_fov - original_scan->angle_min)/original_scan->angle_increment);
        scan_angle_increment = original_scan->angle_increment;
        field_of_view_min_angle = original_scan->angle_min + fov_min_id*original_scan->angle_increment;
        field_of_view_max_angle = original_scan->angle_min + fov_max_id*original_scan->angle_increment;
        //std::cout << "fov_min_id: " << fov_min_id <<"\n";
        //std::cout << "fov_max_id: " << fov_max_id <<"\n";
        return std::vector<float>(original_scan->ranges.begin() + fov_min_id,original_scan->ranges.begin() + fov_max_id);
    }

    void preprocessScan(std::vector<float>& truncated_scan_ranges, float range_min, float range_max){
        // clip ranges between min and max.
        for(float& r : truncated_scan_ranges) {
            if(r < range_min){
                r = range_min;
            }
            else if(r > range_max){
                r = range_max;
            }
        }
        // impose sliding window running average on scan data
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

    double findRadius(double sideA, double sideB, double included_angle) {
        // Using Law of Cosines
        return std::sqrt(sideA*sideA + sideB*sideB - 2*sideA*sideB*std::cos(included_angle));
    }

    void create_angle_buffer(std::vector<float>& truncated_scan_ranges, int safety_angle_indices){

        std::vector<std::pair<int,int>> obstacle_regions;
        int i = 0;
        while(i < truncated_scan_ranges.size()){
            if(truncated_scan_ranges[i] == 0){
                int obstacle_start_index = i;
                int j = i;
                while(j < truncated_scan_ranges.size() && truncated_scan_ranges[j] == 0){
                    ++j;
                }
                int obstacle_end_index = j;
                obstacle_regions.emplace_back(obstacle_start_index, obstacle_end_index);
                i = obstacle_end_index;
            }
            ++i;
        }
        // adding "safety_angle_indices" of zeros to the left and right of each obstacle.
        for(const auto& p:obstacle_regions){
            for(int j = p.first; j >= std::max(0, p.first - safety_angle_indices); --j){
                truncated_scan_ranges[j] = 0;
            }
            for (int j = p.second; j < std::min(static_cast<int>(truncated_scan_ranges.size()),p.second + safety_angle_indices);++j){
                truncated_scan_ranges[j] = 0;
            }
        }
    }

    unsigned int findlongestfreespace(std::vector<float>& truncated_scan_ranges){
    
        unsigned int count = 0, max_count = 0, max_end = 0;
        for(unsigned int i = 0; i <= truncated_scan_ranges.size(); ++i) {
            if(i < truncated_scan_ranges.size() && truncated_scan_ranges[i] > 0.f){
                ++count;
            }
            else{
                if(count > max_count){
                    max_count = count;
                    max_end   = i;
                }
                count = 0;
            }
        }
        // finding the max element from the chosen non-zero free space
        auto max_range_ptr = std::max_element(truncated_scan_ranges.begin() + (max_end - max_count), truncated_scan_ranges.begin() + max_end);
        return(max_range_ptr - truncated_scan_ranges.begin());
    }

    follow_the_gap(){
        node_handle_ = ros::NodeHandle();
        node_handle_.getParam("planner_topic_",planner_topic_);   // Getting planner topic from parameter server.
        node_handle_.getParam("obstacle_distance_threshold_",obstacle_distance_threshold_);
        node_handle_.getParam("field_of_view_angle_degrees_",field_of_view_angle_degrees_);
        node_handle_.getParam("lookahead_distance_",lookahead_distance_);
        node_handle_.getParam("safety_bubble_radius_",safety_bubble_radius_);
        node_handle_.getParam("sliding_window_length_",sliding_window_length_);
        node_handle_.getParam("safety_angle_",safety_angle_);  
        node_handle_.getParam("buffer_angle_scaling_factor_",buffer_angle_scaling_factor_);
        scan_subscriber_ = node_handle_.subscribe("/scan", 1, &follow_the_gap::scan_callback,this);
        drive_command_publisher_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
        ros::Duration(1.0).sleep();// Waiting for 1sec for the tf buffer to fill up so that transforms can be found by lookupTransform().
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

        //==================================================================================================
        //================================ Obstacle avoidance through follow the gap =======================
        //==================================================================================================
        // Initializing values.
        float scan_angle_increment = 0.00;
        float field_of_view_min_angle = 0.00;
        float field_of_view_max_angle = 0.00;
        std::vector<float> truncated_scan_ranges = truncate_scan(scan_msg, field_of_view_min_angle, field_of_view_max_angle, scan_angle_increment);
        //std::cout << "Size of truncated scan ranges is: " << truncated_scan_ranges.size() << "\n";
        preprocessScan(truncated_scan_ranges, scan_msg->range_min, scan_msg->range_max);
        //for(int p=0;p<truncated_scan_ranges.size();p++){
        //    std::cout << truncated_scan_ranges[p] << ", \n";
        //}
        //std::cout << "Size of truncated scan ranges after preprocessing is is: " << truncated_scan_ranges.size() << "\n";
        // Finding the closest obstacle and it's index.
        auto min_ptr = std::min_element(truncated_scan_ranges.begin(), truncated_scan_ranges.end());
        auto min_range = *min_ptr;
        if(min_range < obstacle_distance_threshold_){

            int min_idx = min_ptr - truncated_scan_ranges.begin();
            //std::cout << "The closest obstacle's index is: " << min_idx << "\n";
            //std::cout << "The closest obstacle's range is: " << min_range << "\n";
            // Setting range values within a radius of safety_bubble_radius_ to 0
            for (int i = 0; i < truncated_scan_ranges.size(); ++i){
                if(findRadius(truncated_scan_ranges[i], min_range, scan_angle_increment*std::abs(min_idx - i))<=safety_bubble_radius_){
                    truncated_scan_ranges[i] = 0;
                }
            }
            //for(int p=0;p<truncated_scan_ranges.size();p++){
            //  std::cout << truncated_scan_ranges[p] << ", \n";
            //}
            float adaptive_buffer_angle = safety_angle_ + (buffer_angle_scaling_factor_/min_range);  // In degrees.
            int buffer_size_in_indices = Deg2Rad(adaptive_buffer_angle)/scan_msg->angle_increment;
            //std::cout << "Adaptive buffer angle is: " << adaptive_buffer_angle << "\n";
            create_angle_buffer(truncated_scan_ranges, buffer_size_in_indices);
            //for(int p=0;p<truncated_scan_ranges.size();p++){
            //  std::cout << truncated_scan_ranges[p] << ", \n";
            //}
            unsigned int furthest_point_index = findlongestfreespace(truncated_scan_ranges);
            float furthest_point_angle = scan_msg->angle_min + furthest_point_index*scan_angle_increment + Deg2Rad(105);
            //std::cout << "Furthest point index is: " << furthest_point_index << "\n";
            //std::cout << "Furthest point angle is: " << Rad2Deg(furthest_point_angle) << "\n";
            // Choosing the furthest point in free space and setting the steering angle towards it
            double steering_angle = field_of_view_min_angle + furthest_point_index*scan_angle_increment;
            //std::cout << "Steering angle: " << Rad2Deg(steering_angle) <<"\n";
            // Clipping within max_steering_angle_
            if(steering_angle > max_steering_angle_){
                steering_angle = max_steering_angle_;
            }
            else if(steering_angle < -max_steering_angle_){
                steering_angle = -max_steering_angle_;
            }
            // Publishing command
            ackermann_msgs::AckermannDriveStamped follow_gap_drive_command;
            follow_gap_drive_command.header.stamp         = ros::Time::now();
            follow_gap_drive_command.header.frame_id      = "laser";
            follow_gap_drive_command.drive.steering_angle = steering_angle;
            follow_gap_drive_command.drive.speed          = 2.0;  
            drive_command_publisher_.publish(follow_gap_drive_command);
        }
    } // scan callback ends here.
};

int main(int argc, char **argv){

    ros::init(argc, argv, "follow_the_gap_node");
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    follow_the_gap ftg;
    //ros::Duration(1.0).sleep();
    std::cout << "Follow the gap node created!!\n";
    ros::spin();
    return 0;
}

