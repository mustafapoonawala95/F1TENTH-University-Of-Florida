/* 
@file: motion_planning_RRT_node.cpp
@author: Mustafa Poonawala

RRT planner node, subscribes to /map topic to get occupancy grid map, /gt_pose for car's position 
and /move_base_simple/goal for goal position.
*/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <bits/stdc++.h>
#include <random>
#include <cmath>

class rrt_planner {
    private:

    ros::NodeHandle node_handle_;
    ros::Subscriber map_subscriber;
    ros::Subscriber car_pose_subscriber;
    ros::Subscriber destination_subscriber;
    ros::Publisher path_publisher;

        //=============== Declare variables but don't define anything. Do that in constructor.================
    int INF;
    int rows_;
    int cols_;
    int RowColProduct = rows_*cols_;
    std::vector <int> path;
    int source_row;
    int source_col;
    int dest_row;
    int dest_col;
    int source_index;
    int dest_index;
    int current_node_idx;
    float gridsize;
    float map_origin_x;
    float map_origin_y;
    std::vector<int> map_data_;
    std::vector<int> visited;
    std::vector<int> obstacles;
    float jump_threshold;
    std::vector<int> prev;
    int loop_count;
    float time_to_solve;
    nav_msgs::Path rrt_path;


    public:
        
    rrt_planner(){
        node_handle_ = ros::NodeHandle();
        map_subscriber = node_handle_.subscribe("/map", 1, &rrt_planner::map_callback, this);
        car_pose_subscriber = node_handle_.subscribe("/gt_pose", 1, &rrt_planner::car_pose_callback, this);
        destination_subscriber = node_handle_.subscribe("/move_base_simple/goal", 1, &rrt_planner::destination_callback, this);
        path_publisher = node_handle_.advertise<nav_msgs::Path>("/rrt_path", 1);
        rows_ = 1;
        cols_ = 1;
        INF = 99999.00;
        gridsize = 0.01;
        loop_count = 0;
    }

    int get_idx(int r, int c){
        return ((r*cols_) + c);
    }

    void inflate_obstacles(){
        std::set<int> indexes_to_fill;
        for(int i=0;i<map_data_.size();i++){
            if(map_data_[i]==100){   //Checking for obstacle.
                int r = i/cols_;
                int c = i - r*cols_;
                for(int j=-6; j<=6; j++){       // Inflating 6 rows and 6 cols around the original obsatcle. 
                    for(int k=-6;k<=6;k++){
                        int rr,cc;
                        rr = r + j;
                        cc = c + k;
                        if(rr<0 || rr >= rows_ || cc < 0 || cc >= cols_){
                        }
                        else{
                            int index = get_idx(rr,cc);
                            indexes_to_fill.insert(index);      // Pushing into a set so indexes are not repeated.
                        } 
                    }
                }
            }
        }
        for(auto l:indexes_to_fill){
            map_data_[l] = 100;    // Inflating obstacles.
        }
    }

    std::pair<float, float> get_center_coordinates(int idx){
        int r = idx/cols_;
        int c = idx - (r*cols_);
        float pos_x = map_origin_x + (c + 0.5)*gridsize;   
        float pos_y = map_origin_y + (r + 0.5)*gridsize;
        return std::make_pair(pos_x,pos_y);
    }

    int get_subscriber_count(){
        return path_publisher.getNumSubscribers();
    }

    void publish_path(){
        path_publisher.publish(rrt_path);
    }

    float compute_distance(int node1_idx, int node2_idx){                      // Euclidian distance.
        std::pair<float, float> node1_coordinates =  get_center_coordinates(node1_idx);
        std::pair<float, float> node2_coordinates =  get_center_coordinates(node2_idx);        
        float Dx = (node1_coordinates.first - node2_coordinates.first);
        float Dy = (node1_coordinates.second - node2_coordinates.second);
        float dist = sqrt((Dx*Dx) + (Dy*Dy));
        return dist;
    }

    int generate_random_node(){
        std::random_device device;
        std::mt19937 generator(device());
        std::uniform_int_distribution<int> distribution(0,(rows_*cols_)-1);
        bool valid_node = true;
        //std::cout << "obstacles vector size is " << obstacles.size() << "\n";
        while(true){
            valid_node = true;
            //std::cout << "Generating random node index.\n";
            int index = distribution(generator);
            //std::cout << "Random node index = " << index << "\n";
            if(std::find(visited.begin(), visited.end(), index) != visited.end()){
                valid_node = false;
                //std::cout << "This node is already visited = " << index << "\n";
            }
            else if(std::find(obstacles.begin(), obstacles.end(), index) != obstacles.end()){
                valid_node = false;
                //std::cout << "Random node index clashed with an obstacle.\n";
            }

            if(valid_node){
                //std::cout << "Found valid node index, " << index <<  " returning and exiting function \n";
                return index;
            }
        }
    }

    int find_nearest_node(int random_node_idx){
        float lowest_dist = 99999.00;
        int nearest_node_idx;
        float dist;
        for(int i=0;i<visited.size();i++){
            dist = compute_distance(random_node_idx, visited[i]);
            //std::cout << "Distance between new node and " << i << " element of visited is " << dist << "\n"; 
            if(dist<lowest_dist){
                lowest_dist = dist;
                nearest_node_idx = visited[i];
            }
        }
        //std::cout << "The nearest node index is " << nearest_node_idx << " and it's distance from new node is " << lowest_dist 
        //<< "\n";
        return nearest_node_idx;
    }

    int correct_new_node(int current_nearest_node_idx, int new_node_index){
        int idx = new_node_index;
        if(compute_distance(current_nearest_node_idx, new_node_index) > jump_threshold){
            //std::cout << "Nearest node is farther than the threshold distance so correcting the random node.\n";
            std::pair<float, float> current_nearest_node = get_center_coordinates(current_nearest_node_idx);
            std::pair<float, float> new_node = get_center_coordinates(new_node_index);

            float slope = (new_node.second-current_nearest_node.second)/(new_node.first-current_nearest_node.first);
            float t = jump_threshold/(compute_distance(current_nearest_node_idx, new_node_index));
            std::pair<float, float> corrected_point;

            corrected_point.first = (1 - t)*current_nearest_node.first + t*new_node.first;
            corrected_point.second = ((1 - t)*current_nearest_node.second + t*new_node.second);

            int col = (corrected_point.first - map_origin_x)/gridsize;
            int row = (corrected_point.second - map_origin_y)/gridsize;

            idx = get_idx(row, col); 
        }
        //std::cout << "Corrected node is: " << idx << "\n";
        return idx;
    }

    bool close_to_dest(int node_idx){
        bool result;
        result = (compute_distance(node_idx, dest_index)<=jump_threshold);
        return result;
    }

    void create_final_path(){
        std::vector <int> path;
        path.push_back(dest_index);
        while(true)
        {
            path.push_back(prev[path.back()]);         // Adding the parent of each index to path starting from destination.
            if(path.back() == source_index){                                   
                break;
            }
        }
        rrt_path.header.frame_id = "map";
        std::vector<geometry_msgs::PoseStamped> my_poses(path.size());
        int l=0;
        for(int k=(path.size()-1);k>=0;k--){
            std::pair<float, float> cell_center =  get_center_coordinates(path[k]);
            my_poses[l].pose.position.x = cell_center.first;
            my_poses[l].pose.position.y = cell_center.second;
            my_poses[l].pose.position.z = 0.00;
            my_poses[l].header.seq = l;
            l++;
            //std::cout << "this is l: " << l << "\n";
        }
        rrt_path.poses = my_poses;
        //std::cout << "Just exited the while loop for creating the path \n";
        int psize = path.size();
        ROS_INFO("Path size:  == %d \n", psize);

        //time_to_solve = ros::Time::now().toSec() - current_time; 
        ROS_INFO("Path found using RRT algorithm.\n");
        prev.clear();           // Clearing parent data so that it doesn't interfere with next iteration of A star.
    }


    // Old check obstacle function. 
    /*
    bool check_obstacles(int node1_idx, int node2_idx){
        std::pair<float, float> node1_coordinates =  get_center_coordinates(node1_idx);
        std::pair<float, float> node2_coordinates =  get_center_coordinates(node2_idx);
        std::pair<float,float> curr_obstacle;
        float curr_obstaclex;
        float curr_obstacley;
        float node1x = node1_coordinates.first;
        float node1y = node1_coordinates.second;
        float node2x = node2_coordinates.first;
        float node2y = node2_coordinates.second; 
        float slope;
        float c;
        std::vector<double> arr(4);
        // Checking if obstacle x and y  is betwee the 2 nodes.
        for(int i=0;i<obstacles.size();i++){
            curr_obstacle = get_center_coordinates(obstacles[i]);
            curr_obstaclex = curr_obstacle.first;
            curr_obstacley = curr_obstacle.second;
            // First checking special case of vertical line joining the 2 nodes.
            if(node1x == node2x){
                if(curr_obstaclex == node1x){
                    if(((node1y >= curr_obstacley) && (curr_obstacley >= node2y)) || ((node1y <= curr_obstacley) && (curr_obstacley <= node2y))){
                        return false;
                    }
                }
            } 
            else if(((node1x >= curr_obstaclex) && (curr_obstaclex >= node2x)) || ((node1x <= curr_obstaclex) && (curr_obstaclex <= node2x))){
                if(((node1y >= curr_obstacley) && (curr_obstacley >= node2y)) || ((node1y <= curr_obstacley) && (curr_obstacley <= node2y))){
                    slope = (node2y-node1y)/(node2x-node1x);
                    c = node2y - slope*node2x;
                    arr[0] = (curr_obstacley + (0.5*gridsize)) - (slope*(curr_obstaclex + (0.5*gridsize))) - c;
                    arr[1] = (curr_obstacley + (0.5*gridsize)) - (slope*(curr_obstaclex - (0.5*gridsize))) - c;
                    arr[2] = (curr_obstacley - (0.5*gridsize)) - (slope*(curr_obstaclex + (0.5*gridsize))) - c;
                    arr[3] = (curr_obstacley - (0.5*gridsize)) - (slope*(curr_obstaclex - (0.5*gridsize))) - c;
                    double count = 0;

                    for(int k=0;k<4;k++){
                        if(std::fabs(arr[k]) <= 0.0000001){
                            arr[k] = 0;
                        }else{
                            count += (arr[k]/std::fabs(arr[k]));
                        }
                    }
                    if(std::abs(count)<3){
                        //std::cout << "Atleast one obstacle is in the way.\n";
                        return false;
                    }
                }
            }                                
        }
        //std::cout << "No obstacle in way.\n";
        return true;
    } */

    bool check_obstacles(int node1_idx, int node2_idx){
        std::pair<float, float> node1_coordinates =  get_center_coordinates(node1_idx);
        std::pair<float, float> node2_coordinates =  get_center_coordinates(node2_idx); 
        int number_of_steps = 10;
        int col;
        int row;
        int idx;
        double x_step = (node2_coordinates.first - node1_coordinates.first)/number_of_steps;
        double y_step = (node2_coordinates.second - node1_coordinates.second)/number_of_steps;

        double current_x = node1_coordinates.first - map_origin_x;
        double current_y = node1_coordinates.second - map_origin_y;
        
        for(int i=0; i<number_of_steps; i++){
            current_x += x_step;
            current_y += y_step;
            col = current_x/gridsize;
            row = current_y/gridsize;
            idx = get_idx(row, col);
            if(map_data_[idx] == 100){
                return false;
            }
        }
        return true;
    }


    void car_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& car_pose){
        float x = car_pose->pose.position.x - map_origin_x;
        float y = car_pose->pose.position.y - map_origin_y;
        //std::cout << "x car and y car is: " << x << ", " << y << "\n";
        int col = x/gridsize;
        int row = y/gridsize;
        source_index = get_idx(row, col); 
        //std::cout << "Source index is " << source_index << "\n";
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
        ROS_INFO("Map callback started. \n");
        //float current_time = ros::Time::now().toSec();
        rows_ = map->info.height;
        std::cout << "rows_ from map height:= " << rows_ <<'\n';
        cols_ = map->info.width;
        std::cout << "cols_ from map width:= " << cols_ <<'\n';
        //visited.resize(rows_*cols_,false);
        prev.resize(rows_*cols_,99999);
        gridsize = map->info.resolution;
        std::cout << "Gridsize is: " << gridsize << "\n";
        map_origin_x = map->info.origin.position.x;
        map_origin_y = map->info.origin.position.y;
        map_data_.insert(map_data_.end(), &map->data[0], &map->data[rows_*cols_]);
        jump_threshold = 0.5;//30*gridsize;

        inflate_obstacles();
        for(int i=0;i<(rows_*cols_);i++){                     
            if(map_data_[i]==100){                         //Making all obstacles from the map as visited.
                obstacles.push_back(i);
            }
        }
        //map_data_ = map->data;
        std::cout << "Map origin x and y are: " << map_origin_x << ", " << map_origin_y <<"\n";
        std::cout << "Map origin orientation is: " << map->info.origin.orientation.x << ", " << map->info.origin.orientation.y << ", " << map->info.origin.orientation.z
        << ", " << map->info.origin.orientation.w << "\n";
    }

    void destination_callback(const geometry_msgs::PoseStamped::ConstPtr& destination_point){
        float x = destination_point->pose.position.x - map_origin_x;
        float y = destination_point->pose.position.y - map_origin_y;
        std::cout << "x dest. and y dest. is: " << x << ", " << y << "\n";
        int col = x/gridsize;
        int row = y/gridsize;
        dest_index = get_idx(row, col); 
        //std::cout << "Destination index is " << dest_index << "\n";
        prev[source_index] = 0;
        visited.push_back(source_index);
        bool goal_reached = false;
        if(close_to_dest(source_index)) {       // Start by checking if source is close enough to dest already.
            prev[dest_index] = source_index;
            //distances[dest_index] = compute_distance(source_index,dest_index); 
            goal_reached = true;
            visited.clear();
            std::cout << "The source was very close to goal!! \n";
        }
        //=============================================================================================================
        //====================================== RRT Algorithm starts here ============================================
        //=============================================================================================================
        while(!goal_reached){
            int new_node_idx = generate_random_node();
            int nearest_node_idx = find_nearest_node(new_node_idx);
            int corrected_new_node_idx = correct_new_node(nearest_node_idx, new_node_idx);
            if(!(std::find(visited.begin(), visited.end(), corrected_new_node_idx) != visited.end())){
                if(check_obstacles(nearest_node_idx, corrected_new_node_idx)){
                    prev[corrected_new_node_idx] = nearest_node_idx;
                    //distances[corrected_new_node_idx] = distances[nearest_node_idx] + compute_distance(corrected_new_node_idx,nearest_node_idx);
                    visited.push_back(corrected_new_node_idx);              // Adding corrected new node to visited set(tree).
                    //std::cout << "Visited.size() is: " << visited.size() << "\n"; 
                    if(corrected_new_node_idx == dest_index){
                        goal_reached = true;
                        visited.clear();
                        std::cout << "Last visited node was the destnation, destination reached. \n";
                    }
                    else if(close_to_dest(corrected_new_node_idx) && check_obstacles(corrected_new_node_idx, dest_index)){             // Checking if corrected new node is close enough to dest.
                        prev[dest_index] = corrected_new_node_idx;
                        //distances[dest_index] = distances[corrected_new_node_idx] + compute_distance(corrected_new_node_idx, dest_index);
                        goal_reached = true;
                        visited.clear();
                        std::cout << "The last new node was close enough to dest to join to it. \n";
                    }
                }
            }
        }
        create_final_path();
        publish_path();
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_rrt_node");
    rrt_planner planner;
    ros::Duration(1).sleep();
    std::cout << "motion planning RRT node created!!\n";
    ros::spin();
    return 0;
}