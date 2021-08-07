/* 
@file: motion_planning_dijkstra_node.cpp
@author: Mustafa Poonawala

Dijkstra planner node, subscribes to /map topic to get occupancy grid map, /gt_pose for car's position 
and /move_base_simple/goal for goal position.
*/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <vector>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h>
#include <cmath>

class dijkstra_planner {
    private:

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
    std::vector<int>dr; 
    std::vector<int>dc;
    std::vector<int> prev;
    std::priority_queue<std::pair<float,int>> pq;
    int loop_count;
    float time_to_solve;
    nav_msgs::Path dijkstra_path;

    public:
        
    dijkstra_planner(ros::NodeHandle *nh) {
        map_subscriber = nh->subscribe("/map", 1, &dijkstra_planner::map_callback, this);
        car_pose_subscriber = nh->subscribe("/gt_pose", 1, &dijkstra_planner::car_pose_callback, this);
        destination_subscriber = nh->subscribe("/move_base_simple/goal", 1, &dijkstra_planner::destination_callback, this);
        path_publisher = nh->advertise<nav_msgs::Path>("/dijkstra_shortest_path", 1);
        rows_ = 1;
        cols_ = 1;
        INF = 99999.00;
        dr = {1, -1, 0, 0, 1, 1, -1, -1};
        dc = {0, 0, 1, -1, 1, -1, 1, -1};
        gridsize = 0.01;                    // Initializing gridsize so that mapcallback is executed once.
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
        path_publisher.publish(dijkstra_path);
    }

    float compute_distance(int node1_idx, int node2_idx){                      // Euclidian distance.
        std::pair<float, float> node1_coordinates =  get_center_coordinates(node1_idx);
        std::pair<float, float> node2_coordinates =  get_center_coordinates(node2_idx);        
        float Dx = (node1_coordinates.first - node2_coordinates.first);
        float Dy = (node1_coordinates.second - node2_coordinates.second);
        float dist = sqrt((Dx*Dx) + (Dy*Dy));
        return dist;
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
        dijkstra_path.header.frame_id = "map";
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
        dijkstra_path.poses = my_poses;
        //std::cout << "Just exited the while loop for creating the path \n";
        int psize = path.size();
        ROS_INFO("Path size:  == %d \n", psize);

        //time_to_solve = ros::Time::now().toSec() - current_time; 
        ROS_INFO("Path found using dijkstra algorithm.\n");
        prev.clear();           // Clearing parent data so that it doesn't interfere with next iteration of A star.
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
        gridsize = map->info.resolution;
        std::cout << "Gridsize is: " << gridsize << "\n";
        map_origin_x = map->info.origin.position.x;
        map_origin_y = map->info.origin.position.y;
        std::cout << "Map origin x and y are: " << map_origin_x << ", " << map_origin_y <<"\n";
        std::cout << "Map origin orientation is: " << map->info.origin.orientation.x << ", " << map->info.origin.orientation.y << ", " << map->info.origin.orientation.z
        << ", " << map->info.origin.orientation.w << "\n";
        map_data_.insert(map_data_.end(), &map->data[0], &map->data[rows_*cols_]);
        inflate_obstacles();
        //map_data_ = map->data;
        //std::cout << "Inflate obstacles successful.\n";
    }

    void destination_callback(const geometry_msgs::PoseStamped::ConstPtr& destination_point){
    //================================================================================
        float x = destination_point->pose.position.x - map_origin_x;
        float y = destination_point->pose.position.y - map_origin_y;
        std::vector<bool> visited(rows_*cols_, false);
        std::vector<float> distances(rows_*cols_, INF);
        prev.resize(rows_*cols_,99999);
        std::cout << "x dest. and y dest. is: " << x << ", " << y << "\n";
        int col = x/gridsize;
        int row = y/gridsize;
        dest_index = get_idx(row, col); 
        std::cout << "Destination index is " << dest_index << "\n";
        for(int i=0;i<(rows_*cols_);i++){                     
            if(map_data_[i]==100){                         //Making all obstacles from the map as visited.
                visited.at(i) = true;
            }
        }
        prev[source_index] = 0;                                                // Adding the parent of source as 0.
        distances.at(source_index) = 0.0;                                       //Initializing source node distance as 0
        pq.push(std::make_pair(-1*(distances[source_index]),source_index));     // Added source node pair to priority queue.
        while(!pq.empty()){
            std::pair<float,int> current_node = pq.top();
            pq.pop();
            int current_node_index = current_node.second;                      
            float current_node_distance = distances[current_node_index];
            if(visited[current_node_index]){                                    // Checking if this node was visited earlier.
                continue;
            }                             
            else
            {
                visited[current_node_index] = true;                                 //Making current node as visited.
                
                int r = current_node_index/cols_;
                int c = current_node_index - (r*cols_);

                //ROS_INFO("Current node: row,col:  = %d %d \n", r,c);
                int rr, cc;
                for(int i=0; i<8; i++)              //Going through the 8 neighbours of current node.
                {
                    rr = r + dr[i];
                    cc = c + dc[i];

                    if(rr < 0 || rr >= rows_ || cc < 0 || cc >= cols_ || visited[get_idx(rr,cc)])
                        continue;

                    else{

                        int neighbour_index = get_idx(rr, cc);             
                        float neighbour_dist = gridsize;              
                                                                        
                                                                            
    
                        if(i>3){
                            neighbour_dist = 1.414*gridsize;                   // Cost of going diagonal is 1.4 times.
                        }
                        float alt = current_node_distance + neighbour_dist;      
                        if(alt < distances[get_idx(rr, cc)]){
                            distances[get_idx(rr, cc)] = alt;
                            prev[get_idx(rr, cc)] = current_node_index;      // Updating parent of neighbour as the current node.
                            pq.push(std::make_pair(-1*(distances[neighbour_index]),neighbour_index));  // Pushing current neighbour in pq.
                        }
                        if(visited[dest_index]){
                            //ROS_INFO("Reached goal \n");
                            break;
                        }

                    }
                }
                if(visited[dest_index]){
                    ROS_INFO("Reached goal \n");
                    std::cout << loop_count << " nodes were searched.\n";
                    break;
                }
            }
            loop_count++;
        }
        if(!visited[dest_index]){                  // No path found if destination is not visited after searching all nodes. 
            std::cout << "No path to goal exists.\n";
        }
        else{
            std::cout << "+++++++++++++++++++ Calling create_final_path().\n";
            create_final_path();
            std::cout << "=================== Called create_final_path() succesfully.\n";
            publish_path();
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_test_node");
    ros::NodeHandle nh;
    dijkstra_planner planner = dijkstra_planner(&nh);
    ros::Duration(1).sleep();
    ros::spin();
    return 0;
}