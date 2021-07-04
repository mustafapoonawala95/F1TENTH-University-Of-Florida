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

class rrt_star_planner {
    private:

    ros::Subscriber map_subscriber;
    ros::Publisher path_publisher;

    int INF;
    int rows_;
    int cols_;
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
    float jump_threshold;
    float search_radius;
    int iteration_count;
    int total_iterations;
    std::vector<int> visited;
    std::vector<int> obstacles;                                   
    std::vector<float> distances;
    std::vector<int> prev;
    int loop_count;
    float time_to_solve;
    nav_msgs::Path rrt_star_path;
    

    public:
    bool goal_reached;
    rrt_star_planner(ros::NodeHandle *nh) {
        map_subscriber = nh->subscribe("/map", 1, &rrt_star_planner::map_callback, this);
        path_publisher = nh->advertise<nav_msgs::Path>("/rrt_star_path", 1);
        rows_ = 1;
        cols_ = 1;
        INF = 99999;
        distances.resize((rows_*cols_),99999.00);
        prev.resize((rows_*cols_),INF);
        iteration_count = 1;
        goal_reached = false;
    }

    int get_idx(int r, int c){
        return ((r*cols_) + c);
    }

    int get_subscriber_count(){
        return path_publisher.getNumSubscribers();
    }

    void publish_path(){
        if(goal_reached){
            path_publisher.publish(rrt_star_path);
        }
        else{
            std::cout << "No path to destination exists \n";
        }
    }

    std::pair<float, float> get_center_coordinates(int idx){
        int r = idx/cols_;
        int c = idx - (r*cols_);
        float pos_x = map_origin_x + (c + 0.5)*gridsize;
        float pos_y = map_origin_y + (r + 0.5)*gridsize;
        return std::make_pair(pos_x,pos_y);
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

    float compute_distance(int node1_idx, int node2_idx){                      // Euclidian distance.
        std::pair<float, float> node1_coordinates =  get_center_coordinates(node1_idx);
        std::pair<float, float> node2_coordinates =  get_center_coordinates(node2_idx);        
        float Dx = (node1_coordinates.first - node2_coordinates.first);
        float Dy = (node1_coordinates.second - node2_coordinates.second);
        float dist = sqrt((Dx*Dx) + (Dy*Dy));
        return dist;
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

            int col = corrected_point.first/gridsize;
            int row = corrected_point.second/gridsize;

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

    std::vector<int> get_neighbours(int node_idx){
        std::vector<int> neighbours;
        for(int neighbour_idx:visited){
            if((compute_distance(neighbour_idx, node_idx) < search_radius) && (neighbour_idx!=node_idx)){
                neighbours.push_back(neighbour_idx);
            }
        }
        return neighbours;
    }

    void rewire_node(int potential_parent_node, int child_node){
        float c = distances[potential_parent_node] + compute_distance(potential_parent_node, child_node);
        if(c < distances[child_node]){
            if(check_obstacles(potential_parent_node, child_node)){
                prev[child_node] = potential_parent_node;
                distances.at(child_node) = c;
            }
        }
    }

    //==================================================================================================================
    //=================================================================================================================
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
    }
    //==================================================================================================================

    void create_final_path(){
        path.push_back(dest_index);
        //std::cout << "Size of path after pushing destination index is " << path.size() << "\n";
        //std::cout << "Just entering the while loop for creating the path \n";
        while(true)
        {
            //std::cout << "loop_count = " << loop << "\n";
            //std::cout << "Adding to the path: " << prev[path.back()] << "\n";
            path.push_back(prev[path.back()]);                           // Adding the parent of each index to path starting from destination.
            if(path.back() == source_index){                                   
                break;
            }
        }
        //======================== Adding poses to Path message============================================
        rrt_star_path.header.frame_id = "map";
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
        rrt_star_path.poses = my_poses;
        //std::cout << "Just exited the while loop for creating the path \n";
        int psize = path.size();
        ROS_INFO("Path size:  == %d \n", psize);

        //time_to_solve = ros::Time::now().toSec() - current_time; 
        ROS_INFO("Path found using RRT* algorithm.\n");
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
        ROS_INFO("callback started. \n");
        float current_time = ros::Time::now().toSec();
        rows_ = map->info.height;
        std::cout << "rows_ from map height:= " << rows_ <<'\n';
        cols_ = map->info.width;
        std::cout << "cols_ from map width:= " << cols_ <<'\n';
        //distances.resize(rows_*cols_, 99999.00);
        gridsize = map->info.resolution;
        jump_threshold = 2*sqrt(2)*gridsize;
        search_radius = 3*sqrt(2)*gridsize;
        std::cout << "gridsize from map resolution:= " << gridsize <<'\n';
        map_origin_x = map->info.origin.position.x;
        map_origin_y = map->info.origin.position.y;
        prev.resize(rows_*cols_, INF);
        distances.resize(rows_*cols_, 99999.00);
        //visited.resize(rows_*cols_, false);

        for(int i=0;i<(rows_*cols_);i++){                                 // Create vector of indices of all occupied cells.
            prev.at(i) = INF;
            if(map->data[i]==100){
                obstacles.push_back(i);
                //std::cout << "Index " << i << " is an obstacle.\n";
            }
            
        }
        std::cout << "Number of obstacles is: " << obstacles.size() << "\n";

        while(true){
            std::cout << "Enter source row and col. Make sure they are within map range and also not on any obstacle. \n";
            std::cin >> source_row >> source_col;
            std::cout << "source_row: " << source_row << "\n"; 
            std::cout << "source_col: " << source_col <<"\n";
            if(source_row < 0 || source_row >= rows_ || source_col < 0 || source_col >= cols_ || (std::find(obstacles.begin(), obstacles.end(),get_idx(source_row,source_col))!=obstacles.end()))
            {
                std::cout << "[ERROR] The source is either out of map range or on an obstacle. Please try again \n";
            }
            else{
                std::cout << "Enter destination row and col. Make sure they are within map range and also not on any obstacle. \n";
                std::cin >> dest_row >> dest_col;
                if(dest_row < 0 || dest_row >= rows_ || dest_col < 0 || dest_col >= cols_ || (std::find(obstacles.begin(), obstacles.end(),get_idx(dest_row,dest_col))!=obstacles.end()))
                {
                    std::cout << "[ERROR] The destination is either out of map range or on an obstacle. Please try again \n";
                }
                else{
                    std::cout << "dest_row: " << dest_row << "\n"; 
                    std::cout << "dest_col: " << dest_col <<"\n";
                    std::cout << "Now enter the total iterations to perform. The input should be an integer >0 and <="
                    << ((rows_*cols_)-obstacles.size()) << "\n"; 
                    std::cin >> total_iterations;
                    if(total_iterations > 0 && total_iterations<=((rows_*cols_)-obstacles.size())){
                        break;
                    }
                }
            }
            //std::cout << "In the while loop \n";
        }
        //std::cout << "Exited the while loop \n";
        //=================================================================================

        source_index = get_idx(source_row, source_col);
        dest_index = get_idx(dest_row, dest_col);
        prev[source_index] = 0;                                                 // Adding the parent of source as 0.
        distances.at(source_index) = 0;                                         //Initializing source node distance as 0
        
        visited.push_back(source_index);

        if(close_to_dest(source_index)) {       // Start by checking if source is close enough to dest already.
            prev[dest_index] = source_index;
            distances[dest_index] = compute_distance(source_index,dest_index); 
            goal_reached = true;
            std::cout << "The source was very close to goal!! \n";
            iteration_count = total_iterations;
        }

        //=============================================================================================================
        //===================================== RRT* Algorithm starts here ============================================
        //=============================================================================================================
        while(iteration_count < total_iterations){           // Will keep looping till all cells are visited.
            int new_node_idx = generate_random_node();
            int nearest_node_idx = find_nearest_node(new_node_idx);
            int corrected_new_node_idx = correct_new_node(nearest_node_idx, new_node_idx);

            if(!(std::find(visited.begin(), visited.end(), corrected_new_node_idx) != visited.end())){
                if(check_obstacles(nearest_node_idx, corrected_new_node_idx)){
                    prev[corrected_new_node_idx] = nearest_node_idx;       // Adding nearest node as parent of corrected new node.
                    distances[corrected_new_node_idx] = distances[nearest_node_idx] + compute_distance(corrected_new_node_idx,nearest_node_idx);
                    visited.push_back(corrected_new_node_idx);              // Adding corrected new node to visited set(tree).
                    iteration_count++;
                    std::vector<int> neighbour_nodes = get_neighbours(corrected_new_node_idx);
                    for(int neighbour_idx:neighbour_nodes){
                        rewire_node(neighbour_idx, corrected_new_node_idx);                   
                    }
                    for(int neighbour_idx:neighbour_nodes){
                        rewire_node(corrected_new_node_idx, neighbour_idx);
                    }

                    if(corrected_new_node_idx == dest_index){
                        goal_reached = true;
                        std::cout << "Last visited node was the destnation, destination reached. \n";
                    }
                    else if(close_to_dest(corrected_new_node_idx) && check_obstacles(corrected_new_node_idx, dest_index)){             // Checking if corrected new node is close enough to dest.
                        prev[dest_index] = corrected_new_node_idx;
                        distances[dest_index] = distances[corrected_new_node_idx] + compute_distance(corrected_new_node_idx, dest_index);
                        goal_reached = true;
                        std::cout << "The last new node was close enough to dest to join to it. \n";
                    }   
                }
            }

        }
        //std::cout << "Just exited the while loop \n";
        //std::cout << "Visited.size() = " << visited.size() << "\n"
        if(goal_reached){
            std::cout << "The length of path to destination is: " << distances[dest_index]<<'\n';
            create_final_path();
        }
        else{
            std::cout << "The goal was not reached within " << total_iterations << " iterations, try a higher number of iterations.\n";
        }
    }
}; // class ends here.

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_RRT_star_node");
    ros::NodeHandle nh;
    rrt_star_planner planner = rrt_star_planner(&nh);
    ros::Duration(1).sleep();
    ros::spinOnce();
    std::cout << "Waiting for somebody to subscribe to /rrt_star_path topic. \n";
    if(planner.goal_reached){
        while(planner.get_subscriber_count()==0){            // Will keep on waiting till Rviz subscribes to /shortest_path topic.
            planner.publish_path();
            std::cout << "....\n";
            ros::Duration(5).sleep();
        }
        std::cout << "Somebody subscribed to /rrt_star_path topic. \nPublishing one last time after 30 seconds and exiting.\n";
        ros::Duration(10).sleep();
        for(int i=0;i<10;i++){
            planner.publish_path();
            ros::Duration(0.5).sleep();
        }
    }
    else {
        std::cout << "Goal not reached, so just exiting.\n";
    }    
    return 0;
}




