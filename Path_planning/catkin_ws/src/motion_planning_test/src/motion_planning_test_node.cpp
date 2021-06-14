#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <vector>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <bits/stdc++.h>

class dijkstra_planner {
    private:

    ros::Subscriber map_subscriber;
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
    std::vector<int>dr; // = {1, -1, 0, 0, 1, 1, -1, -1}; // Direction vectors
    std::vector<int>dc;// = {0, 0, 1, -1, 1, -1, 1, -1};
    std::vector<bool> visited;//(RowColProduct,false); 
    std::vector<float> distances;//(rows_*cols_,INF);
    std::vector<int> prev;//(rows_*cols_,INF);
    std::priority_queue<std::pair<float,int>> pq;
    int loop_count;// = 0;
    float time_to_solve;
    nav_msgs::OccupancyGrid optimal_path;

    public:
        
    dijkstra_planner(ros::NodeHandle *nh) {
        map_subscriber = nh->subscribe("/map", 1, &dijkstra_planner::map_callback, this);
        path_publisher = nh->advertise<nav_msgs::OccupancyGrid>("/dijkstra_shortest_path", 1);
        rows_ = 1;
        cols_ = 1;
        INF = 99999;
        dr = {1, -1, 0, 0, 1, 1, -1, -1};
        dc = {0, 0, 1, -1, 1, -1, 1, -1};
        visited.resize(RowColProduct,false);
        distances.resize(RowColProduct,99999.00);
        prev.resize(RowColProduct,INF);
        loop_count = 0;
    }

    int get_idx(int r, int c){
    return ((r*cols_) + c);
    }

    int get_subscriber_count(){
        return path_publisher.getNumSubscribers();
    }

    void publish_path(){
        path_publisher.publish(optimal_path);
    }


    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
    //================================================================================

        ROS_INFO("callback started. \n");
        float current_time = ros::Time::now().toSec();
        rows_ = map->info.height;
        std::cout << "rows_ from map height:= " << rows_ <<'\n';
        cols_ = map->info.width;
        std::cout << "cols_ from map width:= " << cols_ <<'\n';
        distances.resize(rows_*cols_, 99999.00);
        prev.resize(rows_*cols_, INF);
        visited.resize(rows_*cols_, false);

        for(int i=0;i<(rows_*cols_);i++){                     
            distances.at(i) = 99999.00;
            prev.at(i) = INF;
            if(map->data[i]==100){                                             //Making all obstacles from the map as visited.
                visited.at(i) = true;
            }
        }
        while(true){
            std::cout << "Enter source row and col. Make sure they are within map range and also not on any obstacle. \n";
            std::cin >> source_row >> source_col;
            std::cout << "source_row: " << source_row << "\n"; 
            std::cout << "source_col: " << source_col <<"\n";
            if(source_row < 0 || source_row >= rows_ || source_col < 0 || source_col >= cols_ || visited[get_idx(source_row,source_col)])
            {
                std::cout << "[ERROR] The source is either out of map range or on an obstacle. Please try again \n";
            }
            else{
                std::cout << "Enter destination row and col. Make sure they are within map range and also not on any obstacle. \n";
                std::cin >> dest_row >> dest_col;
                if(dest_row < 0 || dest_row >= rows_ || dest_col < 0 || dest_col >= cols_ || visited[get_idx(dest_row,dest_col)]){
                    std::cout << "[ERROR] The destination is either out of map range or on an obstacle. Please try again \n";
                }
                else{
                    std::cout << "dest_row: " << dest_row << "\n"; 
                    std::cout << "dest_col: " << dest_col <<"\n";
                    break;
                }
            }
            std::cout << "In the while loop \n";
        }
        std::cout << "Exited the while loop \n";
        //=================================================================================

        optimal_path.info.resolution = map->info.resolution;         // float32
        optimal_path.info.width  = cols_;                            // uint32
        optimal_path.info.height = rows_;                            // uint32
        optimal_path.info.origin.position.x = map->info.origin.position.x;
        optimal_path.info.origin.position.y = map->info.origin.position.y;
        optimal_path.info.origin.position.z = map->info.origin.position.z;
        optimal_path.info.origin.orientation.x = map->info.origin.orientation.x;
        optimal_path.info.origin.orientation.y = map->info.origin.orientation.y;
        optimal_path.info.origin.orientation.z = map->info.origin.orientation.z;
        optimal_path.info.origin.orientation.w = map->info.origin.orientation.w;


        source_index = get_idx(source_row, source_col);
        dest_index = get_idx(dest_row, dest_col);
        prev[source_index] = 0;                                                // Adding the parent of source as 0.

        distances.at(source_index) = 0;                                         //Initializing source node distance as 0

        pq.push(std::make_pair(-1*(distances[source_index]),source_index));     // Added source node pair to priority queue.

        while(!pq.empty()){
            std::pair<float,int> current_node = pq.top();
            pq.pop();                                                           //Removing current node from priority queue.
            int current_node_distance = -1*current_node.first;
            int current_node_index = current_node.second;
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


                for(int i=0; i<8; i++)                                               //to calculate neighbours
                {
                    rr = r + dr[i];
                    cc = c + dc[i];

                    if(rr < 0 || rr >= rows_ || cc < 0 || cc >= cols_ || visited[get_idx(rr,cc)])
                        continue;

                    else
                    {
                        int neighbour_index = get_idx(rr, cc);
                        float neighbour_dist = 1.00;
                        if(i>3){
                            neighbour_dist = 1.414;                   // Cost of going diagonal is 1.4 times.
                        }
                                                // <==== At the moment the default edge distance between neighbouring cells
                                                // is set to 1
                                                                                
                            
                        
                        float alt = current_node_distance + neighbour_dist;      
                        if(alt < distances[get_idx(rr, cc)])
                        {
                            distances[get_idx(rr, cc)] = alt;
                            prev[get_idx(rr, cc)] = current_node_index;      // Updating parent of neighbour as the current node.
                            pq.push(std::make_pair(-1*(distances[get_idx(rr, cc)]),get_idx(rr, cc)));  // Pushing current neighbour in pq.
                        }
                        if(visited[dest_index]){
                            ROS_INFO("Reached goal \n");
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
        
            // prev contains the path. Trace it back to get the path.

        path.push_back(dest_index);
    
        while(true)
        {
            path.push_back(prev[path.back()]);                           // Adding the parent of each index to path starting from destination.
            if(path.back() == 0){
                path.pop_back();                                         
                break;
            }

        }
        int psize = path.size();
        ROS_INFO("Path size:  == %d \n", psize);
        std::vector<signed char> a(rows_*cols_,0);
        int asize = a.size();
        //ROS_INFO("a size=> %d \n", asize);
        int R,rrr;
        int C,ccc;
        for(int i=(path.size()-1);i>=0;i--)
        {
        
            //ROS_INFO("Path[i] = %d \n", path[i]);
            R = path[i]/cols_;
            C = path[i] - (R*cols_);
            a.at(path[i]) = 100;
            //ROS_INFO("Path: row,col = %d %d \n", R,C);
        }
        optimal_path.data = a;
        
        time_to_solve = ros::Time::now().toSec() - current_time; 
        ROS_INFO("Optimal path found using dijsktra's algorithm.\n");
        //ROS_INFO("Callback counter is: %d \n",callback_count);
        
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_test_node");
    ros::NodeHandle nh;
    dijkstra_planner planner = dijkstra_planner(&nh);
    ros::Duration(1).sleep();
    ros::spinOnce();
    std::cout << "Waiting for somebody to subscribe to /dijkstra_shortest_path topic. \n";
    while(planner.get_subscriber_count()==0){            // Will keep on waiting till Rviz subscribes to /shortest_path topic.
        planner.publish_path();
        std::cout << "....\n";
        ros::Duration(5).sleep();
    }
    std::cout << "Somebody subscribed to /dijkstra_shortest_path topic. \nPublishing one last time after 30 seconds and exiting.\n";
    ros::Duration(10).sleep();
    for(int i=0;i<10;i++){
        planner.publish_path();
        ros::Duration(0.5).sleep();
    }
    
    
    return 0;
}