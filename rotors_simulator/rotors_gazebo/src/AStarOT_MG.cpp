#include<iostream>
#include<algorithm>
#include<utility>
#include <vector>
#include <queue>
#include <map>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <fstream>
#include <istream>
#include <list>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/AbstractOccupancyOcTree.h>
#include <time.h>
#include <fstream>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace Eigen;

#define P1 73
#define P2 89

// Publish on the grid
struct Point3D
{
    int x, y, z;
};

struct PointWithCost
{
    int x, y, z;
    float cost;
};

struct SearchPoint
{
    Point3D pos;
    Point3D parent;
    float cost;
    float g_val;
    float h_val;
};
float aStartWt = 5;
class compareF
{
    public:
        int operator() (const SearchPoint& p1, const SearchPoint& p2)
        {
            return ((p1.g_val + aStartWt*p1.h_val) > (p2.g_val + aStartWt*p2.h_val));
        }
};

class compareD
{
    public:
        int operator() (const PointWithCost& p1, const PointWithCost& p2)
        {
            return ((p1.cost) < (p2.cost));
        }
};



void correctIdx(int &x, int &y, int &z, int x_size, int y_size, int z_size)
{
    if (x < 0)
    {
        x = abs(x) + x_size - 1;
    }

    if (y < 0)
    {
        y = abs(y) + y_size - 1;
    }

    if (z < 0)
    {
        z = abs(z) + z_size - 1;
    }
}

float calcHeuristic(Point3D point, Point3D goal)
{
    float eucDist = 0; // euclidean distance
    eucDist = pow((goal.x - point.x),2) + pow((goal.y - point.y),2) + pow((goal.z - point.z),2);
    eucDist = sqrt(eucDist);
    return eucDist;
}

void publish_path(vector<Point3D> finalPath)
{
    ros::NodeHandle nh;
    ROS_INFO("Started Final Path Publisher");

    ros::Publisher pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("drone_trajectory", 10);

    // Visualization settings
    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= "/world";
    sphere_list.header.stamp= ros::Time::now();
    //sphere_list.ns= "spheres";
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;
    sphere_list.id = 0;
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;

    // Points are green
    sphere_list.color.r = 1.0f;
    sphere_list.color.a = 1.0;

    ros::Rate loop_rate(10);
    geometry_msgs::Point p;
    for (auto i : finalPath)
    {
        p.x = i.x;
        p.y = i.y;
        p.z = i.z;
        sphere_list.points.push_back(p);
        Eigen::Vector3d pos_des(i.x,i.y, i.z);
        double yaw_des = 0.0;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(pos_des, yaw_des, &trajectory_msg);
      //  cout << "Publishing Points (x,y,z): " << i.x << ", " << i.y << ", " << i.z << endl;
        pub.publish(trajectory_msg);
        ros::Duration(1).sleep();
    }
    int ii  =0;
    while (ii < 10000)
    {
        marker_pub.publish(sphere_list);
        ii++;
    }
   cout << "All waypoints published" << endl;
}

// checks if a node is valid and unoccupied
bool validateNode(Point3D point, OcTree* tree)
{
    double x = point.x;
    double y = point.y;
    double z = point.z;
    
    OcTreeNode* node = tree->search(x, y, z, 0);
    
    if (node == 0 || tree->isNodeOccupied(node) == 0)
    {
        return true;
    }

    else 
    {
        return false;
    }
}

Point3D createImagGoal(vector<Point3D> goals)
{
    int max_x = INT_MIN;
    int max_y = INT_MIN;
    int max_z = INT_MIN;

    for (auto i : goals)
    {
        if (abs(i.x) > abs(max_x))
        
        {
            max_x = i.x;
        }

        if (abs(i.y) > abs(max_y))
        {
            max_y = i.y;
        }
        if (abs(i.z) > abs(max_z))
        {
            max_z = i.z;
        }
    }
    return {max_x,max_y,max_z};
}

// Backtrack from goal to start
void backtrack(Point3D currPos, Point3D start, Point3D (*parentNode)[72][72], int max_x, int max_y, int max_z)
{
    // Check if goal reached
    vector<Point3D> finalPath;
    printf("Backtracking from Goal \n");
    while (currPos.x != start.x || currPos.y != start.y || currPos.z != start.z)
    {
        int x_pos = currPos.x;
        int y_pos = currPos.y;
        int z_pos = currPos.z;
        correctIdx(x_pos, y_pos, z_pos, max_x, max_y, max_z);
        
        finalPath.push_back(currPos);
        currPos = parentNode[x_pos][y_pos][z_pos];
    }

    finalPath.push_back(start);
    reverse(finalPath.begin(),finalPath.end());
    // call the waypoint publisher
    publish_path(finalPath);
}

int getID(SearchPoint point)
{
    int a = point.pos.x;
    int b = point.pos.y;
    int c = point.pos.z;
    int h = (a*P1 + b)*P2 + c;
    return h;
}

void aStarPlanner(Point3D start, Point3D goal, OcTree* tree)
{
    ROS_INFO("Starting A* Planner"); 
    // Define map bounds
    int min_x = -36; 
    int max_x =  36;
    int min_y = -36;
    int max_y =  36;
    int min_z =   0;
    int max_z =  72;
    int x_size = abs(min_x) + max_x;
    int y_size = abs(min_y) + max_y;
    int z_size = abs(min_z) + max_z;
    double clearance = 0.50; // clearance from drone while planning
    SearchPoint startNode;
    startNode.pos = start;
    startNode.parent = {0,0,0};
    startNode.cost = 0.1;
    startNode.h_val = calcHeuristic(start,goal);
    startNode.g_val = 0.0;
    PointWithCost dir[10] = {{1,0,0,1},{0,1,0,1},{-1,0,0,1},{0,-1,0,1},{0,0,1,1},{0,0,-1,1},
                            {1,1,0,0.5},{-1,-1,0,0.5}, {1,-1,0,0.5},{-1,1,0,0.5}};

    // Data structures for algorithm
    priority_queue<SearchPoint, vector<SearchPoint>, compareF> open_list;
    unordered_map<int,SearchPoint> graph;
    
    double visited[x_size][y_size][z_size] = {0};

    Point3D currPos;
    float currCost;
    // Push start node to open list
    open_list.push(startNode);
    Point3D parentNode[72][72][72]; // parent node for each node on the map
    int count = 0;
    while (!open_list.empty())
    {
        SearchPoint front = open_list.top();
        open_list.pop();
        currPos = front.pos;
        if (currPos.x == start.x && currPos.y == start.y && currPos.z == start.z)
        {
            cout << "Start Node popped" << endl;
        }
        currCost = front.cost;
        int x_pos = front.pos.x;
        int y_pos = front.pos.y;
        int z_pos = front.pos.z;

        // convert negative idx to +ve for array indexing
        correctIdx(x_pos, y_pos, z_pos, max_x, max_y, max_z);
        // check if node has been visited already
        if (visited[x_pos][y_pos][z_pos] > 0.0)
        {
            continue;
        }
        visited[x_pos][y_pos][z_pos] = front.cost;
        parentNode[x_pos][y_pos][z_pos] = front.parent;
        int id = getID(front); 
        graph[id] = front; // Add searchpoint to the graph
        if (front.parent.x == start.x && front.parent.y == start.y && front.parent.z == start.z)
        {
            cout << "Start Node assigned as parent" << endl;
        }
        count++;

        // Check if a goal location is reached
        if (currPos.x == goal.x && currPos.y == goal.y && currPos.z == goal.z)
        {
            backtrack(currPos,start,parentNode,max_x,max_y,max_z);
            return ;
        }

        // Explore the neighboring nodes
        int new_x = 0;
        int new_y = 0;
        int new_z = 0;

        // Search the s-connected grid in 3D
        for (int i =0; i < 10; i++)
        {
            new_x = currPos.x + dir[i].x;
            new_y = currPos.y + dir[i].y;
            new_z = currPos.z + dir[i].z;

            // Validate that new position lies inside the map
            if(new_x<min_x || new_x >= max_x || new_y<min_y || new_y>=max_y || new_z<min_z || new_z>=max_z)
            {
                continue;
            }

            Point3D new_pos;
            new_pos.x = new_x;
            new_pos.y = new_y;
            new_pos.z = new_z;
            int new_x_arr = new_x;
            int new_y_arr = new_y;
            int new_z_arr = new_z;
            correctIdx(new_x_arr, new_y_arr, new_z_arr, max_x, max_y, max_z);
                        
            // Extract the node from the tree
            double temp_x = new_x;
            double temp_y = new_y;
            double temp_z = new_z;
            OcTreeNode* node = tree->search(temp_x, temp_y, temp_z, 0);
            int occupied = 0;

            if (node == 0 && visited[new_x_arr][new_y_arr][new_z] == 0) // if the node does not exist, assume it is unoccupied
            {
                // Check within 0.8 metres to make sure the drone has enough space to fly
                for (OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(Vector3 (new_x-clearance, new_y-clearance, new_z-clearance), 
                    Vector3 (new_x+clearance, new_y+clearance, new_z+clearance)), end = tree->end_leafs_bbx(); it != end; it++)
                {
                    occupied = tree->isNodeOccupied(*it);
                    if (occupied == 1)
                    {
                        break;
                    }
                }
                if (occupied  == 0)
                {
                    SearchPoint newNode;
                    newNode.pos = new_pos;
                    newNode.cost = dir[i].cost;
                    newNode.h_val = calcHeuristic(new_pos, goal);
                    newNode.g_val = currCost + dir[i].cost;
                    newNode.parent = currPos;
                    open_list.push(newNode);
                }
            }

            else if (visited[new_x_arr][new_y_arr][new_z] == 0 && tree->isNodeOccupied(node)==0)
            {
                for (OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(Vector3 (new_x-clearance, new_y-clearance, new_z-clearance), 
                    Vector3 (new_x+clearance, new_y+clearance, new_z+clearance)), end = tree->end_leafs_bbx(); it != end; it++)
                {
                    occupied = tree->isNodeOccupied(*it);
                    if (occupied == 1)
                    {
                        break;
                    }
                }
                if (occupied  == 0)
                {
                    SearchPoint newNode;
                    newNode.pos = new_pos;
                    newNode.cost = dir[i].cost;
                    newNode.h_val = calcHeuristic(new_pos, goal);
                    newNode.g_val = currCost + dir[i].cost;
                    newNode.parent = currPos;
                    open_list.push(newNode);
                }
            }

        }

    }
} 

typedef pair<int,Point3D> distGoal;
void aStar(Point3D start, vector<Point3D> goals, OcTree* tree)
{
    priority_queue<PointWithCost, vector<PointWithCost>, compareD > pq_goal; 
  
    PointWithCost currPoint;
    // arrange goals in order of distance from start location
    for (auto i: goals)
    {
        currPoint.x = i.x; currPoint.y = i.y; currPoint.z = i.z;
        currPoint.cost = calcHeuristic(start,i);
        pq_goal.push(currPoint);
    }

    Point3D currStart = start;
    Point3D currGoal;
    while (!pq_goal.empty())
    {
        currPoint = pq_goal.top();
        currGoal = {currPoint.x,currPoint.y,currPoint.z};
        pq_goal.pop();

        // Call A* for this goal
        cout << "Sending new start-goal pair" << endl;
        aStarPlanner(currStart,currGoal, tree);
        currStart = currGoal;
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AStarOT_MG");

    Point3D start = {0,0,1};
    Point3D goal  = {5,0,1};
    vector<Point3D> goals;
    goals.push_back(goal);
    goals.push_back({8,0,1});
    goals.push_back({11,7,4});
    // load the map
    AbstractOcTree* tree = AbstractOcTree::read("/home/akshit/obstaclecourse2.ot");
    OcTree* bt = dynamic_cast<OcTree*>(tree);

    // Validate start and goal locations
    bool startValid = validateNode(start, bt);
    bool goalValid  = validateNode(goal, bt);
    auto itr = goals.begin();
    for (auto i:goals)
    {
        bool checkGoal = validateNode(i,bt);
        if (checkGoal == false)
        {
            ROS_ERROR("Goal %d %d %d is not valid. Removing from goal list \n",i.x,i.y,i.z);
            goals.erase(itr);
        }
        else
        {
            itr++;
        }
    }
    if (startValid == false)
    {
        ROS_ERROR("Start Position is not valid/free");
        return 0;
    }

    else 
    {
        printf("Start position is valid \n");
    }
    
    if (goalValid == false)
    {
        ROS_ERROR("Goal Position is not valid/free");
        return 0;
    }

    else 
    {
        printf("Goal position is valid \n");
    }
    
    aStar(start, goals, bt);
    aStarPlanner(goal,start,bt);
      
    ros::spinOnce();
    ros::shutdown();
}