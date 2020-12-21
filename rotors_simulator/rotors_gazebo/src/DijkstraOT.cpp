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

using namespace std;
using namespace octomap;
using namespace octomath;

// Point on the grid
struct Point3D
{
    int x, y, z;
};

struct PointwithCost
{
    int x, y, z;
    float cost;
};

struct SearchPoint
{
    float cost;
    Point3D locn; // location in the map
    Point3D parent;
};

class CostComp
{
    public:
    int operator()(const SearchPoint& p1, const SearchPoint& p2)
    {
        return (p1.cost > p2.cost);
    }
};

vector<Point3D> DijkstraPlanner(Point3D start, Point3D goal, OcTree* tree)
{
    // Define Size of the environment
    int min_x = -36;
    int max_x = 36;
    int min_y = -36;
    int max_y = 36;
    int min_z = 0;
    int max_z = 72;
    int x_size = abs(min_x) + max_x;
    int y_size = abs(min_y) + max_y;
    int z_size = abs(min_z) + max_z;

    Point3D currPos;
    Point3D prevPos;
    // direction of movements
    PointwithCost dir[6] = {{1,0,0,1},{0,1,0,1},{-1,0,0,1},{0,-1,0,1},{0,0,1,1},{0,0,-1,1}};
    // Array to keep track of visited points
    double visited[x_size][y_size][z_size] = {0};
    Point3D prev_point[x_size][y_size][z_size]; // array to keep track of what was the prev point
    // Min Priority queue with lowest cost point
    priority_queue<SearchPoint, vector<SearchPoint>, CostComp> pq;
    // Initialize queue with start point
    SearchPoint startPoint;
    startPoint.locn = start;
    startPoint.cost = 0.00001;
    startPoint.parent = {0,0,0};
    pq.push({.00001, start, {0,0,0}});

    while (!pq.empty())
    {
        // remove the node with min cost
        SearchPoint currNode = pq.top();
        pq.pop();
        
        float currCost = currNode.cost;
        currPos = currNode.locn;
        prevPos = currNode.parent;

        // Correct negative position values for indexing
        int x_arr = currPos.x;
        int y_arr = currPos.y;
        int z_arr = currPos.z;

        if (x_arr < 0)
        {
            x_arr = abs(x_arr) + max_x - 1;
        }

        if (y_arr < 0)
        {
            y_arr = abs(y_arr) + max_y - 1;
        }

        // Check if node has been visited already
        if (visited[x_arr][y_arr][z_arr] > 0)
        {
            continue;
        }

        // Mark node as visited
        visited[x_arr][y_arr][z_arr] = currCost;
        // Assign previous point
        prev_point[x_arr][y_arr][z_arr] = prevPos;

        // Check if goal position reached
        if (currPos.x == goal.x && currPos.y == goal.y && currPos.z == goal.z)
        {
            printf("Goal Found \n");
            break;
        }

        // Explore the neighbors
        int new_x = 0;
        int new_y = 0;
        int new_z = 0;

        for (int i = 0; i < 6; i++)
        {
            new_x = currPos.x + dir[i].x;
            new_y = currPos.y + dir[i].y;
            new_z = currPos.z + dir[i].z;

            // Validate the new position
            if(new_x<min_x || new_x >= max_x || new_y<min_y || new_y>=max_y || new_z<min_z || new_z>=max_z)
            {
                //printf("ERROR: Neighbor Outside the map. \n");
                continue;
            }
            
            Point3D new_pos;
            new_pos.x = new_x;
            new_pos.y = new_y;
            new_pos.z = new_z;
            int new_x_arr = new_x;
            int new_y_arr = new_y;

            if (new_x_arr < 0)
            {
                new_x_arr = abs(new_x_arr) + max_x - 1;
            }

            if (new_y_arr < 0)
            {
                new_y_arr = abs(new_y_arr) + max_y - 1;
            }

            // Lookup the node in octomap
            double temp_x = new_pos.x;
            double temp_y = new_pos.y;
            double temp_z = new_pos.z;
            OcTreeNode* node = tree->search(temp_x, temp_y, temp_z,0);
            int occupied = 0;

            // If the node does not exist (node==0), assume it is unoccupied
            if (node == 0 && visited[new_x_arr][new_y_arr][new_pos.z]==0)
            {
                // Check within 0.8 metres to make sure the drone has enough space to fly
                for(OcTree::leaf_bbx_iterator it = tree-> begin_leafs_bbx(Vector3 (new_pos.x-0.8,new_pos.y-0.8, new_pos.z-0.8), Vector3 (new_pos.x+ 0.8,new_pos.y+0.8, new_pos.z+0.8)), end = tree-> end_leafs_bbx(); it != end; ++it)
                {
                    occupied = tree->isNodeOccupied(*it);
                    if (occupied == 1)
                    {
                        break;
                    }
                }
                 // if none of the surrounding nodes are occupied, add node to the open list
                if (occupied == 0)
                {
                    pq.push({dir[i].cost + currCost,new_pos, currPos});
                }
            }

            // if current node is unoccupied and unvisited
            else if (visited[x_arr][y_arr][new_pos.z]== 0 && tree->isNodeOccupied(node)==0)
            {
                // Check within 0.8 metres 
                for (OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(Vector3 (new_pos.x-0.8, new_pos.y-0.8, new_pos.z-0.8),
                                                                          Vector3 (new_pos.x+0.8, new_pos.y+0.8, new_pos.z+0.8)), 
                                                                          end = tree->end_leafs_bbx(); it != end; ++it)
                {
                    occupied = tree->isNodeOccupied(*it);
                    if (occupied == 1)
                    {
                        break;
                    }
                }
                 // if none of the surrounding nodes are occupied, add node to the open list
                if (occupied == 0)
                {
                    pq.push({dir[i].cost + currCost,new_pos, currPos});
                }
            }
        }
    }
    // Final Path
    vector<Point3D> finalPath;
    if (currPos.x == goal.x && currPos.y == goal.y && currPos.z == goal.z)
    {
        printf("Goal Found \n");
        while (currPos.x != start.x || currPos.y != start.y || currPos.z!= start.z)
        {
            int x_arr = currPos.x;
            int y_arr = currPos.y;
            int z_arr = currPos.z;

            if (x_arr < 0)
            {
                x_arr = abs(x_arr) + max_x - 1;
            }

            if (y_arr < 0)
            {
                y_arr = abs(y_arr) + max_y - 1;
            }
            finalPath.push_back(currPos);
            currPos = prev_point[x_arr][y_arr][currPos.z];
        }
        finalPath.push_back(start);
        reverse(finalPath.begin(), finalPath.end());
    }
    else
    {
        printf("Goal out of reach\n");
    }
    return finalPath;
}

int main()
{
    Point3D start = {0,0,1}; // Start Position
    Point3D goal = {5,0,1}; // Goal Position

    // load the Octomap
    AbstractOcTree* tree = AbstractOcTree::read("/home/akshit/obstaclecourse2.ot");
    OcTree* bt = dynamic_cast<OcTree*>(tree);
    
    // Dijkstra Algorithm
    vector<Point3D> finalPath;
    finalPath = DijkstraPlanner(start,goal,bt);
    cout << "Size of final Path "<< finalPath.size() << endl;
    ofstream outfile("path_DijkstraOT.txt");
    for (int i=0; i<finalPath.size();i++)
    {
        outfile << "0 " << finalPath[i].x << " " << finalPath[i].y << " " << finalPath[i].z << " " << "0" << endl;
    }
    outfile.close();
}