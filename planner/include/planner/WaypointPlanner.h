#pragma once

// #include "planner/RRT.h"
// #include "planner/RRTconnect.h"
#include "planner/RRTstar.h"

#include <math.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <array>
#include <stack>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <ctime>
#include <numeric>
#include <list>
#include <typeinfo>
#include <memory>
#include <random>
#include <cmath>
#include <thread>
#include <functional>
#include <utility>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/AbstractOccupancyOcTree.h>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;
using namespace octomap;

template <class T> 
class WaypointPlanner {
    public:
        WaypointPlanner(OcTree* map, double* start_pose, vector<double*> waypoints);
        ~WaypointPlanner();
        vector<vector<int>> GetNextWaypointPlan();
        double* GetNextWaypoint();
        vector<vector<vector<int>>> GetFullPlan();
        vector<double*> GetWaypointsOrder();
        vector<int> GetPlanLengths();
        double getCost();
        int getVisited();
    protected:
        int planner_id;
        octomap::OcTree* map = nullptr;
        int min_x = -36; 
        int max_x =  36;
        int min_y = -36;
        int max_y =  36;
        int min_z =   0;
        int max_z =  72;
        int x_size = abs(min_x) + max_x;
        int y_size = abs(min_y) + max_y;
        int z_size = abs(min_z) + max_z; 
        double* start_pose;
        static constexpr int numofDOFs = 4; 
        vector<double*> waypoints;
        vector<vector<int>> next_waypoint_plan;
        double* next_waypoint;
        vector<double*> waypoints_order;
        vector<int> plan_lengths;
        int next_waypoint_index;
        double totalcost = 0.0;
        int totalvisited = 0;

        vector<T*> InitPlanners();
        static void RunPlanner(T* planner, int index);
        pair<vector<vector<int>>, double*> RunAllPlanners();
};