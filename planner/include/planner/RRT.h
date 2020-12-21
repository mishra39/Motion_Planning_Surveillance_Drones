#pragma once

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
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <fstream>
#include <istream>
#include <list>
#include <octomap_msgs/Octomap.h>
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

using namespace std;
using namespace octomap;

class RRT {
    public:
        RRT(OcTree* map, double* start_pose, double* goal_pose);
        ~RRT();
        void BuildRRT();
        vector<vector<int>> RunRRT();
        void Run(); // sets the global plan g_plan
        vector<vector<int>> getGPlan();
        int getPlanlength();
        double getPlanCost();
        vector<vector<int>> getGraph();
        void Interrupt();

    protected:
        int run_time = 2;
        double PI = 3.141592654;
        static constexpr int numofDOFs = 4;
        OcTree* map; 
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
        double min_sample[numofDOFs];
        double max_sample[numofDOFs];
        double clearance = 0.50; // clearance from drone while planning
        int theta = 3;
        int sizes[numofDOFs];
        double* start_pose;
        double* goal_pose;
        default_random_engine random;
        bool goalBias = true; // whether or not to use goal biasing
        float goal_bias_value = 0.2;
        double eps = 8;
        double tol = 10; // yaw tolerance for goal
        time_t start, end;
        int interrupt = 0;
        struct Q {
            double pose[numofDOFs]; // 4 DOF (x, y, z, theta)
            float cost;
            Q* parent;
        };
        std::vector<Q*> V; // vector of pointers to vertices (serves as the map)
        int planlength = 0;
        double plancost = INFINITY;
        double bestcost = INFINITY;
        vector<vector<int>> g_plan;

        void correctIdx(int &x, int &y, int &z);       
        int GetMapIndex(int x, int y, int z);
        int IsValidPose(double* pose);
        int ClearPath(double* pose1, double* pose2);
        Q* Sample();
        int Same(Q* q1, Q* q2);
        Q* NearestNeighbor(Q* q);
        double EuclideanDist(double* q1_pose, double* q2_pose);
        int MoveUntilObstacle(Q* qnearest, Q* qsample, Q* qnew);
        void Extend(Q* q);
        double Cost(Q* q1, Q* q2);
        Q* ReachedGoal(Q* qgoal);
        stack<Q*> Interpolate(Q* qnear, Q* qfar);
        double MaxDist(Q* q1, Q* q2);
        vector<vector<int>> MakePlan(Q* qstart, Q* qgoal);  
};