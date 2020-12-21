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

class RRTstar {
    public:
        RRTstar(OcTree* map, double* start, double* goal);
        ~RRTstar();
        vector<vector<int>> RunRRT();
        void Run(); // sets the global plan g_plan
        vector<vector<int>> getGPlan();
        int getPlanlength();
        double getPlanCost();
        vector<vector<int>> getGraph();
        // void Interrupt();

    protected:
        int runtime = 10;
        double PI = 3.141592654;
        int DOF = 9; // planar movements
        int dX[9] = {0, -1, -1, -1, 0, 0, 1, 1, 1};
        int dY[9] = {0, -1, 0, 1, -1, 1, -1, 0, 1};
        static constexpr int numofDOFs = 4;
        octomap::OcTree* map = nullptr;
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
        double clearance = 0.80; // clearance from drone while planning
        int theta = 3;
        int min_sample[numofDOFs] = {min_x, min_y, min_z, 0};
        int max_sample[numofDOFs] = {max_x, max_y, max_z, 180};
        double* start_pose;
        double* goal_pose;
        double bestcost = INFINITY;
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
        vector<vector<int>> g_plan;

        int GetMapIndex(int x, int y, int z);
        int IsValidPose(double* pose);
        // void correctIdx(int x, int y, int z, int x_size, int y_size, int z_size);
        int ClearPath(double* pose1, double* pose2);
        Q* Sample();
        int Same(Q* q1, Q* q2);
        Q* NearestNeighbor(Q* q);
        double EuclideanDist(double* q1_pose, double* q2_pose);
        int MoveUntilObstacle(Q* qnearest, Q* qsample, Q* qnew);
        double Cost(Q* q1, Q* q2);
        double GetRadius();
        Q* Steer(Q* qnearest, Q* qsample);
        vector<Q*> Near(Q* qnew, double rad);
        void Extend(Q* q);
        stack<Q*> Interpolate(Q* qnear, Q* qfar);
        double MaxDist(Q* q1, Q* q2);
        void BuildRRT();
        Q* FindBestNearGoalQ(Q* qgoal);
        vector<vector<int>> MakePlan(Q* qstart, Q* qgoal);
        
};