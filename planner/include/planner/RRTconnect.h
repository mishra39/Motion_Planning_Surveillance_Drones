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

class RRTconnect {
    public:
        RRTconnect(OcTree* map, double* start_pose, double* goal_pose);
        ~RRTconnect();
        vector<vector<int>> RunRRT();
        void Run(); // sets the global plan g_plan
        vector<vector<int>> getGPlan();
        int getPlanlength();
        double getPlanCost();
        vector<vector<int>> getGraph();
        int getStartGraphSize();
        void Interrupt();

    protected:
        int runtime = 20;
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
        double* start_pose;
        double* goal_pose;
        double bestcost = INFINITY;
        default_random_engine random;
        bool goalBias = true; // whether or not to use goal biasing
        float goal_bias_value = 0.4;
        double eps = 8;
        double tol = 10; // yaw tolerance for goal
        time_t start, end;
        struct Q {
            double pose[numofDOFs]; // 4 DOF (x, y, z, theta)
            float cost;
            Q* parent;
        };
        vector<Q*> Vstart; // start map
        vector<Q*> Vgoal; // goal map
        int planlength = 0;
        double plancost = INFINITY;
        bool currmap = true; // true if start map, false if goal map
        int connected = 0; // have start and goal maps been connected
        vector<vector<int>> g_plan;
        int interrupt = 0;

        int startgraphsize = 0;

        void correctIdx(int &x, int &y, int &z);
        int GetMapIndex(int x, int y, int z);
        int IsValidPose(double* pose);
        int ClearPath(double* pose1, double* pose2);
        Q* Sample();
        int Same(Q* q1, Q* q2);
        Q* NearestNeighbor(Q* q, vector<Q*>* map);
        double EuclideanDist(double* q1_pose, double* q2_pose);
        int MoveUntilObstacle(Q* q1, Q* q2, Q* qnew, int bounded);
        double Cost(Q* q1, Q* q2);
        Q* Extend(Q* q, vector<Q*>* map, int bounded);
        Q* Connect(Q* q);
        stack<Q*> Interpolate(Q* qnear, Q* qfar);
        double MaxDist(Q* q1, Q* q2);
        Q* BuildRRT();
        vector<vector<int>> MakePlan(Q* qstart, Q* qgoal, Q* qconnect);
};