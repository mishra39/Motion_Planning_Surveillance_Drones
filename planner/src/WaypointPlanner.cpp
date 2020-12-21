#include "planner/WaypointPlanner.h"
#include "planner/RRTstar.h"

using namespace std;
using namespace octomap;

template <class T>
WaypointPlanner<T>::WaypointPlanner(octomap::OcTree* map, double* start_pose, vector<double*> waypoints) {
    this->map = map;
    this->start_pose = start_pose;
    this->waypoints = waypoints;
}

template <class T>
WaypointPlanner<T>::~WaypointPlanner() {

}

template <class T>
vector<T*> WaypointPlanner<T>::InitPlanners() {
    vector<T*> planners;
    for (auto goal_pose : waypoints) {
        T* planner = new T(map, start_pose, goal_pose);
        planners.push_back(planner);
    }
    return planners;
}

template <class T>
void WaypointPlanner<T>::RunPlanner(T* planner, int index) {
    cout << "Thread " << index << " started." << endl;
    planner->Run();
    cout << "Thread " << index << " stopped." << endl;
}

template <class T>
pair<vector<vector<int>>, double*> WaypointPlanner<T>::RunAllPlanners() {
    vector<T*> planners = InitPlanners();

    vector<thread> threadsvec;
    vector<vector<vector<int>>*> plans;
    int index = 1;
    for (auto planner : planners) {
        threadsvec.push_back(thread(&RunPlanner,planner,index));
        // planner->Run(); // speed comparison testing
        index ++;
    }
    for (auto& t : threadsvec) {
        t.join();
    }

    int j = 0;
    double mincost = INFINITY;
    int mincostidx = 0;
    for (auto planner : planners) {
        totalvisited += planner->getGraphSize();
        // costs.push_back(planner->getPlanCost());
        if (planner->getPlanCost() < mincost) {
            mincost = planner->getPlanCost();
            mincostidx = j;
        }
        j ++;
    }
    // int mincostidx = rand() % planners.size();
    totalcost += planners[mincostidx]->getPlanCost();
    // totalvisited += planners[mincostidx]->getGraphSize();
    next_waypoint_index = mincostidx;
    vector<vector<int>> plan = planners[mincostidx]->getGPlan();
    return make_pair(plan,waypoints[mincostidx]);
}

template <class T>
vector<vector<int>> WaypointPlanner<T>::GetNextWaypointPlan() {
    auto waypoint_pair = RunAllPlanners();
    next_waypoint_plan = waypoint_pair.first;
    next_waypoint = waypoint_pair.second;
    return next_waypoint_plan;
}

template <class T>
double* WaypointPlanner<T>::GetNextWaypoint() {
    return next_waypoint;
}

template <class T>
vector<vector<vector<int>>> WaypointPlanner<T>::GetFullPlan() {
    int nwaypoints = waypoints.size();
    vector<vector<vector<int>>> fullplan;
    for (int n = 0; n < nwaypoints; n++) {
        auto next = RunAllPlanners();
        auto next_plan = next.first;
        auto next_pose = next.second;
        plan_lengths.push_back(next_plan.size());
        fullplan.push_back(next_plan);
        waypoints_order.push_back(next_pose);
        this->waypoints.erase(this->waypoints.begin() + next_waypoint_index);
        this->start_pose = next_pose;
    }
    cout << "Total Cost: " << totalcost << endl;
    return fullplan;
}

template <class T>
vector<double*> WaypointPlanner<T>::GetWaypointsOrder() {
    return waypoints_order;
}

template <class T>
vector<int> WaypointPlanner<T>::GetPlanLengths() {
    return plan_lengths;
}

template <class T>
double WaypointPlanner<T>::getCost() {
    return totalcost;
}

template <class T>
int WaypointPlanner<T>::getVisited() {
    return totalvisited;
}