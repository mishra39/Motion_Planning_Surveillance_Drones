#include <visualization_msgs/Marker.h>
#include "planner/RRTstar.h"
#include "planner/WaypointPlanner.h"
#include "WaypointPlanner.cpp"

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace Eigen;

struct CompareDist {
  double* start;
  CompareDist(double* _start) {
    this->start = _start;
  }
  bool operator()(double* goal1, double* goal2) {
    double cost1 = 0.0;
    double cost2 = 0.0;
    for (int i = 0; i < 3; i++) {
      cost1 += pow(goal1[i]-start[i], 2);
      cost2 += pow(goal2[i]-start[i], 2);
    }
    return (cost1 > cost2);
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("drone_trajectory", 10);

    // Visualization settings
    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= "/my_frame";
    sphere_list.header.stamp= ros::Time::now();
    sphere_list.ns= "spheres";
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

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    ros::Rate loop_rate(10);

    vector<vector<int>> plan;
    string mapfilename = "/home/cerlab-controls/Desktop/catkin_ws/src/planner/map/town.ot";
    AbstractOcTree* tree = AbstractOcTree::read(mapfilename);
    OcTree* bt = dynamic_cast<OcTree*>(tree);
    if ((bt == NULL) || (tree == NULL)) {
      ROS_INFO("No map file loaded.");
      return 0;
    }
    ROS_INFO("Loaded Map");

    auto mapsize = bt->size();
    cout << "map size " << mapsize << endl;
    int occ = 0;
    for(OcTree::leaf_iterator it = bt->begin_leafs(),
       end=bt->end_leafs(); it!= end; ++it) { 
      occ += bt->isNodeOccupied(*it);
    }
    cout << "num occupied " << occ << endl;

// ************* TOWN MAP GOALS *************//
    double start[] = {0, 0, 1, 0};
    double goal1[] = {35, -5, 15, 0};
    double goal2[] = {35, -10, 15, 0};
    double goal3[] = {-25, 30, 15, 0};
    double goal4[] = {-30, 20, 15, 0};
    double goal5[] = {0, 0, 1, 0};

//****************** ARDALAN MAP GOALS *************//
    // double start[] = {0,0,1,0};
    // double goal1[] = {4,2,5,0};
    // double goal2[] = {5,-1,6,0};
    // double goal3[] = {2,-3,6,0};
    // double goal4[] = {2,-3,3,0};

    vector<double*> waypoints;
    waypoints.push_back(goal1);
    waypoints.push_back(goal2);
    waypoints.push_back(goal3);
    waypoints.push_back(goal4);

    clock_t starttime = clock();
    ROS_INFO("Started RRT Planner Object");
    int totalvisited = 0;
    double totalcost = 0.0;

    int waypointplan = 0;
    int eucdist = 1;
    int randomorder = 2;

    int order = 0;

    if (order == waypointplan) {
    //************* Waypoint Planner *************//
      cout << "------ Waypoint Planner ------" << endl;
      WaypointPlanner<RRTstar>* planner = new WaypointPlanner<RRTstar>(bt, start, waypoints);
      vector<vector<vector<int>>> fullplan;
      fullplan = planner->GetFullPlan();
      for (int i = 0; i < fullplan.size(); i++) {
        for (int j = 0; j < fullplan[i].size(); j++) {
          plan.push_back(fullplan[i][j]);
        }
        for (int k = 0; k < 4; k++) {
        cout << plan.back()[k] << ", ";
        }
      cout << endl;
      }
      totalvisited += planner->getVisited();
      totalcost += planner->getCost();
      auto waypointorder = planner->GetWaypointsOrder();
      RRTstar* homeplanner = new RRTstar(bt, waypointorder.back(), start);
      vector<vector<int>> homeplan;
      homeplan = homeplanner->RunRRT();
      for (auto p : homeplan) {
        plan.push_back(p);
      }
      totalvisited += homeplanner->getGraphSize();
      totalcost += homeplanner->getPlanCost();
    }

    else if (order == eucdist) {
    //************* Euclidean Distance **************//
      cout << "------ Euclidean Distance ------" << endl;
      priority_queue<double*, vector<double*>, CompareDist> pq{CompareDist(start)};
      pq.push(goal1);
      pq.push(goal2);
      pq.push(goal3);
      pq.push(goal4);
      double current[] = {start[0],start[1],start[2],start[3]};
      while (!pq.empty()) {
        RRTstar* planner = new RRTstar(bt, current, pq.top());
        auto starplan = planner->RunRRT();
        totalvisited += planner->getGraphSize();
        totalcost += planner->getPlanCost();
        for (auto p : starplan) {
          plan.push_back(p);
        }
        for (int i = 0; i < 4; i++) {
          current[i] = pq.top()[i];
          cout << current[i] << ", ";
        }
        cout << endl;
        pq.pop();
      }
      RRTstar* homeplanner = new RRTstar(bt, current, start);
      auto homeplan = homeplanner->RunRRT();
      totalvisited += homeplanner->getGraphSize();
      totalcost += homeplanner->getPlanCost();
      for (auto p : homeplan) {
        plan.push_back(p);
      }
    }

    else if (order == randomorder) {
      // //*************** Random Order *****************//
      cout << "------ Random Order ------" << endl;
      srand(time(NULL));
      random_shuffle(waypoints.begin(),waypoints.end());
      waypoints.push_back(start);
      double current[] = {start[0],start[1],start[2],start[3]};
      for (auto goal : waypoints) {
        RRTstar* planner = new RRTstar(bt,current,goal);
        auto starplan  = planner->RunRRT();
        totalvisited += planner->getGraphSize();
        totalcost += planner->getPlanCost();
        for (auto p : starplan) {
          plan.push_back(p);
        }
        for (int i = 0; i < 4; i++) {
          current[i] = goal[i];
          cout << starplan.back()[i] << ", ";
        }
        cout << endl;
      }
    }



    clock_t endtime = clock();
    double totaltime = double(endtime - starttime) / (CLOCKS_PER_SEC/1000);
    cout << endl << "----------------------" << endl;
    cout << "Total Visited: " << totalvisited << endl;
    cout << "Total Cost: " << totalcost << endl;
    cout << "Planning Time: " << totaltime << " ms." << endl;
    ROS_INFO("After Run");

    if(plan.empty())
      ROS_INFO("No plan was found");
    
    double yaw_des = 0.0;
    geometry_msgs::Point p;

    // for(auto &waypoint : plan)
    // {
    //   auto x = waypoint[0];
    //   auto y = waypoint[1];
    //   auto z = waypoint[2];
    //   p.x = x;
    //   p.y = y;
    //   p.z = z;
    //   sphere_list.points.push_back(p);
    //   Eigen::Vector3d pos_des(x, y, z);
    //   mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(pos_des, yaw_des, &trajectory_msg);
    //   pub.publish(trajectory_msg);
    //   // marker_pub.publish(sphere_list);
    //   ros::Duration(1).sleep();
    // }
  //   while (ros::ok())
  // {
  //   marker_pub.publish(sphere_list);
  // }
  return 0;
}
