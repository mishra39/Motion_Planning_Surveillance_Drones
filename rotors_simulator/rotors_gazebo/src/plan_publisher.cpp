#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <vector>

using namespace std;

bool sim_inProgress = false;

void ImuCallback(const sensor_msgs::ImuPtr& msg)
{
    sim_inProgress = true;    
}

class WaypointWithTime
{
    public:
        double wait_time;
        double yaw;
        Eigen::Vector3d pos;
        WaypointWithTime(): wait_time(0), yaw(0) {}
        WaypointWithTime(double wait_dur, float x, float y, float z, float yaw_in) :
        pos(x,y,z), yaw(yaw_in),wait_time(wait_dur) {}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_publisher");
    ros::NodeHandle nh;

    ROS_INFO("Started plan publisher");
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    if (args.size() !=2 && args.size() !=3)
    {
        ROS_ERROR("Usage: waypoint_publisher <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
        return -1;
    }

    vector<WaypointWithTime> pathPoints;
    ifstream infile(args.at(1).c_str());

    if (infile.is_open())
    {
        double wait_dur, x,y,z,yaw;
        while (infile >> wait_dur >> x >> y >> z >> yaw)
        {
            pathPoints.push_back(WaypointWithTime(wait_dur,x,y,z,yaw*(M_PI/180.0)));
        }
        infile.close();
        ROS_INFO("Loaded %d path points.", (int) pathPoints.size());
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open waypoint file \n");
        return -1;
    }

    // Subscribe to IMU to check if simulation is running
    ros::Subscriber sub = nh.subscribe("imu",10, &ImuCallback);
    ros::Publisher pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    
    ROS_INFO("Simulation getting ready...");
/*
    while(!sim_inProgress && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ros::Duration(30).sleep();
    ROS_INFO("Wait time finished. Begin publishing waypoints");
*/
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < pathPoints.size(); ++i)
    {
        WaypointWithTime& wp = pathPoints[i];
        Eigen::Vector3d pos_des(wp.pos.x(), wp.pos.y(), wp.pos.z());
        double yaw_des = wp.yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(pos_des, yaw_des, &trajectory_msg);
        ros::Duration(wp.wait_time).sleep();
        cout << "Publishing Waypoint " << i+1 <<  endl;
        cout << "{" << wp.pos.x() << "," << wp.pos.y() << "," << wp.pos.z() << ")" << endl;
        pub.publish(trajectory_msg);
        ros::Duration(1).sleep(); 
    }

    cout << "All waypoints published" << endl;

    ros::spinOnce();
    ros::shutdown();

    return 0;
}