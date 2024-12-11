#ifndef _WAYPOINT_GENE_H_
#define _WAYPOINT_GENE_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>

std::vector<Eigen::Vector3d> waypoint{
    {12.28, -62.90, 3.14},       // start
    {4.33, -61.80, 1.57},      // enter curve
    {-4.33, -61.80, 1.57},      // enter curve
    {-8.33, -61.42, 1.57},      // enter curve
    {-18.33, -61.42, 1.50},      // enter curve
    {-27.33, -60.42, 1.33},      // enter curve
    {-41.33, -54.42, 0.70},      // enter curve
    {-46.33, -46.42, 0.40},      // enter curve
    {-49.14, -38.15, -0.03},    // exit curve
    {-52.60, -15.00, 0.00},      // map edge
    {-52.60, -1.00, 0.00},      // map edge
    {-52.40, 10.00, 0.00},      // map edge
    {-52.40, 22.10, 0.00},      // map edge
    {-52.40, 32.10, 0.00},      // map edge
    {-52.40, 44.00, 0.00},      // junction
    {-52.40, 55.10, 0.00},      // map edge
    {-52.40, 65.10, 0.00},      // map edge
    {-52.40, 75.10, 0.00},      // map edge
    {-52.40, 83.10, 0.00},      // map edge
    {-52.40, 90.10, 0.00},      // map edge
    {-52.40, 106.10, 0.00},      // map edge
    {-52.40, 121.00, 0.00},      // junction
    {-52.40, 133.10, 0.00},      // map edge
    {-52.40, 144.10, 0.00},      // map edge
    {-52.40, 156.10, 0.00},      // map edge
    {-52.40, 165.10, 0.00},      // map edge
    {-52.40, 181.10, 0.00},      // map edge
    {-50.80, 197.10, 0.00},      // enter curve
    {-48.80, 206.10, 0.00},      // enter curve
    {-45.80, 217.10, -0.35},      // enter curve
    {-39.80, 225.10, -1.35},      // enter curve
    {-28.80, 227.10, -1.45},      // enter curve
    {-15.60, 227.10, -1.57},     // exit curve, junction
    {-3.60, 210.10, -3.14},     // infront junction
    {-3.60, 195.10, -3.14},     // infront junction
    {-3.60, 180.10, -3.14},     // infront junction
    {-3.60, 165.10, -3.14},     // infront junction
    {-3.60, 150.10, -3.14},     // infront junction
    {-3.60, 135.10, -3.14},     // infront junction
    {-3.60, 125.00, -3.14},     // junction centre
    {-23, 125.00, 1.57},     // junction centre
    {-38, 125.00, 1.57},     // junction centre
    {-49, 125.00, 1.57},     // junction centre
    {-52.60, 120.10, -3.14},     // infront junction
    {-52.60, 100.10, -3.14},     // infront junction
    {-52.60, 80.10, -3.14},     // infront junction
    {-52.60, 54.10, -3.14},     // infront junction
    {-11.60, 46.10, -1.57},     // infront junction
    {-2.00, -48.10, -3.14},     // infront junction
    {12.28, -62.90, 1.57},       // end
                                            };


class WaypointPublisher {

    ros::NodeHandle nh;

    ros::Timer goalCheckTimer_;
    ros::Subscriber fsmSub_;
    ros::Subscriber currPosSub_;
    ros::Publisher goalPub_;

    // receive service by the state machine, switch between DRIVE and STOP state
    void onStateMachine(const std_msgs::Bool::ConstPtr& msg);
    void publishGoal(const ros::TimerEvent& t);
    void onCurrentState(const nav_msgs::Odometry&);


    bool flag_stop_ = true;
    int waypoint_count_;
    Eigen::Vector3d current_wp_;
    Eigen::Vector3d current_pos_;

public:
    WaypointPublisher();
};


#endif