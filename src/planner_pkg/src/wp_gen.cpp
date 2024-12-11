#include "planner_pkg/wp_gen.h"
#include "planner_pkg/utils.h"


WaypointPublisher::WaypointPublisher(): waypoint_count_{0}, current_wp_{waypoint[waypoint_count_]}, flag_stop_{true} {
    
    fsmSub_ = nh.subscribe("/drive_state", 1, &WaypointPublisher::onStateMachine, this);
    goalPub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, this);
    currPosSub_ = nh.subscribe("/forward_state", 1, &WaypointPublisher::onCurrentState, this);
    goalCheckTimer_ = nh.createTimer(ros::Rate(20), &WaypointPublisher::publishGoal, this);

    for (auto& wp: waypoint) {
        wp[2] += 1.57;
    }
}

void WaypointPublisher::onStateMachine(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && !flag_stop_) {
        flag_stop_ = true;
        ROS_INFO("I STOP");
    } else if (!msg->data && flag_stop_) {
        flag_stop_ = false;
        ROS_INFO("I DRIVE.");
    }
}


void WaypointPublisher::onCurrentState(const nav_msgs::Odometry& cur_state){
    double yaw = getYawFromPose(cur_state.pose.pose);
    current_pos_ << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y, yaw;
}


void WaypointPublisher::publishGoal(const ros::TimerEvent& t) {
    if (flag_stop_ || waypoint_count_ > waypoint.size()) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "world";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = current_pos_[0];
        goal.target_pose.pose.position.y = current_pos_[1];

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_pos_[2]);
        goal.target_pose.pose.orientation = tf2::toMsg(q);
        goalPub_.publish(goal);
        return; 
    }
    if (checkReach(current_pos_, current_wp_)) {
        waypoint_count_++;
        current_wp_ = waypoint[waypoint_count_];
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = current_wp_[0];
    goal.target_pose.pose.position.y = current_wp_[1];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_wp_[2]);
    goal.target_pose.pose.orientation = tf2::toMsg(q);
    goalPub_.publish(goal);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_node");
  WaypointPublisher wp_publisher;
  ros::spin();
}
