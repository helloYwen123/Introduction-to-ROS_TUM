#include <ros/ros.h>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

using Eigen::Vector3d;
using Eigen::Quaterniond;

class StateEstimator {

    ros::NodeHandle nh;
    ros::Subscriber current_state;
    ros::Publisher filtered_state_pub;
    tf2_ros::TransformBroadcaster filtered_state_br;

    int buffer_size_;
    std::deque<Vector3d> pos_buffer_;
    std::deque<Vector3d> vel_buffer_;
    std::deque<Quaterniond> quaternion_buffer_;
    Quaterniond current_quat_;
    std::deque<Vector3d> omega_buffer_;

    void onCurrentState(const nav_msgs::Odometry& cur_state);
    void publishFilteredState();

public:
    StateEstimator(int BufferSize=5);

};

