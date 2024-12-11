#include "planner_pkg/wp_gen.h"
const double POSITION_TOLERANCE = 5.0; // 10 cm tolerance
const double YAW_TOLERANCE = 0.5; // 0.1 radian tolerance (~5.7 degrees)

double getYawFromPose(const geometry_msgs::Pose& pose) {
    tf2::Quaternion quat(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}


bool checkReach(Eigen::Vector3d& curr, Eigen::Vector3d& ref) {
    double dx = std::abs(curr[0] - ref[0]);
    double dy = std::abs(curr[1] - ref[1]);
    double dyaw = std::atan2(std::sin(curr[2] - ref[2]), std::cos(curr[2] - ref[2])); // Normalize yaw difference
    
    if (dx < POSITION_TOLERANCE && dy < POSITION_TOLERANCE && dyaw < YAW_TOLERANCE) {
        return true;
    }
    return false;
}