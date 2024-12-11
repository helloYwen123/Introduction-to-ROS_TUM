#include "controller_pkg/state_estimator.h"

StateEstimator::StateEstimator(int BufferSize): buffer_size_{BufferSize} {

    current_state = nh.subscribe("/current_state_est", 1, &StateEstimator::onCurrentState, this);
    filtered_state_pub = nh.advertise<nav_msgs::Odometry>("/forward_state", 1);

    pos_buffer_.resize(buffer_size_, Vector3d::Zero());
    vel_buffer_.resize(buffer_size_, Vector3d::Zero());
    quaternion_buffer_.resize(buffer_size_);
    for (auto& quat : quaternion_buffer_) {
        quat = Quaterniond::Identity();
    }
    omega_buffer_.resize(buffer_size_, Vector3d::Zero());
}


void StateEstimator::onCurrentState(const nav_msgs::Odometry& cur_state){
    Vector3d position(cur_state.pose.pose.position.x,
                      cur_state.pose.pose.position.y,
                      cur_state.pose.pose.position.z);

    Vector3d velocity(cur_state.twist.twist.linear.x,
                      cur_state.twist.twist.linear.y,
                      cur_state.twist.twist.linear.z);

    current_quat_ = Quaterniond(cur_state.pose.pose.orientation.w,
                            cur_state.pose.pose.orientation.x,
                            cur_state.pose.pose.orientation.y,
                            cur_state.pose.pose.orientation.z);

    Vector3d angular_velocity(cur_state.twist.twist.angular.x,
                              cur_state.twist.twist.angular.y,
                              cur_state.twist.twist.angular.z);


    if (pos_buffer_.size() >= buffer_size_) pos_buffer_.pop_front();
    if (vel_buffer_.size() >= buffer_size_) vel_buffer_.pop_front();
    if (quaternion_buffer_.size() >= buffer_size_) quaternion_buffer_.pop_front();
    if (omega_buffer_.size() >= buffer_size_) omega_buffer_.pop_front();

    pos_buffer_.push_back(position);
    vel_buffer_.push_back(velocity);
    quaternion_buffer_.push_back(current_quat_);
    omega_buffer_.push_back(angular_velocity);

    publishFilteredState();
}


void StateEstimator::publishFilteredState() {
    Vector3d filtered_position = Vector3d::Zero();
    Vector3d filtered_velocity = Vector3d::Zero();
    Quaterniond filtered_orientation = Quaterniond::Identity();
    Vector3d filtered_angular_velocity = Vector3d::Zero();

    for (const auto& pos : pos_buffer_) {
        filtered_position += pos;
    }
    filtered_position /= pos_buffer_.size();

    for (const auto& vel : vel_buffer_) {
        filtered_velocity += vel;
    }
    filtered_velocity /= vel_buffer_.size();

    for (const auto& orient : quaternion_buffer_) {
        filtered_orientation = filtered_orientation.slerp(1.0 / quaternion_buffer_.size(), orient);
    }

    for (const auto& omega : omega_buffer_) {
        filtered_angular_velocity += omega;
    }
    filtered_angular_velocity /= omega_buffer_.size();

    // broadcast tf of filtered state
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "body_filtered";
    transformStamped.transform.translation.x = filtered_position.x();
    transformStamped.transform.translation.y = filtered_position.y();
    transformStamped.transform.translation.z = filtered_position.z();
    transformStamped.transform.rotation.x = current_quat_.x();
    transformStamped.transform.rotation.y = current_quat_.y();
    transformStamped.transform.rotation.z = current_quat_.z();
    transformStamped.transform.rotation.w = current_quat_.w();
    filtered_state_br.sendTransform(transformStamped);

    // publish the odometrya
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()));
    current_quat_ = rotation * current_quat_;

    nav_msgs::Odometry filtered_msg;
    filtered_msg.header.stamp = ros::Time::now();
    filtered_msg.header.frame_id = "world";
    filtered_msg.child_frame_id = "body_forward";
    filtered_msg.pose.pose.position.x = filtered_position.x();
    filtered_msg.pose.pose.position.y = filtered_position.y();
    filtered_msg.pose.pose.position.z = filtered_position.z();
    filtered_msg.pose.pose.orientation.x = current_quat_.x();
    filtered_msg.pose.pose.orientation.y = current_quat_.y();
    filtered_msg.pose.pose.orientation.z = current_quat_.z();
    filtered_msg.pose.pose.orientation.w = current_quat_.w();
    filtered_msg.twist.twist.linear.x = filtered_velocity.x();
    filtered_msg.twist.twist.linear.y = filtered_velocity.y();
    filtered_msg.twist.twist.linear.z = filtered_velocity.z();
    filtered_msg.twist.twist.angular.x = filtered_angular_velocity.x();
    filtered_msg.twist.twist.angular.y = filtered_angular_velocity.y();
    filtered_msg.twist.twist.angular.z = filtered_angular_velocity.z();

    // Leaving covariance empty (set to zero)
    std::fill(std::begin(filtered_msg.pose.covariance), std::end(filtered_msg.pose.covariance), 0.0);
    std::fill(std::begin(filtered_msg.twist.covariance), std::end(filtered_msg.twist.covariance), 0.0);

    filtered_state_pub.publish(filtered_msg);


}



int main(int argc, char** argv) {
    ros::init(argc, argv, "state_estimator_node");

    StateEstimator estimator(5);  // Buffer size of 10

    ros::spin();

    return 0;
}