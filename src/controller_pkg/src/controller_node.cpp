#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html

class controllerNode{
  ros::NodeHandle nh;

  ros::Subscriber current_state;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher car_commands;
  ros::Timer timer;

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  Eigen::Vector3d v_forward;

  double ackermann_cmd_steering_angle;
  double ackermann_cmd_vel;
  double wheelbase;

  double hz;             // frequency of the main control loop

public:
  controllerNode():hz(100.0), ackermann_cmd_steering_angle{0.0}, ackermann_cmd_vel{0.0}{
      
      current_state = nh.subscribe("/forward_state", 1, &controllerNode::onCurrentState, this);
      cmd_vel_sub = nh.subscribe("cmd_vel", 1, &controllerNode::onVelConvert, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
      wheelbase = nh.param("/move_base/TebLocalPlannerROS/wheelbase", 3.0);
  }
  

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();

    v_forward = R.transpose() * v;
    // Rotate omega
    omega = R.transpose()*omega;
  }

  void onVelConvert(const geometry_msgs::Twist &cmd_vel) {
    ackermann_cmd_vel = cmd_vel.linear.x;

    if (ackermann_cmd_vel == 0 || cmd_vel.angular.z == 0) {
      ackermann_cmd_steering_angle = 0.0;
    } else {
      ackermann_cmd_steering_angle = atan(wheelbase * cmd_vel.angular.z / ackermann_cmd_vel);
    }
  }


  // PID控制器增益
  double kp = 2.0;
  double ki = 0.1;  // 如果需要积分控制可以调整这个值
  double kd = 0.1;  // 如果需要微分控制可以调整这个值

  // PID控制器状态变量
  double prev_error_vel = 0.0;
  double integral_vel = 0.0;

  void controlLoop(const ros::TimerEvent& t) {
    // 计算速度误差
    double error_vel = ackermann_cmd_vel - v_forward[0];
    
    // 计算积分和微分
    integral_vel += error_vel * (1.0 / hz);
    double derivative_vel = (error_vel - prev_error_vel) * hz;
    
    // PID控制器输出
    double control_output_vel = kp * error_vel + ki * integral_vel + kd * derivative_vel;
    
    // 更新前一时刻误差
    prev_error_vel = error_vel;

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = control_output_vel; // 使用PID控制器计算的加速度命令
    msg.angular_velocities[1] = -ackermann_cmd_steering_angle * 1.5 ;  // Turning angle
    if (v_forward[0] == 0.0) {
      msg.angular_velocities[2] = 0.0;
    } else {
      msg.angular_velocities[2] = std::min((ackermann_cmd_vel-v_forward[0])/v_forward[0] * 0.0, 0.0);  // Braking
    }
    msg.angular_velocities[3] = 0;

    car_commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}

