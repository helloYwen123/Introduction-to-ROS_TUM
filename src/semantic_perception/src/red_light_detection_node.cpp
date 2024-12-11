#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <perception_msgs/LightState.h> 
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class RedLightDetector {
public:
    RedLightDetector()
    : red_count_threshold_(1),
      consecutive_frames_(2),
      current_consecutive_(0),
      red_light_active_(false) // Track if red light is currently considered 'active'
    {
        // initialize subscriber and publisher
        semantic_sub_ = nh_.subscribe("/semantic_image", 1, &RedLightDetector::semanticCallback, this);
        rgb_sub_ = nh_.subscribe("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &RedLightDetector::rgbCallback, this);
        alert_pub_ = nh_.advertise<perception_msgs::LightState>("/traffic_light_state", 1);
    }

    void semanticCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!red_light_active_) { // Only process semantic images if we are not currently tracking a red light
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            //ROS_INFO("Semantic Callback!!!.");
            int red_count = countNonBlackPixels(cv_ptr->image);
            if (red_count > red_count_threshold_) {
                current_consecutive_++;
                if (current_consecutive_ >= consecutive_frames_) {
                    //ROS_INFO("Red Light Active!!!");
                    red_light_active_ = true; // Start checking RGB images
                    current_consecutive_ = 0; // Reset counter
                }
            } else {
                current_consecutive_ = 0; // Reset counter
            }
        }
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
        perception_msgs::LightState stop_signal;
        stop_signal.is_stop = false;  // Default to false
        stop_signal.header.stamp = msg->header.stamp;
        if (red_light_active_) { // Process RGB images only if red light is active
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Extract the right two-thirds of the image
            cv::Mat rgb_right = cv_ptr->image(cv::Rect(cv_ptr->image.cols * 1 / 5, 0, cv_ptr->image.cols * 3 / 5, cv_ptr->image.rows));

            // Check for red light in the RGB image
            bool red_light_detected = check_red_light(rgb_right);

            if (red_light_detected) {
                stop_signal.is_stop = true;  // Set to true if red light is detected
                // ROS_INFO("Red light detected: Stopping the car.");
            } else {
                // ROS_INFO("No red light detected: Car may continue.");
                red_light_active_ = false; // Red light no longer active, go back to scanning semantic images
            }
        }

        alert_pub_.publish(stop_signal);
        //ROS_INFO("SEND STATE OF CAR!!!.");
    }

    int countNonBlackPixels(const cv::Mat& image) {
        int count = 0;
        // traverse all row and columns
        for (int i = 0; i < image.rows; i++) {
            for (int j = 0; j < image.cols; j++) {
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                int r = pixel[2];
                int g = pixel[1];
                int b = pixel[0];
                // check pixel if is black
                if (r > 30 || g > 30 || b > 30) {
                    count++; 
                }
            }
        }
        return count;
    }


    bool check_red_light(const cv::Mat& image) {
        int count = 0;
        for (int i = 0; i < image.rows; i++) {
            for (int j = 0; j < image.cols; j++) {
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                int r = pixel[2];
                int g = pixel[1];
                int b = pixel[0];
                if (r > 250 && g < 110 && b < 110) {
                    count++;
                    if (count > 0) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber semantic_sub_, rgb_sub_;
    ros::Publisher alert_pub_;
    int red_count_threshold_;
    int consecutive_frames_, current_consecutive_;
    bool red_light_active_; // Indicates if we are in 'red light detection mode'
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_light_detector");
    RedLightDetector detector;
    ros::spin();
    return 0;
}
