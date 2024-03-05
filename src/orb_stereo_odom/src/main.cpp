#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

ros::Publisher pub_cam1;
ros::Publisher pub_cam2;

cv::Mat K_1 = (cv::Mat_<double>(3,3) << 286.38861083984375, 0.0, 421.372314453125, 0.0, 286.45220947265625, 390.54730224609375, 0.0, 0.0, 1.0);
cv::Mat D_1 = (cv::Mat_<double>(4,1) << -0.012872150167822838, 0.05464962124824524, -0.05153217166662216, 0.010507550090551376);

cv::Mat K_2 = (cv::Mat_<double>(3,3) << 286.74090576171875, 0.0, 420.4461975097656, 0.0, 286.7449035644531, 393.5614929199219, 0.0, 0.0, 1.0);
cv::Mat D_2 = (cv::Mat_<double>(4,1) << -0.008860611356794834, 0.044272929430007935, -0.04117792844772339, 0.0071413880214095116);

std::vector<int> compression_params;

void undistort(Mat& img, const Mat& K, const Mat& D);
void image_callback_cam1(const sensor_msgs::ImageConstPtr& msg);
void image_callback_cam2(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fisheye_calibration");
    ros::NodeHandle nh;

    pub_cam1 = nh.advertise<sensor_msgs::Image>("/camera/fisheye1/image_undistorted", 10);
    pub_cam2 = nh.advertise<sensor_msgs::Image>("/camera/fisheye2/image_undistorted", 10);

    ros::Subscriber sub_cam1 = nh.subscribe("/camera/fisheye1/image_raw", 10, image_callback_cam1);
    ros::Subscriber sub_cam2 = nh.subscribe("/camera/fisheye2/image_raw", 10, image_callback_cam2);

    ros::spin();

    return 0;
}

void undistort(Mat& img, const Mat& K, const Mat& D)
{
    Mat map1, map2;
    fisheye::initUndistortRectifyMap(K, D, Mat::eye(3, 3, CV_64F), K, Size(img.cols, img.rows), CV_16SC2, map1, map2);
    Mat undistorted_img;
    remap(img, undistorted_img, map1, map2, INTER_LINEAR);
    
    // Crop to the middle 400x400 region
    int crop_y = (undistorted_img.rows - 400) / 2;
    int crop_x = (undistorted_img.cols - 400) / 2;
    undistorted_img = undistorted_img(Rect(crop_x, crop_y, 400, 400));

    img = undistorted_img;
}

void image_callback_cam1(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try 
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    undistort(cv_ptr->image, K_1, D_1);

    sensor_msgs::ImagePtr undist_cam1_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub_cam1.publish(undist_cam1_msg);
}

void image_callback_cam2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    undistort(cv_ptr->image, K_2, D_2);

    sensor_msgs::ImagePtr undist_cam2_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    pub_cam2.publish(undist_cam2_msg);
}


