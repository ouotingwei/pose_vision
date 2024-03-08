#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

ros::Publisher pub_cam1;
ros::Publisher pub_cam2;
ros::Publisher disparity_pub;
ros::Publisher pointcloud_pub;

cv::Mat img1, img2;

cv::Mat K_1 = (cv::Mat_<double>(3,3) << 286.38861083984375, 0.0, 421.372314453125, 0.0, 286.45220947265625, 390.54730224609375, 0.0, 0.0, 1.0);
cv::Mat D_1 = (cv::Mat_<double>(4,1) << -0.012872150167822838, 0.05464962124824524, -0.05153217166662216, 0.010507550090551376);

cv::Mat K_2 = (cv::Mat_<double>(3,3) << 286.74090576171875, 0.0, 420.4461975097656, 0.0, 286.7449035644531, 393.5614929199219, 0.0, 0.0, 1.0);
cv::Mat D_2 = (cv::Mat_<double>(4,1) << -0.008860611356794834, 0.044272929430007935, -0.04117792844772339, 0.0071413880214095116);

void undistort(cv::Mat& img, const cv::Mat& K, const cv::Mat& D);
void image_callback_cam1(const sensor_msgs::ImageConstPtr& msg);
void image_callback_cam2(const sensor_msgs::ImageConstPtr& msg);
void gen_disparity();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fisheye_calibration");
    ros::NodeHandle nh;

    disparity_pub = nh.advertise<sensor_msgs::Image>("/disparity_image", 10);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    pub_cam1 = nh.advertise<sensor_msgs::Image>("/camera/fisheye1/image_undistorted", 10);
    pub_cam2 = nh.advertise<sensor_msgs::Image>("/camera/fisheye2/image_undistorted", 10);

    ros::Subscriber sub_cam1 = nh.subscribe("/camera/fisheye1/image_raw", 10, image_callback_cam1);
    ros::Subscriber sub_cam2 = nh.subscribe("/camera/fisheye2/image_raw", 10, image_callback_cam2);

    ros::spin();

    return 0;
}

void undistort(cv::Mat& img, const cv::Mat& K, const cv::Mat& D)
{
    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), K, img.size(), CV_16SC2, map1, map2);
    cv::Mat undistorted_img;
    remap(img, undistorted_img, map1, map2, cv::INTER_LINEAR);

    // Crop to the middle 400x400 region
    int crop_y = (undistorted_img.rows - 400) / 2;
    int crop_x = (undistorted_img.cols - 400) / 2;
    undistorted_img = undistorted_img(cv::Rect(crop_x, crop_y, 400, 400));

    img = undistorted_img;
}

void image_callback_cam1(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Convert color image to grayscale
        cv::Mat img_gray;
        cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);

        // Undistort the grayscale image
        undistort(img_gray, K_2, D_2);

        // Assign the undistorted grayscale image to imgR
        img1 = img_gray;

        // Publish the undistorted image
        pub_cam2.publish(cv_ptr->toImageMsg());

        // Generate disparity
        gen_disparity();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void image_callback_cam2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::Mat img_gray;
        cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);

        undistort(img_gray, K_2, D_2);

        img2 = img_gray;

        pub_cam2.publish(cv_ptr->toImageMsg());

        gen_disparity();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void gen_disparity()
{
    if (!img1.empty() && !img2.empty()) 
    {
        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 11);
        cv::Mat disparity;
        stereo->compute(img1, img2, disparity);

        // Normalize the disparity image
        cv::Mat disparity_normalized;
        cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

        // Convert the normalized disparity image to sensor_msgs::Image
        sensor_msgs::ImagePtr disparity_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disparity_normalized).toImageMsg();

        // Publish the disparity image
        disparity_pub.publish(disparity_img_msg);
    }
}
