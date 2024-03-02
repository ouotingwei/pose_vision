/*
edited by Ting Wei Ou, Graduate Degree Program of Robotics, National Yang Ming Chiao Tung University
email: kklb1716@gmail.com
*/
#ifndef CAMERA_H
#define CAMERA_H

#include "orb_stereo_odom/common_include.h"

namespace orb_stereo_odom
{
    //pinhole stereo camera
    class StereoCamera
    {
        public:
            // Camera parameters for left camera
            float fx_left_, fy_left_, cx_left_, cy_left_;
            // Camera parameters for right camera
            float fx_right_, fy_right_, cx_right_, cy_right_;
            // Baseline and depth scale
            float baseline_, depth_scale_;

            StereoCamera(float fx_left, float fy_left, float cx_left, float cy_left,
                         float fx_right, float fy_right, float cx_right, float cy_right,
                         float baseline, float depth_scale=0) :
            fx_left_(fx_left), fy_left_(fy_left), cx_left_(cx_left), cy_left_(cy_left),
            fx_right_(fx_right), fy_right_(fy_right), cx_right_(cx_right), cy_right_(cy_right),
            baseline_(baseline), depth_scale_(depth_scale) {}

            // coordinate transform: world, camera, pixel
            Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);
            Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);
            Vector2d camera2pixel(const Vector3d &p_c);
            Vector3d pixel2camera(const Vector2d &p_p, double depth=1);
            Vector3d pixel2world (const Vector2d &p_p, const SE3 &T_c_w, double depth=1);
            Vector2d world2pixel (const Vector3d &p_w, const SE3 &T_c_w);
    };
}

#endif //CAMERA_H
