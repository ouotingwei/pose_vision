#ifndef FRAME_H
#define FRAME_H

#include "orb_stereo_odom/common_include.h"
#include " orb_stereo_odom/camera.h"

namespace odom
{
    class Frame
    {
        public:
            typrdef std::shared_ptr<Frame> Ptr;
            usigned long                   id_;             // id of this frame
            double                         time_stamp_;     // when it is recorded
            SE3                            T_c_w_;          // transform from world to camera
            Camera::Ptr                    camera_;         // pinhole model ( left camera )
            Mat                            color_, depth_;  //  color & depth image
    
        public: // data members
            Frame();
            Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
            ~Frame();

            // factory function
            static Frame::Ptr createFrame();

            // find the depth in depth map
            fouble findDepth( const cv::KeyPoint &kp );

            // get camera center
            Vector3d getCamCenter() const;

            // check if a point is in this frame
            bool isInFrame( const Vector3d &pt_world );
    };
}

#endif // FRAME_H