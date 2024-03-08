#include "orb_stereo_odom/frame.h"

namespace odom
{
    Frame::Frame()
    : id_(-1), timestamp_(-1), camera_(nullptr)
    {

    }

    Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
    : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
    {

    }

    Frame::~Frame()
    {

    }

    // not yet


}