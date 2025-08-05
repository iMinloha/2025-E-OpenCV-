#ifndef _CAMERA_H
#define _CAMERA_H

#include "opencv2/opencv.hpp"

// 纯虚类
class Camera {
public:
    virtual void open() = 0;

    virtual void close() = 0;

    virtual cv::Mat capture() = 0;

    virtual bool capture(cv::Mat & mat) = 0;

    virtual void release() = 0;

    virtual bool isOpen() = 0;
};

#endif //HLJU_RM_CAMERA_H
