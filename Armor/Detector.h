#ifndef SCHOOLDETECT_DETECTOR_H
#define SCHOOLDETECT_DETECTOR_H

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;

class Detector {
public:
    Detector() = default;

    ~Detector() = default;

    Point2f run(const Mat &input);

    Point2f purple_point(const Mat &input);

    // 半画幅显示图片
    void _imgshow(const std::string& name, const Mat& img);
};

#endif //SCHOOLDETECT_DETECTOR_H
